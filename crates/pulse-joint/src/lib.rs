//! The single-joint control logic with no Copper, no std and no allocation, so the exact same code runs in the
//! host simulation and on the microcontroller. Copper tasks are thin wrappers around these types.

#![no_std]

pub mod params;

use multicalc::control::Pid;
use multicalc::error::ControlError;
use multicalc::prelude::*;
use multicalc::random::Pcg32;
use params::*;

pub const NOMINAL: u8 = 0;
pub const DERATING: u8 = 1;
pub const FAULT: u8 = 2;

fn quantize(x: f32, step: f32) -> f32 {
    (x / step).round() * step
}

/// What the sensor block reports: quantized, delayed, possibly a repeat of the last good reading (`valid: false`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Reading {
    pub theta: f32,
    pub current: f32,
    pub temp: f32,
    pub sampled_at: u64,
    pub valid: bool,
}

const RING: usize = LATENCY_TICKS as usize + 1;

/// Sensor block: fixed latency (ring buffer), quantization, and dropout bounded by `MAX_DROPOUT_RUN`.
pub struct SensorModel {
    tick: u64,
    ring: [[f32; 3]; RING],
    rng: Pcg32<f32>,
    dropout_run: u32,
    last: Reading,
}

impl SensorModel {
    pub fn new(initial_temp: f32) -> Self {
        Self {
            tick: 0,
            ring: [[0.0, 0.0, initial_temp]; RING],
            rng: Pcg32::new(0xC0FFEE),
            dropout_run: 0,
            last: Reading::default(),
        }
    }

    /// One base tick: push the true `[theta, current, temp]`, return what the sensor reports now.
    pub fn sample(&mut self, truth: [f32; 3]) -> Reading {
        self.ring[self.tick as usize % RING] = truth;
        let delayed = self.ring[(self.tick as usize + 1) % RING]; // LATENCY_TICKS ticks old
        if self.dropout_run < MAX_DROPOUT_RUN && self.rng.next_unit() < DROPOUT_P {
            self.dropout_run += 1;
            self.last.valid = false;
        } else {
            self.dropout_run = 0;
            self.last = Reading {
                theta: quantize(delayed[0], POS_QUANT),
                current: quantize(delayed[1], CUR_QUANT),
                temp: quantize(delayed[2], TEMP_QUANT),
                sampled_at: self.tick.saturating_sub(LATENCY_TICKS as u64),
                valid: true,
            };
        }
        self.tick += 1;
        self.last
    }
}

/// Position PID (200 Hz): held position -> current setpoint.
pub struct PositionCtl(Pid<f32>);

impl PositionCtl {
    pub fn new() -> Result<Self, ControlError> {
        Ok(Self(
            Pid::new(POS_KP, POS_KI, POS_KD, OUTER_DT)?.with_output_limits(-I_MAX, I_MAX)?,
        ))
    }
    pub fn update(&mut self, setpoint: f32, theta: f32) -> f32 {
        self.0.update(setpoint, theta)
    }
}

/// Thermal state machine (200 Hz): nominal -> derating -> fault, fault latched, hysteresis on recovery.
pub struct ThermalFsm {
    pub state: u8,
}

impl ThermalFsm {
    pub fn new() -> Self {
        Self { state: NOMINAL }
    }
    pub fn update(&mut self, temp: f32) -> u8 {
        self.state = match self.state {
            _ if temp > T_FAULT => FAULT,
            FAULT => FAULT,
            NOMINAL if temp > T_DERATE => DERATING,
            DERATING if temp < T_RECOVER => NOMINAL,
            s => s,
        };
        self.state
    }
    /// Current-limit scale for a state.
    pub fn scale(state: u8) -> f32 {
        match state {
            NOMINAL => 1.0,
            DERATING => DERATE_SCALE,
            _ => 0.0,
        }
    }
}

impl Default for ThermalFsm {
    fn default() -> Self {
        Self::new()
    }
}

/// Current PI (8 kHz): clamped setpoint -> volts. Fault forces zero volts and clears the integrator.
pub struct CurrentCtl(Pid<f32>);

pub struct CurrentOut {
    pub volts: f32,
    /// The setpoint actually used, after the thermal clamp.
    pub setpoint: f32,
}

impl CurrentCtl {
    pub fn new() -> Result<Self, ControlError> {
        Ok(Self(
            Pid::new(CUR_KP, CUR_KI, 0.0, DT as f32)?.with_output_limits(-V_BUS, V_BUS)?,
        ))
    }
    pub fn update(&mut self, wanted: f32, measured: f32, scale: f32, state: u8) -> CurrentOut {
        let lim = I_MAX * scale;
        let setpoint = wanted.max(-lim).min(lim); // not clamp(): that panics on NaN bounds
        let volts = if state == FAULT {
            self.0.reset();
            0.0
        } else {
            self.0.update(setpoint, measured)
        };
        CurrentOut { volts, setpoint }
    }
}
