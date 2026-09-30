//! The single-joint control logic with no Copper, no std and no allocation, so the exact same code runs in the
//! host simulation and on the microcontroller. Copper tasks are thin wrappers around these types.

#![no_std]

// Generated from the IR, never hand-edited. Parentheses fix evaluation order; `x.max(lo).min(hi)` and `(1 - 1) * x`
// are deliberate (`clamp` panics on NaN bounds; `0 * x` is NaN when `x` is), so lints that would rewrite them are off.
#[rustfmt::skip]
#[allow(unused_parens, clippy::double_parens, clippy::manual_clamp, clippy::eq_op)]
pub mod generated;
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
            _ if temp > T_FAULT || temp.is_nan() => FAULT, // a broken (NaN) sensor faults too
            FAULT => FAULT,
            NOMINAL if temp > T_DERATE => DERATING,
            DERATING if temp < T_RECOVER => NOMINAL,
            s => s,
        };
        self.state
    }
    /// Current-limit scale for a state. Unknown states get 0 (fail safe).
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
/// Non-finite inputs fail safe: a NaN setpoint or scale commands 0 A, never an unclamped or full-scale current.
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
        // Not clamp(): it panics on NaN bounds. Not bare max/min either: f32::max(NaN, x) == x, so a NaN setpoint
        // would come out as -lim (full reverse current). NaN fails every comparison, so these map it to 0.
        let lim = if scale >= 0.0 {
            I_MAX * scale.min(1.0)
        } else {
            0.0
        };
        let setpoint = if wanted.is_finite() {
            wanted.max(-lim).min(lim)
        } else {
            0.0
        };
        let volts = if state == FAULT {
            self.0.reset();
            0.0
        } else {
            self.0.update(setpoint, measured)
        };
        CurrentOut { volts, setpoint }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TEMPS: [f32; 9] = [
        f32::NEG_INFINITY,
        25.0,
        T_RECOVER - 0.1,
        T_RECOVER + 0.1,
        T_DERATE + 0.1,
        T_FAULT,
        T_FAULT + 0.1,
        f32::INFINITY,
        f32::NAN,
    ];

    /// Every (state, temperature class) pair: fault is absorbing, NaN and over-temperature always fault.
    #[test]
    fn thermal_fsm_transition_table() {
        for from in [NOMINAL, DERATING, FAULT] {
            for t in TEMPS {
                let to = ThermalFsm { state: from }.update(t);
                let want = if from == FAULT || t > T_FAULT || t.is_nan() {
                    FAULT
                } else if from == NOMINAL && t > T_DERATE {
                    DERATING
                } else if from == DERATING && t < T_RECOVER {
                    NOMINAL
                } else {
                    from
                };
                assert_eq!(to, want, "state {from} at {t}");
            }
        }
    }

    #[test]
    fn setpoint_always_within_thermal_limit() {
        let odd = [
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            -1e30,
            -9.0,
            -0.5,
            0.0,
            3.0,
            9.0,
            1e30,
        ];
        for wanted in odd {
            for scale in [f32::NAN, -1.0, 0.0, DERATE_SCALE, 1.0, 2.0, f32::INFINITY] {
                let mut c = CurrentCtl::new().unwrap();
                let out = c.update(wanted, 0.0, scale, NOMINAL);
                let lim = if scale >= 0.0 {
                    I_MAX * scale.min(1.0)
                } else {
                    0.0
                };
                assert!(
                    out.setpoint.abs() <= lim,
                    "wanted {wanted} scale {scale} -> {}",
                    out.setpoint
                );
                if !wanted.is_finite() {
                    assert_eq!(out.setpoint, 0.0);
                }
            }
        }
        assert_eq!(
            CurrentCtl::new()
                .unwrap()
                .update(5.0, 0.0, 1.0, FAULT)
                .volts,
            0.0
        );
    }

    /// The contract `pulse-ir` derives staleness from: fixed latency, dropout runs never longer than declared.
    #[test]
    fn sensor_latency_and_dropout_bound() {
        let mut s = SensorModel::new(25.0);
        let (mut run, mut longest, mut drops) = (0, 0, 0);
        for tick in 0..1_000_000u64 {
            let r = s.sample([tick as f32 * 1e-6, 0.0, 25.0]);
            if r.valid {
                run = 0;
                assert_eq!(r.sampled_at, tick.saturating_sub(LATENCY_TICKS as u64));
                assert_eq!(r.theta, quantize(r.sampled_at as f32 * 1e-6, POS_QUANT));
            } else {
                run += 1;
                drops += 1;
                longest = longest.max(run);
            }
        }
        assert_eq!(longest, MAX_DROPOUT_RUN, "bound reached but never exceeded");
        assert!(
            (15_000..25_000).contains(&drops),
            "dropout rate ~{DROPOUT_P}: {drops}"
        );
    }

    /// The firmware generated from the IR is bit-identical to this hand-written controller, including NaN,
    /// infinities, rail saturation and fault resets. The hand-written one stays as the oracle until it is retired.
    #[test]
    fn generated_current_loop_matches_hand_written() {
        let mut rng = Pcg32::new(7);
        let odd = [
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            -0.0,
            0.0,
            1e30,
            -1e30,
            8.0,
            -8.0,
            0.75,
        ];
        let pick = |rng: &mut Pcg32<f32>, odd: &[f32]| {
            if rng.next_unit() < 0.1 {
                odd[(rng.next_unit() * odd.len() as f32) as usize % odd.len()]
            } else {
                rng.next_unit() * 20.0 - 10.0
            }
        };
        for run in 0..200 {
            let (mut hand, mut generated) =
                (CurrentCtl::new().unwrap(), generated::CurrentLoop::new());
            for i in 0..2000 {
                let (w, m) = (pick(&mut rng, &odd), pick(&mut rng, &odd));
                let s = pick(&mut rng, &[f32::NAN, -1.0, 0.0, DERATE_SCALE, 1.0, 2.0]);
                let state = [NOMINAL, NOMINAL, NOMINAL, DERATING, FAULT]
                    [(rng.next_unit() * 5.0) as usize % 5];
                let want = hand.update(w, m, s, state);
                let got = generated.step(w, m, s, state);
                assert!(
                    got.0.to_bits() == want.volts.to_bits()
                        && got.1.to_bits() == want.setpoint.to_bits(),
                    "run {run} step {i} ({w}, {m}, {s}, {state}): generated {got:?}, hand-written ({}, {})",
                    want.volts,
                    want.setpoint
                );
            }
        }
    }
}
