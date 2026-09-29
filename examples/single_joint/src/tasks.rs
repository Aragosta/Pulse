//! The single-joint blocks as Copper tasks. Copper has one loop rate (8 kHz); a 200 Hz block is a task that does
//! real work only on every 40th tick and otherwise republishes its held output. No task allocates.

use crate::params::*;
use crate::sim::{self, timed};
use bincode::{Decode, Encode};
use core::sync::atomic::Ordering::Relaxed;
use cu29::prelude::*;
use multicalc::control::Pid;
use multicalc::prelude::*;
use multicalc::random::Pcg32;
use serde::{Deserialize, Serialize};

macro_rules! payload {
    ($($name:ident { $($field:ident: $ty:ty),* })*) => {$(
        #[derive(Default, Debug, Clone, Copy, Encode, Decode, Serialize, Deserialize, Reflect)]
        pub struct $name { $(pub $field: $ty),* }
    )*};
}
payload! {
    // What the sensor block reports: quantized, delayed, possibly a repeat of the last good reading (`valid: false`).
    SensorSample { theta: f32, current: f32, temp: f32, sampled_at: u64, valid: bool }
    // The 200 Hz latch of a sensor sample; `fresh` on the tick it was (re)latched.
    HeldSample { sample: SensorSample, tick: u64, fresh: bool }
    CurrentRef { amps: f32 }
    ThermalLimit { scale: f32, state: u8 }
    VoltageCmd { volts: f32 }
}

pub const NOMINAL: u8 = 0;
pub const DERATING: u8 = 1;
pub const FAULT: u8 = 2;

fn critical(tick: u64) -> bool {
    tick.is_multiple_of(DECIMATION) // the tick on which every rate fires
}
fn quantize(x: f32, step: f32) -> f32 {
    (x / step).round() * step
}
fn cfg_err<E>(_: E) -> CuError {
    CuError::from("invalid controller configuration")
}

// ---- sensor (8 kHz source): latency ring, quantization, bounded dropout -------------------------------------------

const RING: usize = LATENCY_TICKS as usize + 1;

#[derive(Reflect)]
pub struct Sensor {
    tick: u64,
    ring: [[f32; 3]; RING],
    #[reflect(ignore)]
    rng: Pcg32<f32>,
    dropout_run: u32,
    last: SensorSample,
}
impl Freezable for Sensor {}

impl CuSrcTask for Sensor {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(SensorSample);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            tick: 0,
            ring: [[0.0, 0.0, T_AMB as f32]; RING],
            rng: Pcg32::new(0xC0FFEE),
            dropout_run: 0,
            last: SensorSample::default(),
        })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        // Sim only: advance the plant one tick with the voltage the actuator last held, then sample it.
        let x = {
            let mut s = sim::SIM.lock().map_err(cfg_err)?;
            sim::step(&mut s);
            s.x
        };
        sim::tick_start();
        timed(sim::SENSOR, critical(self.tick), || {
            self.ring[self.tick as usize % RING] = [x[2] as f32, x[0] as f32, x[3] as f32];
            let delayed = self.ring[(self.tick as usize + 1) % RING]; // LATENCY_TICKS ticks old
            let drop = self.dropout_run < MAX_DROPOUT_RUN && self.rng.next_unit() < DROPOUT_P;
            if drop {
                self.dropout_run += 1;
                self.last.valid = false;
            } else {
                self.dropout_run = 0;
                self.last = SensorSample {
                    theta: quantize(delayed[0], POS_QUANT),
                    current: quantize(delayed[1], CUR_QUANT),
                    temp: quantize(delayed[2], TEMP_QUANT),
                    sampled_at: self.tick.saturating_sub(LATENCY_TICKS as u64),
                    valid: true,
                };
            }
            output.set_payload(self.last);
        });
        self.tick += 1;
        Ok(())
    }
}

// ---- hold_200: the explicit 8 kHz -> 200 Hz sample-and-hold ---------------------------------------------------------

#[derive(Reflect)]
pub struct Hold200 {
    tick: u64,
    latched: SensorSample,
}
impl Freezable for Hold200 {}

impl CuTask for Hold200 {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(SensorSample);
    type Output<'m> = output_msg!(HeldSample);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            tick: 0,
            latched: SensorSample::default(),
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let fresh = critical(self.tick);
        timed(sim::HOLD, fresh, || {
            if let (true, Some(s)) = (fresh, input.payload()) {
                self.latched = *s;
            }
            output.set_payload(HeldSample {
                sample: self.latched,
                tick: self.tick,
                fresh,
            });
        });
        self.tick += 1;
        Ok(())
    }
}

// ---- position_loop (200 Hz): PID on the held position -> current setpoint ---------------------------------------------

#[derive(Reflect)]
pub struct PositionLoop {
    tick: u64,
    #[reflect(ignore)]
    pid: Pid<f32>,
    amps: f32,
}
impl Freezable for PositionLoop {}

impl CuTask for PositionLoop {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(HeldSample);
    type Output<'m> = output_msg!(CurrentRef);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let pid = Pid::new(POS_KP, POS_KI, POS_KD, OUTER_DT)
            .map_err(cfg_err)?
            .with_output_limits(-I_MAX, I_MAX)
            .map_err(cfg_err)?;
        Ok(Self {
            tick: 0,
            pid,
            amps: 0.0,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        timed(sim::POS, critical(self.tick), || {
            if let Some(h) = input.payload().filter(|h| h.fresh) {
                sim::AGE_MAX_TICKS.fetch_max(h.tick - h.sample.sampled_at, Relaxed);
                self.amps = self.pid.update(THETA_REF, h.sample.theta);
            }
            output.set_payload(CurrentRef { amps: self.amps }); // held between activations
        });
        self.tick += 1;
        Ok(())
    }
}

// ---- thermal_fsm (200 Hz): nominal -> derating -> fault (latched) -------------------------------------------------------

#[derive(Reflect)]
pub struct ThermalFsm {
    tick: u64,
    state: u8,
}
impl Freezable for ThermalFsm {}

impl CuTask for ThermalFsm {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(HeldSample);
    type Output<'m> = output_msg!(ThermalLimit);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            tick: 0,
            state: NOMINAL,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        timed(sim::FSM, critical(self.tick), || {
            if let Some(h) = input.payload().filter(|h| h.fresh) {
                let t = h.sample.temp;
                self.state = match self.state {
                    _ if t > T_FAULT => FAULT,
                    FAULT => FAULT,
                    NOMINAL if t > T_DERATE => DERATING,
                    DERATING if t < T_RECOVER => NOMINAL,
                    s => s,
                };
                sim::STATE_FIRST_TICK[self.state as usize].fetch_min(h.tick, Relaxed);
            }
            let scale = [1.0, DERATE_SCALE, 0.0][self.state as usize];
            output.set_payload(ThermalLimit {
                scale,
                state: self.state,
            }); // held between activations
        });
        self.tick += 1;
        Ok(())
    }
}

// ---- current_loop (8 kHz): PI on the full-rate current, setpoint clamped by the thermal limit -------------------------

#[derive(Reflect)]
pub struct CurrentLoop {
    tick: u64,
    #[reflect(ignore)]
    pi: Pid<f32>,
}
impl Freezable for CurrentLoop {}

impl CuTask for CurrentLoop {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, SensorSample, CurrentRef, ThermalLimit);
    type Output<'m> = output_msg!(VoltageCmd);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let pi = Pid::new(CUR_KP, CUR_KI, 0.0, DT as f32)
            .map_err(cfg_err)?
            .with_output_limits(-V_BUS, V_BUS)
            .map_err(cfg_err)?;
        Ok(Self { tick: 0, pi })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let (sensor, iref, limit) = *input;
        timed(sim::CUR, critical(self.tick), || {
            if let (Some(s), Some(r), Some(l)) = (sensor.payload(), iref.payload(), limit.payload())
            {
                let lim = I_MAX * l.scale;
                let setpoint = r.amps.clamp(-lim, lim);
                if l.state == DERATING {
                    sim::DERATE_SETPOINT_MAX_MA
                        .fetch_max((setpoint.abs() * 1000.0) as u64, Relaxed);
                }
                let volts = if l.state == FAULT {
                    self.pi.reset();
                    0.0
                } else {
                    self.pi.update(setpoint, s.current)
                };
                output.set_payload(VoltageCmd { volts });
            }
        });
        self.tick += 1;
        Ok(())
    }
}

// ---- actuator (8 kHz sink): sim writes the voltage the plant holds for the next tick ---------------------------------

#[derive(Reflect)]
pub struct Actuator {
    tick: u64,
}
impl Freezable for Actuator {}

impl CuSinkTask for Actuator {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(VoltageCmd);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { tick: 0 })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let crit = critical(self.tick);
        timed(sim::ACT, crit, || {
            if let (Some(v), Ok(mut s)) = (input.payload(), sim::SIM.lock()) {
                s.v_cmd = v.volts as f64;
            }
        });
        sim::tick_end(crit);
        self.tick += 1;
        Ok(())
    }
}
