//! The single-joint blocks as Copper tasks. Copper has one loop rate (8 kHz); a 200 Hz block is a task that does
//! real work only on every 40th tick and otherwise republishes its held output. No task allocates.

use crate::params::*;
use crate::sim::{self, timed};
use bincode::{Decode, Encode};
use core::sync::atomic::Ordering::Relaxed;
use cu29::prelude::*;
use pulse_joint::generated;
pub use pulse_joint::{DERATING, FAULT, NOMINAL};
use pulse_joint::{PositionCtl, SensorModel, ThermalFsm};
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

fn critical(tick: u64) -> bool {
    tick.is_multiple_of(DECIMATION) // the tick on which every rate fires
}
fn cfg_err<E>(_: E) -> CuError {
    CuError::from("invalid controller configuration")
}

// ---- sensor (8 kHz source): latency ring, quantization, bounded dropout -------------------------------------------

#[derive(Reflect)]
pub struct Sensor {
    tick: u64,
    #[reflect(ignore)]
    model: SensorModel,
}
impl Freezable for Sensor {}

impl CuSrcTask for Sensor {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(SensorSample);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            tick: 0,
            model: SensorModel::new(T_AMB as f32),
        })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        // Sim only: advance the plant one tick with the voltage the actuator last held, then sample it.
        let x = {
            let mut s = sim::SIM
                .lock()
                .map_err(|_| CuError::from("sim state lock poisoned"))?;
            sim::step(&mut s);
            s.x
        };
        sim::tick_start();
        timed(sim::SENSOR, critical(self.tick), || {
            let r = self.model.sample([x[2] as f32, x[0] as f32, x[3] as f32]);
            output.set_payload(SensorSample {
                theta: r.theta,
                current: r.current,
                temp: r.temp,
                sampled_at: r.sampled_at,
                valid: r.valid,
            });
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
    ctl: PositionCtl,
    amps: f32,
}
impl Freezable for PositionLoop {}

impl CuTask for PositionLoop {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(HeldSample);
    type Output<'m> = output_msg!(CurrentRef);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            tick: 0,
            ctl: PositionCtl::new().map_err(cfg_err)?,
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
                self.amps = self.ctl.update(THETA_REF, h.sample.theta);
            }
            output.set_payload(CurrentRef { amps: self.amps }); // held between activations
        });
        self.tick += 1;
        Ok(())
    }
}

// ---- thermal_fsm (200 Hz): nominal -> derating -> fault (latched) -------------------------------------------------------

#[derive(Reflect)]
pub struct ThermalFsmTask {
    tick: u64,
    #[reflect(ignore)]
    fsm: ThermalFsm,
}
impl Freezable for ThermalFsmTask {}

impl CuTask for ThermalFsmTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(HeldSample);
    type Output<'m> = output_msg!(ThermalLimit);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            tick: 0,
            fsm: ThermalFsm::new(),
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
                let state = self.fsm.update(h.sample.temp);
                sim::STATE_FIRST_TICK[state as usize].fetch_min(h.tick, Relaxed);
            }
            let state = self.fsm.state;
            output.set_payload(ThermalLimit {
                scale: ThermalFsm::scale(state),
                state,
            }); // held between activations
        });
        self.tick += 1;
        Ok(())
    }
}

// ---- current_loop (8 kHz): PI on the full-rate current, setpoint clamped by the thermal limit -------------------------
// The controller is generated from the IR (`ir::current_loop`), not hand-written.

#[derive(Reflect)]
pub struct CurrentLoop {
    tick: u64,
    #[reflect(ignore)]
    ctl: generated::CurrentLoop,
}
impl Freezable for CurrentLoop {}

impl CuTask for CurrentLoop {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, SensorSample, CurrentRef, ThermalLimit);
    type Output<'m> = output_msg!(VoltageCmd);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            tick: 0,
            ctl: generated::CurrentLoop::new(),
        })
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
                let (volts, setpoint) = self.ctl.step(r.amps, s.current, l.scale, l.state);
                if l.state == DERATING {
                    sim::DERATE_SETPOINT_MAX_MA
                        .fetch_max((setpoint.abs() * 1000.0) as u64, Relaxed);
                }
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
