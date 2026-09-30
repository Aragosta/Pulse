//! The single-joint control logic with no Copper, no std and no allocation, so the exact same code runs in the
//! host simulation and on the microcontroller. Copper tasks are thin wrappers around these types.

#![no_std]

// Generated from the IR, never hand-edited. Parentheses fix evaluation order; `x.max(lo).min(hi)` and `(1 - 1) * x`
// are deliberate (`clamp` panics on NaN bounds; `0 * x` is NaN when `x` is), so lints that would rewrite them are off.
// `a__b` names are flattened component names (`__` keeps them unambiguous).
#[rustfmt::skip]
#[allow(unused_parens, non_snake_case, clippy::double_parens, clippy::manual_clamp, clippy::eq_op)]
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

    /// The firmware generated from the IR is the hand-written controllers wired as the old Copper tasks wired them
    /// (position PID and thermal FSM firing every 40th tick and holding their outputs, the current PI every tick),
    /// plus one fix: a bad current or angle sample is replaced by the last good one. So that chain, fed the
    /// substituted samples, must match it bit for bit, including NaN temperatures, rail saturation and fault resets.
    #[test]
    fn generated_firmware_matches_hand_written_chain() {
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
        ];
        let pick = |rng: &mut Pcg32<f32>, lo: f32, hi: f32| {
            if rng.next_unit() < 0.05 {
                odd[(rng.next_unit() * odd.len() as f32) as usize % odd.len()]
            } else {
                lo + rng.next_unit() * (hi - lo)
            }
        };
        for run in 0..50 {
            let mut fw = generated::Firmware::new();
            let (mut pos, mut fsm, mut cur) = (
                PositionCtl::new().unwrap(),
                ThermalFsm::new(),
                CurrentCtl::new().unwrap(),
            );
            let (mut amps, mut state) = (0.0, NOMINAL);
            // What the generated PIDs substitute for a bad sample: their stored last good measurement.
            let (mut last_theta, mut last_i) = (0.0, 0.0);
            for k in 0..4000u32 {
                let theta = pick(&mut rng, -2.0, 2.0);
                let current = pick(&mut rng, -10.0, 10.0);
                let temp = pick(&mut rng, 20.0, 110.0);
                let cmd = rng.next_unit() * 2.0 - 1.0; // the command contract: finite, within +-10 rad
                if k.is_multiple_of(DECIMATION as u32) {
                    let th = if theta.is_finite() { theta } else { last_theta };
                    last_theta = th;
                    amps = pos.update(cmd, th);
                    state = fsm.update(temp);
                }
                let fed = if current.is_finite() { current } else { last_i };
                last_i = if state == FAULT { 0.0 } else { fed };
                let want = cur.update(amps, fed, ThermalFsm::scale(state), state);
                let got = fw.step(theta, current, temp, cmd);
                assert!(
                    got.0.to_bits() == want.volts.to_bits()
                        && got.1 == state as f32
                        && got.2.to_bits() == want.setpoint.to_bits(),
                    "run {run} tick {k} ({theta}, {current}, {temp}, {cmd}): generated {got:?}, hand-written ({}, {state}, {})",
                    want.volts,
                    want.setpoint
                );
            }
        }
    }

    /// The latch the hand-written current loop had: one NaN current sample drove it to -24 V until a fault.
    #[test]
    fn one_bad_current_sample_does_not_latch() {
        let mut fw = generated::Firmware::new();
        let volts: [f32; 6] =
            [0.0, 0.0, f32::NAN, 0.0, 0.0, 0.0].map(|i| fw.step(0.0, i, 25.0, 1.0).0);
        assert!(volts.iter().all(|v| (0.0..24.0).contains(v)), "{volts:?}");
    }
}
