//! Hand-written IR of the single-joint graph (stand-in for a frontend). Source of truth: Class 1 and Class 3 check it,
//! `copperconfig.ron` and `pulse_joint::generated` are generated from it.

use crate::params::*;
use pulse_ir::expr::{
    Component, Compute, Def, Expr, Param, Port, StateVar, Stmt, Ty, Use, num, select, var,
};
use pulse_ir::{Block, Edge, Formalism::*, Hold, IR_VERSION, Ir, SensorSpec};

fn def(name: &str, expr: Expr) -> Def {
    Def {
        name: name.into(),
        expr,
    }
}
fn port(name: &str, ty: Ty, unit: &str, range: Option<[f64; 2]>) -> Port {
    Port {
        name: name.into(),
        ty,
        unit: Some(unit.into()),
        range,
        glitch: false,
    }
}

fn param(name: &str, value: f64, unit: &str) -> Param {
    Param {
        name: name.into(),
        value,
        unit: Some(unit.into()),
    }
}
fn sv(name: &str, init: Expr, unit: &str, range: Option<[f64; 2]>) -> StateVar {
    StateVar {
        name: name.into(),
        init,
        unit: Some(unit.into()),
        range,
    }
}

/// A PID controller as an IR component: multicalc's `Pid::update` term for term (the same f32 operations in the same
/// order), plus one fix: a non-finite measurement is replaced by the last good one before use, so a bad sample cannot
/// reach the state and latch the output (the hand-written loop latched at the rail). Derivative on the measurement
/// through a one-pole filter; conditional integration against the output limits. `reset` returns it to its initial
/// state (its output is then the caller's business).
struct Pid {
    name: &'static str,
    kp: f64,
    ki: f64,
    kd: f64,
    dt: f64,
    smoothing: f64,
    /// Output limits, in `out` units.
    lo: f64,
    hi: f64,
    /// A good measurement is within +-this.
    meas_max: f64,
    out: &'static str,
    meas: &'static str,
}

impl Pid {
    fn component(&self) -> Component {
        let (o, m) = (self.out, self.meas);
        // Invariants proved by `class3`: the integral within the output limits, the stored measurement within
        // `meas_max`, the filtered derivative within what that measurement range allows (2x margin).
        let d_max = 2.0 * (2.0 * self.meas_max / self.dt);
        let or_reset = |e: Expr| select(var("reset"), num(0.0), e);
        let compute = Compute {
            inputs: vec![
                port("setpoint", Ty::F32, m, None),
                Port {
                    glitch: true,
                    ..port(
                        "measured",
                        Ty::F32,
                        m,
                        Some([-self.meas_max, self.meas_max]),
                    )
                },
                port("reset", Ty::Bool, "1", None),
            ],
            params: vec![
                param("kp", self.kp, &format!("{o}/{m}")),
                param("ki", self.ki, &format!("{o}/({m}*s)")),
                param("kd", self.kd, &format!("{o}*s/{m}")),
                param("dt", self.dt, "s"),
                param("smoothing", self.smoothing, "1"),
                param("lo", self.lo, o),
                param("hi", self.hi, o),
            ],
            state: vec![
                sv("integral", num(0.0), o, Some([self.lo, self.hi])),
                sv(
                    "prev_meas",
                    num(0.0),
                    m,
                    Some([-self.meas_max, self.meas_max]),
                ),
                sv("has_prev", Expr::Bool(false), "1", None),
                sv("filt", num(0.0), &format!("{m}/s"), Some([-d_max, d_max])),
                sv("filt_init", Expr::Bool(false), "1", None),
            ],
            defs: vec![
                def(
                    "meas",
                    select(
                        var("measured").is_finite(),
                        var("measured"),
                        var("prev_meas"),
                    ),
                ),
                def("error", var("setpoint") - var("meas")),
                def("p_term", var("kp") * var("error")),
                def(
                    "raw_d",
                    select(
                        var("has_prev"),
                        (var("prev_meas") - var("meas")) / var("dt"),
                        num(0.0),
                    ),
                ),
                def(
                    "filt_d",
                    select(
                        var("filt_init"),
                        var("smoothing") * var("raw_d")
                            + (num(1.0) - var("smoothing")) * var("filt"),
                        var("raw_d"),
                    ),
                ),
                def("d_term", var("kd") * var("filt_d")),
                def(
                    "cand",
                    var("integral") + var("ki") * var("error") * var("dt"),
                ),
                def("unsat", var("p_term") + var("cand") + var("d_term")),
                def("out", var("unsat").max(var("lo")).min(var("hi"))),
                def(
                    "deeper",
                    (var("unsat").gt(var("hi")).and(var("error").gt(num(0.0))))
                        .or(var("unsat").lt(var("lo")).and(var("error").lt(num(0.0)))),
                ),
            ]
            .into_iter()
            .map(Into::into)
            .collect(),
            outputs: vec![port("out", Ty::F32, o, Some([self.lo, self.hi]))],
            next: vec![
                // Conditional integration already keeps the integral inside the output limits (it only grows while
                // the output is unsaturated), so on finite inputs this clamp never acts. Intervals cannot see that
                // relation; the clamp states it, and the invariant becomes provable.
                def(
                    "integral",
                    or_reset(
                        select(var("deeper"), var("integral"), var("cand"))
                            .max(var("lo"))
                            .min(var("hi")),
                    ),
                ),
                def("prev_meas", or_reset(var("meas"))),
                def("has_prev", var("reset").not()),
                def("filt", or_reset(var("filt_d"))),
                def("filt_init", var("reset").not()),
            ],
        };
        Component {
            name: self.name.into(),
            compute,
        }
    }
}

const CURRENT_PID: Pid = Pid {
    name: "current_pid",
    kp: CUR_KP as f64,
    ki: CUR_KI as f64,
    kd: 0.0,
    dt: DT,
    smoothing: 1.0,
    lo: -V_BUS as f64,
    hi: V_BUS as f64,
    meas_max: I_SENSE_MAX as f64,
    out: "V",
    meas: "A",
};

pub fn components() -> Vec<Component> {
    vec![CURRENT_PID.component()]
}

/// The 8 kHz current loop: thermal clamp on the setpoint (NaN-safe), then PI to volts; fault forces 0 V and resets.
pub fn current_loop() -> Compute {
    let lim = select(
        var("scale").ge(num(0.0)),
        var("i_max") * var("scale").min(num(1.0)),
        num(0.0),
    );
    let clamp = var("wanted").max(-var("lim")).min(var("lim"));
    let meas_max = I_SENSE_MAX as f64;
    Compute {
        inputs: vec![
            port("wanted", Ty::F32, "A", None),
            Port {
                glitch: true,
                ..port("measured", Ty::F32, "A", Some([-meas_max, meas_max]))
            },
            port("scale", Ty::F32, "1", None),
            port("state", Ty::U8, "1", None),
        ],
        params: vec![
            param("i_max", I_MAX as f64, "A"),
            param("fault_code", pulse_joint::FAULT as f64, "1"),
        ],
        state: vec![],
        defs: vec![
            def("lim", lim).into(),
            def(
                "setpoint",
                select(var("wanted").is_finite(), clamp, num(0.0)),
            )
            .into(),
            def("fault", var("state").eq(var("fault_code"))).into(),
            Stmt::Use(Use {
                name: "pid".into(),
                component: CURRENT_PID.name.into(),
                bind: vec![
                    def("setpoint", var("setpoint")),
                    def("measured", var("measured")),
                    def("reset", var("fault")),
                ],
            }),
            def("volts", select(var("fault"), num(0.0), var("pid__out"))).into(),
        ],
        outputs: vec![
            port("volts", Ty::F32, "V", Some([-V_BUS as f64, V_BUS as f64])),
            port(
                "setpoint",
                Ty::F32,
                "A",
                Some([-I_MAX as f64, I_MAX as f64]),
            ),
        ],
        next: vec![],
    }
}

pub const PLANT: &str = "plant"; // simulated physics, not a Copper task

pub fn single_joint() -> Ir {
    let blk = |i: usize, formalism, rate_hz, imp: &str| Block {
        id: crate::sim::NAMES[i].into(),
        formalism,
        rate_hz,
        wcet_budget_ns: BUDGET_NS[i],
        sensor: None,
        imp: Some(imp.into()),
        compute: None,
        span: None,
    };
    let sensor = Block {
        sensor: Some(SensorSpec {
            latency_ticks: LATENCY_TICKS,
            quant_step: POS_QUANT,
            dropout_p: DROPOUT_P,
            max_dropout_run: MAX_DROPOUT_RUN,
        }),
        ..blk(0, Discrete, BASE_HZ, "tasks::Sensor")
    };
    let e = |from: &str, to: &str, hold, msg: Option<&str>| Edge {
        from: from.into(),
        to: to.into(),
        hold,
        max_age_ns: None,
        delay_ticks: 0,
        msg: msg.map(|m| format!("crate::tasks::{m}")),
        span: None,
    };
    Ir {
        version: IR_VERSION,
        base_rate_hz: BASE_HZ,
        components: components(),
        blocks: vec![
            Block {
                id: PLANT.into(),
                formalism: Continuous,
                rate_hz: BASE_HZ,
                wcet_budget_ns: 0,
                sensor: None,
                imp: None,
                compute: None,
                span: None,
            },
            sensor,
            blk(1, Discrete, 200, "tasks::Hold200"),
            blk(2, Discrete, 200, "tasks::PositionLoop"),
            blk(3, StateMachine, 200, "tasks::ThermalFsmTask"),
            Block {
                compute: Some(current_loop()),
                ..blk(4, Discrete, BASE_HZ, "tasks::CurrentLoop")
            },
            blk(5, Discrete, BASE_HZ, "tasks::Actuator"),
        ],
        // Order matters: it is Copper's connection order, which fixes current_loop's input tuple order.
        edges: vec![
            e(PLANT, "sensor", None, None),
            // Fast path: full-rate, low-latency, straight into the inner loop.
            e("sensor", "current_loop", None, Some("SensorSample")),
            // Slow path: a decimated copy with its own declared age bound.
            Edge {
                max_age_ns: Some(6_000_000),
                ..e(
                    "sensor",
                    "hold_200",
                    Some(Hold::Decimate(40)),
                    Some("SensorSample"),
                )
            },
            e("hold_200", "position_loop", None, Some("HeldSample")),
            e("hold_200", "thermal_fsm", None, Some("HeldSample")),
            e(
                "position_loop",
                "current_loop",
                Some(Hold::Zoh),
                Some("CurrentRef"),
            ),
            e(
                "thermal_fsm",
                "current_loop",
                Some(Hold::Zoh),
                Some("ThermalLimit"),
            ),
            e("current_loop", "actuator", None, Some("VoltageCmd")),
            // The plant integrates the voltage held from the previous tick: the loop's one declared delay.
            Edge {
                delay_ticks: 1,
                ..e("actuator", PLANT, None, None)
            },
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn passes_class1() {
        let r = pulse_ir::class1::check(&single_joint()).unwrap();
        assert_eq!((r.hyperperiod_ticks, r.tick_ns), (40, 125_000));
    }

    /// `copperconfig.ron` is generated from the IR. Regenerate with `PULSE_BLESS=1 cargo test -p single_joint`.
    #[test]
    fn copper_config_is_generated() {
        let want = pulse_ir::copper::emit(&single_joint()).unwrap();
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/copperconfig.ron");
        if std::env::var_os("PULSE_BLESS").is_some() {
            std::fs::write(path, &want).unwrap();
        }
        assert_eq!(
            std::fs::read_to_string(path).unwrap(),
            want,
            "copperconfig.ron is stale: run PULSE_BLESS=1 cargo test -p single_joint"
        );
    }

    #[test]
    fn unheld_cross_rate_read_is_refused() {
        let mut ir = single_joint();
        ir.edges.push(Edge {
            from: "sensor".into(),
            to: "position_loop".into(),
            hold: None,
            max_age_ns: None,
            delay_ticks: 0,
            msg: None,
            span: None,
        });
        let errs = pulse_ir::class1::check(&ir).unwrap_err();
        assert!(
            errs.iter().any(
                |v| v.code == "C1-HOLD-MISSING" && v.msg.starts_with("sensor -> position_loop")
            )
        );
    }

    #[test]
    fn undeclared_actuation_delay_is_refused() {
        let mut ir = single_joint();
        ir.edges.last_mut().unwrap().delay_ticks = 0;
        let errs = pulse_ir::class1::check(&ir).unwrap_err();
        assert_eq!(errs[0].code, "C1-LOOP", "{errs:?}");
    }

    fn current_loop_flat() -> Compute {
        current_loop().flatten(&components()).unwrap()
    }

    /// Firmware generated from the IR. Regenerate with `PULSE_BLESS=1 cargo test -p single_joint`.
    pub fn generated_source() -> String {
        format!(
            "//! GENERATED from the Pulse IR (`examples/single_joint/src/ir.rs`). Do not edit; regenerate with\n\
             //! `PULSE_BLESS=1 cargo test -p single_joint`.\n\n{}",
            pulse_ir::rust::emit("CurrentLoop", &current_loop_flat())
        )
    }

    #[test]
    fn firmware_is_generated() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../crates/pulse-joint/src/generated.rs"
        );
        let want = generated_source();
        if std::env::var_os("PULSE_BLESS").is_some() {
            std::fs::write(path, &want).unwrap();
        }
        assert_eq!(
            std::fs::read_to_string(path).unwrap(),
            want,
            "pulse-joint/src/generated.rs is stale: run PULSE_BLESS=1 cargo test -p single_joint"
        );
    }

    #[test]
    fn current_loop_ranges_are_proved() {
        assert_eq!(pulse_ir::class3::check(&single_joint()), vec![]);
    }

    /// Inputs that exercise every branch: nominal values, the clamp and rail edges, and non-finite values.
    fn inputs(rng: &mut u64) -> (f32, f32, f32, u8) {
        let mut next = || {
            *rng ^= *rng << 13;
            *rng ^= *rng >> 7;
            *rng ^= *rng << 17;
            *rng
        };
        let mut pick = |odd: &[f32]| {
            let r = next();
            if r.is_multiple_of(4) {
                odd[(r >> 8) as usize % odd.len()]
            } else {
                ((r >> 11) % 20_000) as f32 / 1000.0 - 10.0 // -10 .. 10
            }
        };
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
        let (w, m) = (pick(&odd), pick(&odd));
        let s = pick(&[f32::NAN, -1.0, 0.0, 0.75, 1.0, 2.0]);
        let state = [0, 0, 0, 1, 1, 2, 7][(next() >> 20) as usize % 7]; // 7: an unknown code, not FAULT
        (w, m, s, state)
    }

    /// The generated firmware and the f32 interpreter of the IR are one computation.
    #[test]
    fn generated_code_equals_ir_interpreter() {
        use pulse_ir::expr::{F32, Val};
        let c = current_loop_flat();
        let mut state = c.init(&mut F32);
        let mut fw = pulse_joint::generated::CurrentLoop::new();
        let mut rng = 0x2545_F491u64;
        for i in 0..200_000 {
            let (w, m, s, st) = inputs(&mut rng);
            let ins = vec![Val::N(w), Val::N(m), Val::N(s), Val::N(st as f32)];
            let (outs, next) = c.step(&mut F32, ins, state);
            state = next;
            let got = fw.step(w, m, s, st);
            let want = (&outs[0], &outs[1]);
            let same =
                |a: f32, b: &Val<f32, bool>| matches!(b, Val::N(x) if x.to_bits() == a.to_bits());
            assert!(
                same(got.0, want.0) && same(got.1, want.1),
                "step {i}: {got:?} vs {want:?}"
            );
        }
    }

    /// The f64 reading of the IR is the model; on finite inputs the f32 firmware tracks it closely.
    #[test]
    fn f32_firmware_tracks_f64_model() {
        use pulse_ir::expr::{F32, F64, Val};
        let c = current_loop_flat();
        let (mut s32, mut s64) = (c.init(&mut F32), c.init(&mut F64));
        let mut worst = 0.0f64;
        for k in 0..8000 {
            let (w, m) = (
                (k as f64 * 0.01).sin() * 6.0,
                (k as f64 * 0.013).cos() * 5.0,
            );
            let (o32, n32) = c.step(
                &mut F32,
                vec![Val::N(w as f32), Val::N(m as f32), Val::N(1.0), Val::N(0.0)],
                s32,
            );
            let (o64, n64) = c.step(
                &mut F64,
                vec![Val::N(w), Val::N(m), Val::N(1.0), Val::N(0.0)],
                s64,
            );
            (s32, s64) = (n32, n64);
            if let (Val::N(a), Val::N(b)) = (&o32[0], &o64[0]) {
                worst = worst.max((*a as f64 - b).abs());
            }
        }
        assert!(
            worst < 1e-3,
            "f32 firmware drifts {worst} V from the f64 model"
        );
    }

    #[test]
    fn json_round_trip_still_passes_class1() {
        let ir = single_joint();
        let back: Ir = serde_json::from_str(&serde_json::to_string(&ir).unwrap()).unwrap();
        assert_eq!(back, ir);
        pulse_ir::class1::check(&back).unwrap();
    }
}
