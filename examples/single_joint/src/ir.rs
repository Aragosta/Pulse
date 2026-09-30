//! Hand-written IR of the single joint (stand-in for a frontend). Source of truth: Class 1 and Class 3 check it and
//! the firmware (`pulse_joint::generated::Firmware`) is generated from it.

use crate::params::*;
use pulse_ir::expr::{
    Component, Compute, Def, Expr, Param, Port, StateVar, Stmt, Ty, Use, num, select, var,
};
use pulse_ir::fsm::{Fsm, Transition};
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

const POSITION_PID: Pid = Pid {
    name: "position_pid",
    kp: POS_KP as f64,
    ki: POS_KI as f64,
    kd: POS_KD as f64,
    dt: OUTER_DT as f64,
    smoothing: 1.0,
    lo: -I_MAX as f64,
    hi: I_MAX as f64,
    meas_max: THETA_SENSE_MAX as f64,
    out: "A",
    meas: "rad",
};

pub fn components() -> Vec<Component> {
    vec![CURRENT_PID.component(), POSITION_PID.component()]
}

fn use_pid(pid: &Pid, setpoint: Expr, measured: Expr, reset: Expr) -> Stmt {
    Stmt::Use(Use {
        name: "pid".into(),
        component: pid.name.into(),
        bind: vec![
            def("setpoint", setpoint),
            def("measured", measured),
            def("reset", reset),
        ],
    })
}

/// The 200 Hz position loop: PID from the commanded angle to a current setpoint. The command comes from outside
/// (an operator, a planner, a learned policy), so it is enforced, not trusted: clamped into the envelope, and a NaN or
/// infinite command holds the last good target.
pub fn position_loop() -> Compute {
    let cmd_max = || var("cmd_max");
    let tgt = select(
        var("target").is_finite(),
        var("target").max(-cmd_max()).min(cmd_max()),
        var("last_target"),
    );
    let lim = THETA_CMD_MAX as f64;
    Compute {
        inputs: vec![
            port("target", Ty::F32, "rad", None),
            Port {
                glitch: true,
                ..port(
                    "measured",
                    Ty::F32,
                    "rad",
                    Some([-THETA_SENSE_MAX as f64, THETA_SENSE_MAX as f64]),
                )
            },
        ],
        params: vec![param("cmd_max", lim, "rad")],
        state: vec![sv("last_target", num(0.0), "rad", Some([-lim, lim]))],
        defs: vec![
            def("tgt", tgt).into(),
            use_pid(
                &POSITION_PID,
                var("tgt"),
                var("measured"),
                Expr::Bool(false),
            ),
            def("amps", var("pid__out")).into(),
        ],
        outputs: vec![port(
            "amps",
            Ty::F32,
            "A",
            Some([-I_MAX as f64, I_MAX as f64]),
        )],
        next: vec![def("last_target", var("tgt"))],
    }
}

/// The 200 Hz thermal state machine: nominal -> derating -> fault (latched), hysteresis on recovery, NaN faults.
/// `scale` limits the current: 1 nominal, `DERATE_SCALE` derating, 0 fault.
pub fn thermal_fsm() -> Compute {
    let temp = || var("temp");
    let to = |from: &[&str], to: &str, guard: Expr| Transition {
        from: from.iter().map(|s| s.to_string()).collect(),
        to: to.into(),
        guard,
    };
    let machine = Fsm {
        state: "mode".into(),
        next: "state".into(),
        // The order fixes the codes the current loop and telemetry see: 0, 1, 2.
        states: ["nominal", "derating", "fault"].map(String::from).to_vec(),
        initial: "nominal".into(),
        transitions: vec![
            // From anywhere: over-temperature, or a broken (NaN) sensor. Fault then has no way out: latched.
            to(
                &[],
                "fault",
                temp().gt(var("t_fault")).or(temp().eq(temp()).not()),
            ),
            to(&["nominal"], "derating", temp().gt(var("t_derate"))),
            to(&["derating"], "nominal", temp().lt(var("t_recover"))),
        ],
    };
    let scale = select(
        var("state").eq(var("nominal")),
        num(1.0),
        select(
            var("state").eq(var("derating")),
            var("derate_scale"),
            num(0.0),
        ),
    );
    Compute {
        inputs: vec![Port {
            glitch: true,
            ..port("temp", Ty::F32, "degC", Some([-40.0, 200.0]))
        }],
        params: vec![
            param("t_fault", T_FAULT as f64, "degC"),
            param("t_derate", T_DERATE as f64, "degC"),
            param("t_recover", T_RECOVER as f64, "degC"),
            param("derate_scale", DERATE_SCALE as f64, "1"),
        ],
        state: vec![],
        defs: vec![Stmt::Fsm(machine), def("scale", scale).into()],
        outputs: vec![
            port("state", Ty::F32, "1", Some([0.0, 2.0])),
            port("scale", Ty::F32, "1", Some([0.0, 1.0])),
        ],
        next: vec![],
    }
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
            use_pid(&CURRENT_PID, var("setpoint"), var("measured"), var("fault")),
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

/// The single joint: firmware blocks (with `compute`) and the outside world they talk to.
pub fn single_joint() -> Ir {
    let outside = |id: &str, formalism, rate_hz, wcet_budget_ns| Block {
        id: id.into(),
        formalism,
        rate_hz,
        wcet_budget_ns,
        sensor: None,
        compute: None,
        span: None,
    };
    let firmware = |id: &str, formalism, rate_hz, wcet_budget_ns, c: Compute| Block {
        compute: Some(c),
        ..outside(id, formalism, rate_hz, wcet_budget_ns)
    };
    let e = |from: &str, fp: Option<&str>, to: &str, tp: Option<&str>, hold| Edge {
        from: from.into(),
        to: to.into(),
        from_port: fp.map(Into::into),
        to_port: tp.map(Into::into),
        hold,
        max_age_ns: None,
        delay_ticks: 0,
        span: None,
    };
    Ir {
        version: IR_VERSION,
        base_rate_hz: BASE_HZ,
        components: components(),
        blocks: vec![
            outside("plant", Continuous, BASE_HZ, 0),
            Block {
                sensor: Some(SensorSpec {
                    latency_ticks: LATENCY_TICKS,
                    quant_step: POS_QUANT,
                    dropout_p: DROPOUT_P,
                    max_dropout_run: MAX_DROPOUT_RUN,
                }),
                ..outside("sensor", Discrete, BASE_HZ, SENSOR_BUDGET_NS)
            },
            outside("command", Discrete, 200, 0),
            firmware(
                "position_loop",
                Discrete,
                200,
                POSITION_BUDGET_NS,
                position_loop(),
            ),
            firmware(
                "thermal_fsm",
                StateMachine,
                200,
                THERMAL_BUDGET_NS,
                thermal_fsm(),
            ),
            firmware(
                "current_loop",
                Discrete,
                BASE_HZ,
                CURRENT_BUDGET_NS,
                current_loop(),
            ),
            outside("actuator", Discrete, BASE_HZ, ACTUATOR_BUDGET_NS),
            outside("telemetry", Discrete, BASE_HZ, 0),
        ],
        // Order matters: it fixes the order of the firmware's inputs and outputs.
        edges: vec![
            e("plant", None, "sensor", None, None),
            // Slow path: a decimated copy of the position with its own declared age bound.
            Edge {
                max_age_ns: Some(6_000_000),
                ..e(
                    "sensor",
                    Some("theta"),
                    "position_loop",
                    Some("measured"),
                    Some(Hold::Decimate(40)),
                )
            },
            // Fast path: full-rate, low-latency current straight into the inner loop.
            e(
                "sensor",
                Some("current"),
                "current_loop",
                Some("measured"),
                None,
            ),
            e(
                "sensor",
                Some("temp"),
                "thermal_fsm",
                Some("temp"),
                Some(Hold::Decimate(40)),
            ),
            e(
                "command",
                Some("theta"),
                "position_loop",
                Some("target"),
                None,
            ),
            e(
                "position_loop",
                Some("amps"),
                "current_loop",
                Some("wanted"),
                Some(Hold::Zoh),
            ),
            e(
                "thermal_fsm",
                Some("scale"),
                "current_loop",
                Some("scale"),
                Some(Hold::Zoh),
            ),
            e(
                "thermal_fsm",
                Some("state"),
                "current_loop",
                Some("state"),
                Some(Hold::Zoh),
            ),
            e(
                "current_loop",
                Some("volts"),
                "actuator",
                Some("volts"),
                None,
            ),
            e(
                "thermal_fsm",
                Some("state"),
                "telemetry",
                Some("thermal_state"),
                Some(Hold::Zoh),
            ),
            e(
                "current_loop",
                Some("setpoint"),
                "telemetry",
                Some("setpoint"),
                None,
            ),
            // The plant integrates the voltage held from the previous tick: the loop's one declared delay.
            Edge {
                delay_ticks: 1,
                ..e("actuator", None, "plant", None, None)
            },
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pulse_ir::expr::{F32, F64, Val};

    #[test]
    fn passes_class1() {
        let r = pulse_ir::class1::check(&single_joint()).unwrap();
        assert_eq!((r.hyperperiod_ticks, r.tick_ns), (40, 125_000));
    }

    #[test]
    fn unheld_cross_rate_read_is_refused() {
        let mut ir = single_joint();
        ir.edges[1].hold = None; // sensor.theta (8 kHz) -> position_loop (200 Hz)
        let errs = pulse_ir::class1::check(&ir).unwrap_err();
        assert!(
            errs.iter().any(|v| v.code == "C1-HOLD-MISSING"
                && v.msg.starts_with("sensor.theta -> position_loop.measured")),
            "{errs:?}"
        );
    }

    #[test]
    fn undeclared_actuation_delay_is_refused() {
        let mut ir = single_joint();
        ir.edges.last_mut().unwrap().delay_ticks = 0;
        let errs = pulse_ir::class1::check(&ir).unwrap_err();
        assert_eq!(errs[0].code, "C1-LOOP", "{errs:?}");
    }

    fn firmware() -> Compute {
        pulse_ir::graph::firmware(&single_joint())
            .map_err(|e| e.join("; "))
            .unwrap()
            .compute
    }

    /// Firmware generated from the IR. Regenerate with `PULSE_BLESS=1 cargo test -p single_joint`.
    pub fn generated_source() -> String {
        format!(
            "//! GENERATED from the Pulse IR (`examples/single_joint/src/ir.rs`). Do not edit; regenerate with\n\
             //! `PULSE_BLESS=1 cargo test -p single_joint`.\n\n{}",
            pulse_ir::rust::emit("Firmware", &firmware())
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

    /// Units are checked inside components, not just in blocks: a PID gain declared upside down is refused.
    #[test]
    fn component_units_are_checked() {
        let mut ir = single_joint();
        let kp = ir.components[0]
            .compute
            .params
            .iter_mut()
            .find(|p| p.name == "kp")
            .unwrap();
        kp.unit = Some("A/V".into());
        let errs = pulse_ir::class1::check(&ir).unwrap_err();
        assert!(
            errs.iter()
                .any(|v| v.code == "IR-UNIT" && v.msg.contains("current_pid")),
            "{errs:?}"
        );
    }

    #[test]
    fn firmware_invariants_and_ranges_are_proved() {
        assert_eq!(pulse_ir::class3::check(&single_joint()), vec![]);
    }

    /// Inputs that exercise every branch: nominal values, the clamp and rail edges, and non-finite values.
    struct Inputs(u64);
    impl Inputs {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        fn pick(&mut self, nominal: (f32, f32), odd: &[f32]) -> f32 {
            let r = self.next();
            if r.is_multiple_of(8) {
                odd[(r >> 8) as usize % odd.len()]
            } else {
                nominal.0 + ((r >> 11) % 1_000_000) as f32 / 1e6 * (nominal.1 - nominal.0)
            }
        }
    }
    const ODD: [f32; 8] = [
        f32::NAN,
        f32::INFINITY,
        f32::NEG_INFINITY,
        -0.0,
        0.0,
        1e30,
        -1e30,
        8.0,
    ];

    /// The generated firmware and the f32 interpreter of the same IR are one computation, bit for bit.
    #[test]
    fn generated_firmware_equals_ir_interpreter() {
        let c = firmware();
        let mut state = c.init(&mut F32);
        let mut fw = pulse_joint::generated::Firmware::new();
        let mut rng = Inputs(0x2545_F491);
        for i in 0..50_000 {
            let theta = rng.pick((-2.0, 2.0), &ODD);
            let current = rng.pick((-10.0, 10.0), &ODD);
            let temp = rng.pick((20.0, 110.0), &ODD);
            let cmd = rng.pick((-12.0, 12.0), &ODD);
            let ins = [theta, current, temp, cmd].map(Val::N).to_vec();
            let (outs, next) = c.step(&mut F32, ins, state);
            state = next;
            let got = fw.step(theta, current, temp, cmd);
            let bits = |v: &Val<f32, bool>| match v {
                Val::N(x) => x.to_bits(),
                Val::B(_) => unreachable!(),
            };
            let want = (bits(&outs[0]), bits(&outs[1]), bits(&outs[2]));
            assert_eq!(
                (got.0.to_bits(), got.1.to_bits(), got.2.to_bits()),
                want,
                "tick {i}"
            );
        }
    }

    /// The f64 reading of the IR is the model; on a nominal run the f32 firmware tracks it closely.
    #[test]
    fn f32_firmware_tracks_f64_model() {
        let c = firmware();
        let (mut s32, mut s64) = (c.init(&mut F32), c.init(&mut F64));
        let mut worst = 0.0f64;
        for k in 0..8000 {
            let theta = (k as f64 * 0.001).sin() * 0.5;
            let current = (k as f64 * 0.013).cos() * 5.0;
            let (temp, cmd) = (40.0, 1.0);
            let (o32, n32) = c.step(
                &mut F32,
                [theta, current, temp, cmd]
                    .map(|x| Val::N(x as f32))
                    .to_vec(),
                s32,
            );
            let (o64, n64) = c.step(
                &mut F64,
                [theta, current, temp, cmd].map(Val::N).to_vec(),
                s64,
            );
            (s32, s64) = (n32, n64);
            if let (Val::N(a), Val::N(b)) = (&o32[0], &o64[0]) {
                worst = worst.max((*a as f64 - b).abs());
            }
        }
        assert!(
            worst < 1e-2,
            "f32 firmware drifts {worst} V from the f64 model"
        );
    }

    /// A state machine with a state no transition can reach is refused, naming the state.
    #[test]
    fn unreachable_fsm_state_is_refused() {
        let mut ir = single_joint();
        let fsm = ir
            .blocks
            .iter_mut()
            .find(|b| b.id == "thermal_fsm")
            .unwrap();
        let Stmt::Fsm(f) = &mut fsm.compute.as_mut().unwrap().defs[0] else {
            unreachable!()
        };
        f.transitions.retain(|t| t.to != "derating");
        let errs = pulse_ir::class1::check(&ir).unwrap_err();
        assert!(
            errs.iter()
                .any(|v| v.msg.contains("derating is unreachable")),
            "{errs:?}"
        );
    }

    /// Evidence names the exact model: same IR, same hash; any change to the model, a new hash.
    #[test]
    fn evidence_hash_follows_the_model() {
        let hash = |ir: &Ir| pulse_ir::evidence::evidence(ir).unwrap().ir_hash;
        let mut ir = single_joint();
        let h = hash(&ir);
        assert_eq!(h, hash(&single_joint()));
        ir.components[0].compute.params[0].value += 1e-3; // a gain, slightly
        assert_ne!(h, hash(&ir));
    }

    #[test]
    fn json_round_trip_still_passes_class1() {
        let ir = single_joint();
        let back: Ir = serde_json::from_str(&serde_json::to_string(&ir).unwrap()).unwrap();
        assert_eq!(back, ir);
        pulse_ir::class1::check(&back).unwrap();
    }
}
