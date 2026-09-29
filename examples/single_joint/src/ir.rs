//! Hand-written IR of the single-joint graph (stand-in for a frontend). Source of truth: Class 1 checks run on it and
//! `copperconfig.ron` is generated from it.

use crate::params::*;
use pulse_ir::{Block, Edge, Formalism::*, Hold, IR_VERSION, Ir, SensorSpec};

pub const PLANT: &str = "plant"; // simulated physics, not a Copper task

pub fn single_joint() -> Ir {
    let blk = |i: usize, formalism, rate_hz, imp: &str| Block {
        id: crate::sim::NAMES[i].into(),
        formalism,
        rate_hz,
        wcet_budget_ns: BUDGET_NS[i],
        sensor: None,
        imp: Some(imp.into()),
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
        blocks: vec![
            Block {
                id: PLANT.into(),
                formalism: Continuous,
                rate_hz: BASE_HZ,
                wcet_budget_ns: 0,
                sensor: None,
                imp: None,
                span: None,
            },
            sensor,
            blk(1, Discrete, 200, "tasks::Hold200"),
            blk(2, Discrete, 200, "tasks::PositionLoop"),
            blk(3, StateMachine, 200, "tasks::ThermalFsmTask"),
            blk(4, Discrete, BASE_HZ, "tasks::CurrentLoop"),
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

    #[test]
    fn json_round_trip_still_passes_class1() {
        let ir = single_joint();
        let back: Ir = serde_json::from_str(&serde_json::to_string(&ir).unwrap()).unwrap();
        assert_eq!(back, ir);
        pulse_ir::class1::check(&back).unwrap();
    }
}
