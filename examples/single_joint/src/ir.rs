//! Hand-written IR of the single-joint graph. Source of truth for Class 1 checks; the tests below keep it
//! in lockstep with `copperconfig.ron` so the two descriptions cannot drift.

use crate::params::*;
use pulse_ir::{Block, Edge, Formalism::*, Hold, Ir, SensorSpec};

pub const PLANT: &str = "plant"; // simulated physics, not a Copper task

pub fn single_joint() -> Ir {
    let blk = |i: usize, formalism, rate_hz| Block {
        id: crate::sim::NAMES[i],
        formalism,
        rate_hz,
        wcet_budget_ns: BUDGET_NS[i],
        sensor: None,
    };
    let sensor = Block {
        sensor: Some(SensorSpec {
            latency_ticks: LATENCY_TICKS,
            quant_step: POS_QUANT,
            dropout_p: DROPOUT_P,
            max_dropout_run: MAX_DROPOUT_RUN,
        }),
        ..blk(0, Discrete, BASE_HZ)
    };
    let e = |from, to, hold| Edge {
        from,
        to,
        hold,
        max_age_ns: None,
    };
    Ir {
        base_rate_hz: BASE_HZ,
        blocks: vec![
            Block {
                id: PLANT,
                formalism: Continuous,
                rate_hz: BASE_HZ,
                wcet_budget_ns: 0,
                sensor: None,
            },
            sensor,
            blk(1, Discrete, 200),
            blk(2, Discrete, 200),
            blk(3, StateMachine, 200),
            blk(4, Discrete, BASE_HZ),
            blk(5, Discrete, BASE_HZ),
        ],
        edges: vec![
            e(PLANT, "sensor", None),
            // Fast path: full-rate, low-latency, straight into the inner loop.
            e("sensor", "current_loop", None),
            // Slow path: a decimated copy with its own declared age bound.
            Edge {
                max_age_ns: Some(6_000_000),
                ..e("sensor", "hold_200", Some(Hold::Decimate(40)))
            },
            e("hold_200", "position_loop", None),
            e("hold_200", "thermal_fsm", None),
            e("position_loop", "current_loop", Some(Hold::Zoh)),
            e("thermal_fsm", "current_loop", Some(Hold::Zoh)),
            e("current_loop", "actuator", None),
            e("actuator", PLANT, None),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ron::Value;

    fn cfg() -> Value {
        ron::from_str(include_str!("../copperconfig.ron")).unwrap()
    }
    fn get<'a>(v: &'a Value, k: &str) -> &'a Value {
        let Value::Map(m) = v else {
            panic!("not a map")
        };
        m.iter()
            .find(|(key, _)| matches!(key, Value::String(s) if s == k))
            .unwrap_or_else(|| panic!("no {k}"))
            .1
    }
    fn seq(v: &Value) -> &Vec<Value> {
        let Value::Seq(s) = v else {
            panic!("not a seq")
        };
        s
    }
    fn string(v: &Value) -> &str {
        let Value::String(s) = v else {
            panic!("not a string")
        };
        s
    }

    #[test]
    fn passes_class1() {
        let r = pulse_ir::class1::check(&single_joint()).unwrap();
        assert_eq!((r.hyperperiod_ticks, r.tick_ns), (40, 125_000));
    }

    #[test]
    fn copper_config_matches_ir() {
        let ir = single_joint();
        let c = cfg();
        let Value::Number(hz) = get(get(&c, "runtime"), "rate_target_hz") else {
            panic!()
        };
        assert_eq!(hz.into_f64() as u32, ir.base_rate_hz);

        let mut ids: Vec<&str> = seq(get(&c, "tasks"))
            .iter()
            .map(|t| string(get(t, "id")))
            .collect();
        let mut blocks: Vec<&str> = ir
            .blocks
            .iter()
            .map(|b| b.id)
            .filter(|&id| id != PLANT)
            .collect();
        ids.sort();
        blocks.sort();
        assert_eq!(ids, blocks);

        let mut cnx: Vec<(&str, &str)> = seq(get(&c, "cnx"))
            .iter()
            .map(|e| (string(get(e, "src")), string(get(e, "dst"))))
            .collect();
        let mut edges: Vec<(&str, &str)> = ir
            .edges
            .iter()
            .filter(|e| e.from != PLANT && e.to != PLANT)
            .map(|e| (e.from, e.to))
            .collect();
        cnx.sort();
        edges.sort();
        assert_eq!(cnx, edges);
    }

    #[test]
    fn unheld_cross_rate_read_is_refused() {
        let mut ir = single_joint();
        ir.edges.push(Edge {
            from: "sensor",
            to: "position_loop",
            hold: None,
            max_age_ns: None,
        });
        let errs = pulse_ir::class1::check(&ir).unwrap_err();
        assert!(
            errs.iter()
                .any(|v| v.check == "sample-hold" && v.msg.starts_with("sensor -> position_loop"))
        );
    }
}
