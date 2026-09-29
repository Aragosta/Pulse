//! Class 1: Temporal Determinism, computed from the IR alone (no execution).

use crate::{Hold, Ir};

#[derive(Debug, PartialEq)]
pub struct Violation {
    pub check: &'static str,
    pub msg: String,
}

#[derive(Debug)]
pub struct Staleness {
    pub edge: String,
    pub worst_age_ns: u64,
    pub declared_max_ns: Option<u64>,
}

#[derive(Debug)]
pub struct Report {
    pub tick_ns: u64,
    pub hyperperiod_ticks: u32,
    pub staleness: Vec<Staleness>,
    /// Sum of WCET budgets on the busiest tick of the hyperperiod, and the tick it occurs on.
    pub worst_tick_budget_ns: u64,
    pub worst_tick: u32,
}

fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 { a } else { gcd(b, a % b) }
}

fn v(check: &'static str, msg: String) -> Violation {
    Violation { check, msg }
}

pub fn check(ir: &Ir) -> Result<Report, Vec<Violation>> {
    let mut bad = Vec::new();
    if ir.base_rate_hz == 0 {
        return Err(vec![v("rate-ratio", "base rate is 0".into())]);
    }
    let tick_ns = 1_000_000_000 / ir.base_rate_hz as u64;

    // 1. Rational rate ratios: every block rate divides the base rate, so a finite hyperperiod exists.
    let mut hyper = 1u32;
    let mut period_ticks = |id: &str, hz: u32| -> Option<u32> {
        if hz == 0 || !ir.base_rate_hz.is_multiple_of(hz) {
            bad.push(v(
                "rate-ratio",
                format!(
                    "{id}: {hz} Hz does not divide base rate {} Hz",
                    ir.base_rate_hz
                ),
            ));
            return None;
        }
        let p = ir.base_rate_hz / hz;
        hyper = hyper / gcd(hyper, p) * p;
        Some(p)
    };
    let periods: Vec<(&str, Option<u32>)> = ir
        .blocks
        .iter()
        .map(|b| (b.id, period_ticks(b.id, b.rate_hz)))
        .collect();

    // 2. Every cross-rate edge carries a declared, direction- and factor-correct hold.
    let mut staleness = Vec::new();
    for e in &ir.edges {
        let name = format!("{} -> {}", e.from, e.to);
        let (Some(f), Some(t)) = (ir.block(e.from), ir.block(e.to)) else {
            bad.push(v("edge", format!("{name}: unknown block")));
            continue;
        };
        match (f.rate_hz.cmp(&t.rate_hz), e.hold) {
            (std::cmp::Ordering::Equal, None) => {}
            (std::cmp::Ordering::Equal, Some(_)) => bad.push(v(
                "sample-hold",
                format!("{name}: hold declared on a same-rate edge"),
            )),
            (_, None) => bad.push(v(
                "sample-hold",
                format!(
                    "{name}: {} Hz -> {} Hz read with no declared sample/hold",
                    f.rate_hz, t.rate_hz
                ),
            )),
            (std::cmp::Ordering::Greater, Some(Hold::Decimate(n))) => {
                if f.rate_hz % t.rate_hz != 0 || n != f.rate_hz / t.rate_hz {
                    bad.push(v(
                        "sample-hold",
                        format!(
                            "{name}: Decimate({n}) but rate ratio is {}/{}",
                            f.rate_hz, t.rate_hz
                        ),
                    ));
                }
                // 3. Staleness: sensor latency + worst dropout run + one full hold period (consumer may read at any phase) + one tick to compute.
                let lag = f.sensor.map_or(0, |s| s.latency_ticks + s.max_dropout_run) as u64;
                let worst = (lag + n as u64 + 1) * tick_ns;
                if e.max_age_ns.is_some_and(|m| worst > m) {
                    bad.push(v(
                        "staleness",
                        format!(
                            "{name}: worst-case age {worst} ns exceeds declared {} ns",
                            e.max_age_ns.unwrap()
                        ),
                    ));
                }
                staleness.push(Staleness {
                    edge: name,
                    worst_age_ns: worst,
                    declared_max_ns: e.max_age_ns,
                });
            }
            (std::cmp::Ordering::Less, Some(Hold::Zoh)) => {}
            (_, Some(h)) => bad.push(v(
                "sample-hold",
                format!(
                    "{name}: {h:?} is the wrong direction for {} Hz -> {} Hz",
                    f.rate_hz, t.rate_hz
                ),
            )),
        }
    }

    // 4. Tick budget: Copper runs the whole graph sequentially in one base-rate slot, so on the busiest tick
    //    the sum of WCET budgets of every block firing on it must fit inside one tick.
    let (mut worst_tick, mut worst_sum) = (0, 0);
    if bad.is_empty() {
        for tick in 0..hyper {
            let sum: u64 = ir
                .blocks
                .iter()
                .zip(&periods)
                .filter(|(_, (_, p))| tick % p.unwrap() == 0)
                .map(|(b, _)| b.wcet_budget_ns)
                .sum();
            if sum > worst_sum {
                (worst_tick, worst_sum) = (tick, sum);
            }
        }
        if worst_sum > tick_ns {
            bad.push(v(
                "wcet-budget",
                format!(
                    "tick {worst_tick}: budgets sum to {worst_sum} ns > tick period {tick_ns} ns"
                ),
            ));
        }
    }

    if bad.is_empty() {
        Ok(Report {
            tick_ns,
            hyperperiod_ticks: hyper,
            staleness,
            worst_tick_budget_ns: worst_sum,
            worst_tick,
        })
    } else {
        Err(bad)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::*;

    fn blk(id: &'static str, rate_hz: u32, wcet: u64) -> Block {
        Block {
            id,
            formalism: Formalism::Discrete,
            rate_hz,
            wcet_budget_ns: wcet,
            sensor: None,
        }
    }
    fn edge(from: &'static str, to: &'static str, hold: Option<Hold>) -> Edge {
        Edge {
            from,
            to,
            hold,
            max_age_ns: None,
        }
    }
    fn good() -> Ir {
        let mut fast = blk("fast", 8000, 20_000);
        fast.sensor = Some(SensorSpec {
            latency_ticks: 2,
            quant_step: 0.0,
            dropout_p: 0.0,
            max_dropout_run: 3,
        });
        Ir {
            base_rate_hz: 8000,
            blocks: vec![fast, blk("slow", 200, 30_000), blk("sink", 8000, 10_000)],
            edges: vec![
                edge("fast", "slow", Some(Hold::Decimate(40))),
                edge("slow", "sink", Some(Hold::Zoh)),
                edge("fast", "sink", None),
            ],
        }
    }
    fn only(ir: &Ir) -> &'static str {
        check(ir).unwrap_err()[0].check
    }

    #[test]
    fn valid_ir_passes() {
        let r = check(&good()).unwrap();
        assert_eq!(
            (
                r.hyperperiod_ticks,
                r.tick_ns,
                r.worst_tick,
                r.worst_tick_budget_ns
            ),
            (40, 125_000, 0, 60_000)
        );
        assert_eq!(r.staleness[0].worst_age_ns, (2 + 3 + 40 + 1) * 125_000);
    }
    #[test]
    fn unheld_cross_rate_edge_rejected() {
        let mut ir = good();
        ir.edges[0].hold = None;
        assert_eq!(only(&ir), "sample-hold");
    }
    #[test]
    fn irrational_rate_rejected() {
        let mut ir = good();
        ir.blocks[1].rate_hz = 7000;
        assert_eq!(only(&ir), "rate-ratio");
    }
    #[test]
    fn wrong_decimation_factor_rejected() {
        let mut ir = good();
        ir.edges[0].hold = Some(Hold::Decimate(39));
        assert_eq!(only(&ir), "sample-hold");
    }
    #[test]
    fn wrong_hold_direction_rejected() {
        let mut ir = good();
        ir.edges[1].hold = Some(Hold::Decimate(40));
        assert_eq!(only(&ir), "sample-hold");
    }
    #[test]
    fn over_budget_tick_rejected() {
        let mut ir = good();
        ir.blocks[1].wcet_budget_ns = 100_000; // only the tick both fast+slow fire on is over: 20+100+10 > 125
        let e = check(&ir).unwrap_err();
        assert_eq!(e[0].check, "wcet-budget");
        assert!(e[0].msg.starts_with("tick 0:"));
    }
    #[test]
    fn stale_read_rejected() {
        let mut ir = good();
        ir.edges[0].max_age_ns = Some(1_000_000); // 1 ms << derived 5.75 ms
        assert_eq!(only(&ir), "staleness");
    }
}
