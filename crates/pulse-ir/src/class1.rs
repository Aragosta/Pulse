//! Class 1: Temporal Determinism, computed from the IR alone (no execution).

pub use crate::Violation;
use crate::{Hold, Ir, v};
use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct Staleness {
    pub edge: String,
    pub worst_age_ns: u64,
    pub declared_max_ns: Option<u64>,
}

#[derive(Debug, Serialize)]
pub struct Report {
    pub tick_ns: u64,
    pub hyperperiod_ticks: u32,
    /// One per IR edge, in IR order.
    pub staleness: Vec<Staleness>,
    /// Sum of all WCET budgets: what any one tick may cost, since every task runs on every tick.
    pub tick_budget_ns: u64,
}

fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 { a } else { gcd(b, a % b) }
}

pub fn check(ir: &Ir) -> Result<Report, Vec<Violation>> {
    let mut bad = ir.validate();
    if !bad.is_empty() {
        return Err(bad);
    }
    // Integer ns ticks: a rounded-down tick would make every derived age bound optimistic.
    if ir.base_rate_hz == 0 || !1_000_000_000u32.is_multiple_of(ir.base_rate_hz) {
        return Err(vec![v(
            "rate-ratio",
            "C1-RATE",
            format!(
                "base rate {} Hz is not a whole number of ns per tick",
                ir.base_rate_hz
            ),
        )]);
    }
    let tick_ns = 1_000_000_000 / ir.base_rate_hz as u64;

    // 1. Rational rate ratios: every block rate divides the base rate, so a finite hyperperiod exists.
    let mut hyper = 1u32;
    let mut period_ticks = |id: &str, hz: u32| -> Option<u32> {
        if hz == 0 || !ir.base_rate_hz.is_multiple_of(hz) {
            bad.push(v(
                "rate-ratio",
                "C1-RATE",
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
        .map(|b| (b.id.as_str(), period_ticks(&b.id, b.rate_hz)))
        .collect();

    // 2. Every cross-rate edge carries a declared, direction- and factor-correct hold.
    let mut staleness = Vec::new();
    for e in &ir.edges {
        let name = format!("{} -> {}", e.from, e.to);
        let (Some(f), Some(t)) = (ir.block(&e.from), ir.block(&e.to)) else {
            bad.push(v("edge", "C1-EDGE", format!("{name}: unknown block")));
            continue;
        };
        match (f.rate_hz.cmp(&t.rate_hz), e.hold) {
            (std::cmp::Ordering::Equal, None) => {}
            (std::cmp::Ordering::Equal, Some(_)) => bad.push(v(
                "sample-hold",
                "C1-HOLD-SAME",
                format!("{name}: hold declared on a same-rate edge"),
            )),
            (_, None) => bad.push(v(
                "sample-hold",
                "C1-HOLD-MISSING",
                format!(
                    "{name}: {} Hz -> {} Hz read with no declared sample/hold",
                    f.rate_hz, t.rate_hz
                ),
            )),
            (std::cmp::Ordering::Greater, Some(Hold::Decimate(n))) => {
                if f.rate_hz % t.rate_hz != 0 || n != f.rate_hz / t.rate_hz {
                    bad.push(v(
                        "sample-hold",
                        "C1-HOLD-FACTOR",
                        format!(
                            "{name}: Decimate({n}) but rate ratio is {}/{}",
                            f.rate_hz, t.rate_hz
                        ),
                    ));
                }
            }
            (std::cmp::Ordering::Less, Some(Hold::Zoh)) => {}
            (_, Some(h)) => bad.push(v(
                "sample-hold",
                "C1-HOLD-DIR",
                format!(
                    "{name}: {h:?} is the wrong direction for {} Hz -> {} Hz",
                    f.rate_hz, t.rate_hz
                ),
            )),
        }

        // 3. Staleness, in base ticks, on every edge: from the producer computing (a sensor: sampling) a value to the
        //    end of the last tick the consumer still acts on it. Every block fires on multiples of its period and
        //    republishes in between, so it is
        //      sensor lag + delay + wait for the producer's last firing + the consumer's own hold period.
        //    At consumer firings k (multiples of pc) the producer last fired ((k - d) mod pf) ticks earlier, which
        //    peaks at pf - g + ((-d) mod g), g = gcd(pc, pf). All arithmetic saturates: the numbers are untrusted.
        let period = |id: &str| periods.iter().find(|(b, _)| *b == id).and_then(|(_, p)| *p);
        let (Some(pf), Some(pc)) = (period(&e.from), period(&e.to)) else {
            continue;
        };
        let (pf, pc, d, g) = (
            pf as u64,
            pc as u64,
            e.delay_ticks as u64,
            gcd(pc, pf) as u64,
        );
        // A sensor's latency and dropout run count its own samples, pf base ticks each.
        let lag = f.sensor.map_or(0, |s| {
            (s.latency_ticks as u64 + s.max_dropout_run as u64).saturating_mul(pf)
        });
        let wait = pf - g + (g - d % g) % g;
        let worst = lag
            .saturating_add(d)
            .saturating_add(wait + pc)
            .saturating_mul(tick_ns);
        if let Some(m) = e.max_age_ns.filter(|&m| worst > m) {
            bad.push(v(
                "staleness",
                "C1-STALE",
                format!("{name}: worst-case age {worst} ns exceeds declared {m} ns"),
            ));
        }
        staleness.push(Staleness {
            edge: name,
            worst_age_ns: worst,
            declared_max_ns: e.max_age_ns,
        });
    }

    // 4. Causality: every feedback loop crosses at least one delayed edge, so each tick has an evaluation order.
    //    Kahn's algorithm on the zero-delay edges; whatever cannot be ordered sits on a zero-delay cycle.
    let idx = |id: &str| ir.blocks.iter().position(|b| b.id == id);
    let fast: Vec<(usize, usize)> = ir
        .edges
        .iter()
        .filter(|e| e.delay_ticks == 0)
        .filter_map(|e| Some((idx(&e.from)?, idx(&e.to)?)))
        .collect();
    let mut indeg = vec![0; ir.blocks.len()];
    for &(_, t) in &fast {
        indeg[t] += 1;
    }
    let mut ready: Vec<usize> = (0..indeg.len()).filter(|&i| indeg[i] == 0).collect();
    while let Some(n) = ready.pop() {
        for &(f, t) in &fast {
            if f == n {
                indeg[t] -= 1;
                if indeg[t] == 0 {
                    ready.push(t);
                }
            }
        }
    }
    let stuck: Vec<&str> = (0..indeg.len())
        .filter(|&i| indeg[i] > 0)
        .map(|i| ir.blocks[i].id.as_str())
        .collect();
    if !stuck.is_empty() {
        bad.push(v(
            "causality",
            "C1-LOOP",
            format!("zero-delay feedback loop through {}", stuck.join(", ")),
        ));
    }

    // 5. Tick budget: Copper runs every task on every base tick, sequentially in one slot; a slow task fires on its
    //    ticks and republishes its held output on the others. So every tick, not just the one where all rates
    //    align, costs up to the sum of all budgets (each budget bounds every call, firing or not).
    let tick_budget_ns = ir
        .blocks
        .iter()
        .fold(0u64, |s, b| s.saturating_add(b.wcet_budget_ns));
    if tick_budget_ns > tick_ns {
        bad.push(v(
            "wcet-budget",
            "C1-BUDGET",
            format!("budgets sum to {tick_budget_ns} ns > tick period {tick_ns} ns"),
        ));
    }

    if bad.is_empty() {
        Ok(Report {
            tick_ns,
            hyperperiod_ticks: hyper,
            staleness,
            tick_budget_ns,
        })
    } else {
        Err(bad)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::*;

    fn blk(id: &str, rate_hz: u32, wcet: u64) -> Block {
        Block {
            id: id.into(),
            formalism: Formalism::Discrete,
            rate_hz,
            wcet_budget_ns: wcet,
            sensor: None,
            imp: None,
            compute: None,
            span: None,
        }
    }
    fn edge(from: &str, to: &str, hold: Option<Hold>) -> Edge {
        Edge {
            from: from.into(),
            to: to.into(),
            hold,
            max_age_ns: None,
            delay_ticks: 0,
            msg: None,
            span: None,
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
            version: IR_VERSION,
            base_rate_hz: 8000,
            components: vec![],
            blocks: vec![fast, blk("slow", 200, 30_000), blk("sink", 8000, 10_000)],
            edges: vec![
                edge("fast", "slow", Some(Hold::Decimate(40))),
                edge("slow", "sink", Some(Hold::Zoh)),
                edge("fast", "sink", None),
            ],
        }
    }
    /// The one violation `ir` produces (fails if there are zero or several).
    fn only(ir: &Ir) -> &'static str {
        let e = check(ir).unwrap_err();
        assert_eq!(e.len(), 1, "{e:?}");
        e[0].code
    }

    #[test]
    fn valid_ir_passes() {
        let r = check(&good()).unwrap();
        assert_eq!(
            (r.hyperperiod_ticks, r.tick_ns, r.tick_budget_ns),
            (40, 125_000, 60_000)
        );
        // sensor lag 2 + 3, held by slow for 40; slow -> sink waits up to 39 then holds 1; fast -> sink 5 + 1.
        let ages: Vec<u64> = r
            .staleness
            .iter()
            .map(|s| s.worst_age_ns / 125_000)
            .collect();
        assert_eq!(ages, [45, 40, 6]);
    }
    #[test]
    fn unheld_cross_rate_edge_rejected() {
        let mut ir = good();
        ir.edges[0].hold = None;
        assert_eq!(only(&ir), "C1-HOLD-MISSING");
    }
    #[test]
    fn irrational_rate_rejected() {
        let mut ir = good();
        ir.blocks[1].rate_hz = 7000; // also breaks the holds on its edges; the rate error is reported first
        assert_eq!(check(&ir).unwrap_err()[0].code, "C1-RATE");
    }
    #[test]
    fn wrong_decimation_factor_rejected() {
        let mut ir = good();
        ir.edges[0].hold = Some(Hold::Decimate(39));
        assert_eq!(only(&ir), "C1-HOLD-FACTOR");
    }
    #[test]
    fn wrong_hold_direction_rejected() {
        let mut ir = good();
        ir.edges[1].hold = Some(Hold::Decimate(40));
        assert_eq!(only(&ir), "C1-HOLD-DIR");
    }
    #[test]
    fn over_budget_tick_rejected() {
        let mut ir = good();
        ir.blocks[1].wcet_budget_ns = 100_000; // 20 + 100 + 10 > 125, on every tick
        assert_eq!(only(&ir), "C1-BUDGET");
    }
    #[test]
    fn stale_read_rejected() {
        let mut ir = good();
        ir.edges[0].max_age_ns = Some(1_000_000); // 1 ms << derived 5.625 ms
        assert_eq!(only(&ir), "C1-STALE");
    }
    #[test]
    fn staleness_is_in_base_ticks_not_the_rate_ratio() {
        // 1 kHz -> 200 Hz on an 8 kHz base is Decimate(5), but the consumer holds each read for 40 base ticks.
        let mut ir = good();
        ir.blocks.push(blk("mid", 1000, 0));
        ir.edges.push(edge("mid", "slow", Some(Hold::Decimate(5))));
        assert_eq!(check(&ir).unwrap().staleness[3].worst_age_ns, 40 * 125_000);
    }
    #[test]
    fn declared_age_checked_on_every_edge() {
        let mut ir = good();
        ir.edges[1].max_age_ns = Some(1_000_000); // Zoh 200 Hz -> 8 kHz: up to 39 ticks waiting + 1 = 5 ms
        assert_eq!(only(&ir), "C1-STALE");
    }
    #[test]
    fn slow_sensor_lag_counts_its_own_samples() {
        let mut ir = good();
        ir.blocks[0].rate_hz = 1000; // latency 2 + dropout run 3 samples = 40 base ticks, then held for 40
        ir.edges[0].hold = Some(Hold::Decimate(5));
        ir.edges[2].hold = Some(Hold::Zoh); // fast -> sink is now 1 kHz -> 8 kHz
        assert_eq!(check(&ir).unwrap().staleness[0].worst_age_ns, 80 * 125_000);
    }
    #[test]
    fn huge_untrusted_numbers_do_not_overflow() {
        let mut ir = good();
        ir.blocks[0].sensor.as_mut().unwrap().latency_ticks = u32::MAX;
        ir.blocks[1].wcet_budget_ns = u64::MAX;
        ir.edges[0].delay_ticks = u32::MAX;
        assert!(check(&ir).is_err());
    }
    #[test]
    fn unknown_block_rejected() {
        let mut ir = good();
        ir.edges[2].to = "nowhere".into();
        assert_eq!(only(&ir), "C1-EDGE");
    }
    #[test]
    fn hold_on_same_rate_edge_rejected() {
        let mut ir = good();
        ir.edges[2].hold = Some(Hold::Zoh);
        assert_eq!(only(&ir), "C1-HOLD-SAME");
    }
    #[test]
    fn base_rate_must_give_whole_ns_ticks() {
        let mut ir = good();
        ir.base_rate_hz = 0;
        assert_eq!(only(&ir), "C1-RATE");
        ir.base_rate_hz = 3000; // 333_333.3 ns
        assert_eq!(only(&ir), "C1-RATE");
    }
    #[test]
    fn zero_delay_loop_rejected_delayed_loop_accepted() {
        let mut ir = good();
        ir.edges.push(edge("sink", "fast", None));
        assert_eq!(only(&ir), "C1-LOOP");
        ir.edges[3].delay_ticks = 1;
        check(&ir).unwrap();
    }
    #[test]
    fn malformed_ir_rejected_before_any_timing_check() {
        let mut ir = good();
        ir.version = 1;
        assert_eq!(only(&ir), "IR-VERSION");
        let mut ir = good();
        ir.blocks[2].id = "fast".into();
        assert!(
            check(&ir)
                .unwrap_err()
                .iter()
                .any(|v| v.code == "IR-DUP-ID")
        );
        let mut ir = good();
        ir.blocks[0].imp = Some("t::A\"), (id: \"x".into()); // would inject into copperconfig.ron
        assert_eq!(only(&ir), "IR-NAME");
    }
    #[test]
    fn unknown_json_field_rejected() {
        // A typo must not silently drop a safety bound.
        let json = serde_json::to_string(&good())
            .unwrap()
            .replace("max_age_ns", "max_age_n");
        assert!(serde_json::from_str::<Ir>(&json).is_err());
    }
}
