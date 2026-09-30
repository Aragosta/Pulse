//! Differential test of Class 1: random legal IRs, run tick by tick with brute force, must never show an age or a
//! tick cost above what `class1::check` derives. The simulator shares no formula with the checker; it only encodes
//! the execution model: every block runs every base tick in zero-delay order, fires on multiples of its period and
//! republishes its held output otherwise; a sensor returns the sample `latency` samples old, or on dropout repeats
//! its last good one.

use pulse_ir::{Block, Edge, Formalism, Hold, IR_VERSION, Ir, SensorSpec, class1};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        // xorshift64*
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

fn random_ir(r: &mut Rng) -> Ir {
    let base = [1000, 8000, 10_000][r.below(3) as usize];
    let periods = [1, 2, 4, 5, 8, 10, 20, 40];
    let n = 2 + r.below(5) as usize;
    let blocks: Vec<Block> = (0..n)
        .map(|i| Block {
            id: format!("b{i}"),
            formalism: Formalism::Discrete,
            rate_hz: base / periods[r.below(periods.len() as u64) as usize],
            wcet_budget_ns: r.below(1000),
            sensor: (r.below(2) == 0).then(|| SensorSpec {
                latency_ticks: r.below(4) as u32,
                quant_step: 0.0,
                dropout_p: 0.5,
                max_dropout_run: r.below(4) as u32,
            }),
            imp: None,
            compute: None,
            span: None,
        })
        .collect();
    let mut edges = Vec::new();
    for i in 0..n {
        for j in 0..n {
            // Forward edges may have no delay; backward ones close a loop and need one.
            let delay_ticks = match (i < j, r.below(5)) {
                (true, 0..=2) => r.below(3) as u32 * (r.below(3) / 2) as u32,
                (false, 0) if i != j => 1 + r.below(3) as u32,
                _ => continue,
            };
            let (f, t) = (blocks[i].rate_hz, blocks[j].rate_hz);
            let hold = match f.cmp(&t) {
                std::cmp::Ordering::Equal => None,
                std::cmp::Ordering::Less => Some(Hold::Zoh),
                _ if f % t == 0 => Some(Hold::Decimate(f / t)),
                _ => continue,
            };
            edges.push(Edge {
                from: blocks[i].id.clone(),
                to: blocks[j].id.clone(),
                hold,
                max_age_ns: None,
                delay_ticks,
                msg: None,
                span: None,
            });
        }
    }
    Ir {
        version: IR_VERSION,
        base_rate_hz: base,
        blocks,
        edges,
    }
}

/// Worst observed age per edge in base ticks, and worst observed tick cost in ns.
fn simulate(ir: &Ir, r: &mut Rng, ticks: usize) -> (Vec<u64>, u64) {
    let n = ir.blocks.len();
    let idx = |id: &str| ir.blocks.iter().position(|b| b.id == id).unwrap();
    let period: Vec<usize> = ir
        .blocks
        .iter()
        .map(|b| (ir.base_rate_hz / b.rate_hz) as usize)
        .collect();
    let edges: Vec<(usize, usize, usize)> = ir
        .edges
        .iter()
        .map(|e| (idx(&e.from), idx(&e.to), e.delay_ticks as usize))
        .collect();

    // Zero-delay evaluation order, by repeated passes (tiny graphs).
    let mut order = Vec::new();
    while order.len() < n {
        let next = (0..n)
            .find(|&b| {
                !order.contains(&b)
                    && edges
                        .iter()
                        .all(|&(f, t, d)| t != b || d > 0 || order.contains(&f))
            })
            .expect("checker accepted a zero-delay loop");
        order.push(next);
    }

    // out[b][k]: tick the information in b's output at the end of tick k was produced (sampled, for a sensor).
    let mut out: Vec<Vec<Option<i64>>> = vec![vec![None; ticks]; n];
    let mut read: Vec<Option<i64>> = vec![None; edges.len()]; // what each edge's consumer last read
    let mut samples: Vec<Vec<i64>> = vec![Vec::new(); n]; // each sensor's sample times, one per firing
    let mut run = vec![0u32; n];
    let mut last_good: Vec<Option<i64>> = vec![None; n];
    let mut worst_age = vec![0u64; edges.len()];
    let mut worst_cost = 0u64;

    for k in 0..ticks {
        for &b in &order {
            let blk = &ir.blocks[b];
            if k % period[b] != 0 {
                out[b][k] = if k > 0 { out[b][k - 1] } else { None };
                continue;
            }
            for (ei, &(f, _, d)) in edges.iter().enumerate().filter(|(_, e)| e.1 == b) {
                read[ei] = k.checked_sub(d).and_then(|s| out[f][s]);
            }
            out[b][k] = match blk.sensor {
                None => Some(k as i64),
                Some(s) => {
                    samples[b].push(k as i64);
                    let m = samples[b].len() - 1;
                    let fresh = m
                        .checked_sub(s.latency_ticks as usize)
                        .map(|i| samples[b][i]);
                    if run[b] < s.max_dropout_run && r.below(2) == 0 {
                        run[b] += 1; // dropout: repeat the last good sample
                    } else {
                        run[b] = 0;
                        last_good[b] = fresh;
                    }
                    last_good[b]
                }
            };
        }
        // Every block runs every tick, firing or republishing; charge its whole budget either way.
        worst_cost = worst_cost.max(ir.blocks.iter().map(|b| b.wcet_budget_ns).sum());
        // Age at the end of this tick of what each consumer is acting on.
        for (ei, got) in read.iter().enumerate() {
            if let Some(t) = got {
                worst_age[ei] = worst_age[ei].max((k as i64 - t + 1) as u64);
            }
        }
    }
    (worst_age, worst_cost)
}

#[test]
fn simulated_ages_and_tick_costs_never_exceed_what_class1_derives() {
    let mut r = Rng(0x9E37_79B9_7F4A_7C15);
    let (mut checked, mut tight, mut edges) = (0, 0, 0);
    for case in 0..500 {
        let ir = random_ir(&mut r);
        let Ok(rep) = class1::check(&ir) else {
            continue; // over budget: fine, the check refused it
        };
        checked += 1;
        let ticks = 20 * rep.hyperperiod_ticks as usize + 200;
        let (ages, cost) = simulate(&ir, &mut r, ticks);
        assert!(cost <= rep.tick_budget_ns, "case {case}: {ir:?}");
        assert_eq!(rep.staleness.len(), ir.edges.len());
        for (s, &age) in rep.staleness.iter().zip(&ages) {
            let derived = s.worst_age_ns / rep.tick_ns;
            assert!(
                age <= derived,
                "case {case}: {} observed {age} ticks > derived {derived}\n{ir:?}",
                s.edge
            );
            edges += 1;
            tight += (age == derived) as u32;
        }
    }
    // Soundness alone is satisfied by "infinity"; the bound must also be reached, or it is needlessly loose.
    assert!(checked > 300, "only {checked} IRs passed Class 1");
    assert!(
        tight * 10 >= edges * 9,
        "bound reached on only {tight}/{edges} edges"
    );
}
