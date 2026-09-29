pub mod ir;
pub mod params;
pub mod sim;
pub mod tasks;

use core::sync::atomic::Ordering::Relaxed;
use cu29::curuntime::LoopRateLimiter;
use cu29::prelude::*;
use params::*;
use std::path::Path;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 512);

#[copper_runtime(config = "copperconfig.ron")]
struct SingleJointApplication {}

fn us(ns: u64) -> String {
    format!("{:>8.1} us", ns as f64 / 1e3)
}
fn verdict(ok: bool) -> &'static str {
    if ok { "PASS" } else { "FAIL" }
}

fn main() {
    let seconds: u64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(12);
    let ir = ir::single_joint();

    // Class 1, static: refuse to run a graph that cannot be proven temporally deterministic.
    let report = match pulse_ir::class1::check(&ir) {
        Ok(r) => r,
        Err(violations) => {
            eprintln!("Class 1 (Temporal Determinism): REFUSING TO RUN");
            for v in violations {
                eprintln!("  [FAIL] {}: {}", v.check, v.msg);
            }
            std::process::exit(1);
        }
    };

    let logger_path = "logs/single-joint.copper";
    std::fs::create_dir_all(Path::new(logger_path).parent().unwrap()).expect("logs dir");
    let application = SingleJointApplication::builder()
        .with_log_path(logger_path, PREALLOCATED_STORAGE_SIZE)
        .expect("logger")
        .build()
        .expect("application");
    let clock = application.clock();
    let mut app = application
        .start()
        .unwrap_or_else(|e| panic!("start: {}", e.error));
    let mut limiter = LoopRateLimiter::from_rate_target_hz(BASE_HZ as u64, &clock).expect("rate");

    println!(
        "running {seconds} s of sim at {BASE_HZ} Hz ({} ticks)...",
        seconds * BASE_HZ as u64
    );
    for _ in 0..seconds * BASE_HZ as u64 {
        app.run_one_iteration().expect("iteration");
        limiter.limit(&clock);
    }
    let _ = app.stop();

    // ---- report ----
    let mut ok = true;
    let tick_ns = report.tick_ns;
    let stale = &report.staleness[0];
    let age_ns = sim::AGE_MAX_TICKS.load(Relaxed) * tick_ns;

    println!("\nPulse single-joint: Class 1 (Temporal Determinism)");
    println!("\nSTATIC, proved from the IR (holds for every run)");
    println!(
        "  [PASS] rate ratios       base {BASE_HZ} Hz, hyperperiod {} ticks = {:.3} ms",
        report.hyperperiod_ticks,
        report.hyperperiod_ticks as f64 * tick_ns as f64 / 1e6
    );
    println!(
        "  [PASS] cross-rate reads  {} edges, every one with a declared sample/hold",
        ir.edges.iter().filter(|e| e.hold.is_some()).count()
    );
    println!(
        "  [PASS] staleness bound   {}: worst case {:.3} ms <= declared {:.3} ms",
        stale.edge,
        stale.worst_age_ns as f64 / 1e6,
        stale.declared_max_ns.unwrap_or(0) as f64 / 1e6
    );
    println!(
        "  [PASS] tick budget       tick {}: WCET budgets sum to {:.1} us <= {:.1} us",
        report.worst_tick,
        report.worst_tick_budget_ns as f64 / 1e3,
        tick_ns as f64 / 1e3
    );
    println!(
        "  [ -- ] WCET <= budget    NOT PROVEN: no static WCET bound yet (LLVMTA spike), observed only below"
    );

    println!("\nMEASURED on host (observed, not a proof)");
    let staleness_ok = age_ns <= stale.worst_age_ns;
    ok &= staleness_ok;
    println!(
        "  [{}] staleness           observed max age {:.3} ms <= derived bound {:.3} ms",
        verdict(staleness_ok),
        age_ns as f64 / 1e6,
        stale.worst_age_ns as f64 / 1e6
    );
    println!(
        "  task                 observed max     on critical tick    budget   (first 0.5 s excluded; * = over budget)"
    );
    for (i, name) in sim::NAMES.iter().enumerate() {
        let t = &sim::TASK[i];
        let star = |ns: u64| if ns > BUDGET_NS[i] { '*' } else { ' ' };
        let (max, crit) = (t.max_ns.load(Relaxed), t.max_crit_ns.load(Relaxed));
        println!(
            "  {name:<16} {}{}    {}{}    {}",
            us(max),
            star(max),
            us(crit),
            star(crit),
            us(BUDGET_NS[i])
        );
    }
    let span = sim::SPAN_CRIT_MAX_NS.load(Relaxed);
    println!(
        "  graph span, critical tick {}   (tick period {})  [{}]",
        us(span),
        us(tick_ns),
        if span <= tick_ns {
            "fits"
        } else {
            "OVER on host"
        }
    );
    println!(
        "  tick-start jitter max     {}   (indicative: host OS scheduling)",
        us(sim::JITTER_MAX_NS.load(Relaxed))
    );

    println!(
        "\nSCENARIO: stall against a wall, thermal FSM must walk nominal -> derating -> fault"
    );
    let first = |s: usize| sim::STATE_FIRST_TICK[s].load(Relaxed);
    let secs = |t: u64| t as f64 / BASE_HZ as f64;
    let order_ok = first(1) != u64::MAX && first(2) != u64::MAX && first(1) < first(2);
    let clamp_ok =
        sim::DERATE_SETPOINT_MAX_MA.load(Relaxed) as f32 <= DERATE_SCALE * I_MAX * 1000.0 + 1.0;
    ok &= order_ok && clamp_ok;
    let at = |s: usize| {
        if first(s) == u64::MAX {
            "never".to_string()
        } else {
            format!("tick {} ({:.2} s)", first(s), secs(first(s)))
        }
    };
    println!(
        "  [{}] transitions         derating at {}, fault at {}",
        verdict(order_ok),
        at(1),
        at(2)
    );
    println!(
        "  [{}] derating clamp      max setpoint {} mA <= {} mA",
        verdict(clamp_ok),
        sim::DERATE_SETPOINT_MAX_MA.load(Relaxed),
        DERATE_SCALE * I_MAX * 1000.0
    );

    if !ok {
        std::process::exit(1);
    }
}
