//! The single joint: check the IR (Class 1 timing, Class 3 invariants and ranges), then run the generated firmware
//! against a simulated plant and sensor for the stall scenario. The firmware is `pulse_joint::generated::Firmware`,
//! generated from `ir::single_joint()`; nothing in its path is hand-written.

pub mod ir;
pub mod params;
pub mod sim;

use params::*;
use pulse_joint::generated::Firmware;
use pulse_joint::{DERATING, SensorModel};
use std::time::Instant;

fn us(ns: u64) -> String {
    format!("{:>8.1} us", ns as f64 / 1e3)
}
fn verdict(ok: bool) -> &'static str {
    if ok { "PASS" } else { "FAIL" }
}

/// What one run of the stall scenario produced.
pub struct Outcome {
    /// First tick the thermal state machine was in each state (nominal, derating, fault); u64::MAX = never.
    pub first_tick: [u64; 3],
    /// Largest current setpoint (mA) the current loop accepted while derating.
    pub derate_setpoint_max_ma: u64,
    /// Host wall time of one firmware step: max over all ticks, and over ticks where every rate fires.
    pub step_max_ns: u64,
    pub step_max_all_fire_ns: u64,
}

/// Run `ticks` base ticks: plant, then sensor, then one firmware step, whose voltage the plant holds next tick.
pub fn run(ticks: u64) -> Outcome {
    let (mut plant, mut sensor, mut fw) = (
        sim::Plant::default(),
        SensorModel::new(T_AMB as f32),
        Firmware::new(),
    );
    let mut o = Outcome {
        first_tick: [u64::MAX; 3],
        derate_setpoint_max_ma: 0,
        step_max_ns: 0,
        step_max_all_fire_ns: 0,
    };
    for k in 0..ticks {
        plant.step();
        let x = plant.x;
        let r = sensor.sample([x[2] as f32, x[0] as f32, x[3] as f32]);
        let t = Instant::now();
        let (volts, state, setpoint) = fw.step(r.theta, r.current, r.temp, THETA_REF);
        let ns = t.elapsed().as_nanos() as u64;
        plant.v_cmd = volts as f64;
        let state = state as usize;
        o.first_tick[state] = o.first_tick[state].min(k);
        if state == DERATING as usize {
            o.derate_setpoint_max_ma = o
                .derate_setpoint_max_ma
                .max((setpoint.abs() * 1000.0) as u64);
        }
        if k >= WARMUP_TICKS {
            o.step_max_ns = o.step_max_ns.max(ns);
            if k.is_multiple_of(DECIMATION) {
                o.step_max_all_fire_ns = o.step_max_all_fire_ns.max(ns);
            }
        }
    }
    o
}

fn refuse(what: &str, violations: Vec<pulse_ir::Violation>) -> ! {
    eprintln!("{what}: REFUSING TO RUN");
    for v in violations {
        eprintln!("  [FAIL] {} ({}): {}", v.check, v.code, v.msg);
    }
    std::process::exit(1);
}

fn main() {
    let arg = std::env::args().nth(1);
    let ir = ir::single_joint();
    if arg.as_deref() == Some("--ir-json") {
        // The file a frontend (Python, Modelica) will produce.
        println!(
            "{}",
            serde_json::to_string_pretty(&ir).expect("IR serializes")
        );
        return;
    }
    let seconds: u64 = arg.and_then(|s| s.parse().ok()).unwrap_or(12);

    // Static: refuse to run firmware that cannot be proven.
    let report = pulse_ir::class1::check(&ir)
        .unwrap_or_else(|v| refuse("Class 1 (Temporal Determinism)", v));
    let violations = pulse_ir::class3::check(&ir);
    if !violations.is_empty() {
        refuse("Class 3 (state invariants, output ranges)", violations);
    }
    let fw = pulse_ir::graph::firmware(&ir)
        .map_err(|e| e.join("; "))
        .expect("checked above");

    let tick_ns = report.tick_ns;
    println!("Pulse single-joint\n\nSTATIC, proved from the IR (holds for every run)");
    println!(
        "  [PASS] rate ratios       base {BASE_HZ} Hz, hyperperiod {} ticks = {:.3} ms",
        report.hyperperiod_ticks,
        report.hyperperiod_ticks as f64 * tick_ns as f64 / 1e6
    );
    println!(
        "  [PASS] cross-rate reads  {} edges, every one with a declared sample/hold",
        ir.edges.iter().filter(|e| e.hold.is_some()).count()
    );
    for s in report
        .staleness
        .iter()
        .filter(|s| s.declared_max_ns.is_some())
    {
        println!(
            "  [PASS] staleness bound   {}: worst case {:.3} ms <= declared {:.3} ms",
            s.edge,
            s.worst_age_ns as f64 / 1e6,
            s.declared_max_ns.unwrap() as f64 / 1e6
        );
    }
    println!(
        "  [PASS] tick budget       every tick: WCET budgets sum to {:.1} us <= {:.1} us",
        report.tick_budget_ns as f64 / 1e3,
        tick_ns as f64 / 1e3
    );
    let ranged = |ps: &[pulse_ir::expr::Port]| ps.iter().filter(|p| p.range.is_some()).count();
    println!(
        "  [PASS] output ranges     {} firmware outputs and {} block outputs in range, never NaN, for every input",
        ranged(&fw.compute.outputs),
        ranged(&fw.probes)
    );
    let held: Vec<&str> = fw
        .compute
        .state
        .iter()
        .filter(|s| s.range.is_some())
        .map(|s| s.name.as_str())
        .collect();
    println!(
        "  [PASS] state invariants  {} state variables stay in range on every firing, so a bad sample cannot latch",
        held.len()
    );
    println!(
        "  [ -- ] WCET <= budget    NOT PROVEN: no static WCET bound yet (see NOTES.md D-001)"
    );

    println!(
        "\nrunning {seconds} s of sim at {BASE_HZ} Hz ({} ticks)...",
        seconds * BASE_HZ as u64
    );
    let o = run(seconds * BASE_HZ as u64);
    println!("\nMEASURED on host (observed, not a proof; first 0.5 s excluded)");
    println!(
        "  firmware step  max {}   on all-rates tick {}   (tick period {})",
        us(o.step_max_ns),
        us(o.step_max_all_fire_ns),
        us(tick_ns)
    );

    println!(
        "\nSCENARIO: stall against a wall, thermal FSM must walk nominal -> derating -> fault"
    );
    let first = o.first_tick;
    let order_ok = first[1] != u64::MAX && first[2] != u64::MAX && first[1] < first[2];
    let clamp_ok = o.derate_setpoint_max_ma as f32 <= DERATE_SCALE * I_MAX * 1000.0 + 1.0;
    let at = |t: u64| {
        if t == u64::MAX {
            "never".to_string()
        } else {
            format!("tick {t} ({:.2} s)", t as f64 / BASE_HZ as f64)
        }
    };
    println!(
        "  [{}] transitions         derating at {}, fault at {}",
        verdict(order_ok),
        at(first[1]),
        at(first[2])
    );
    println!(
        "  [{}] derating clamp      max setpoint {} mA <= {} mA",
        verdict(clamp_ok),
        o.derate_setpoint_max_ma,
        DERATE_SCALE * I_MAX * 1000.0
    );
    if !(order_ok && clamp_ok) {
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Regression oracle for the stall scenario: the sim is deterministic (seeded sensor, fixed-step plant), so the
    /// thermal transitions land on exact ticks. A change here is a behaviour change: explain it, then update.
    #[test]
    fn stall_scenario_transitions_on_known_ticks() {
        let o = run(38_000);
        assert_eq!((o.first_tick[1], o.first_tick[2]), (16_840, 37_160));
        assert!(o.derate_setpoint_max_ma as f32 <= DERATE_SCALE * I_MAX * 1000.0 + 1.0);
    }
}
