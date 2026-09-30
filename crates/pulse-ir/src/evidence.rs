//! Evidence: what was proved, for which exact IR, and what those proofs rest on (SEMANTICS.md §6-7). A green check
//! means nothing without its assumptions, so both come out of the same call.

use crate::{Ir, Violation, class1, class3, graph, v};
use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct Evidence {
    /// FNV-1a (64-bit) of the IR's JSON: names the exact model these claims hold for. It identifies, it does not
    /// protect against tampering. ponytail: swap for SHA-256 when evidence leaves the building.
    pub ir_hash: String,
    pub ir_version: u32,
    pub proved: Vec<String>,
    /// What the proofs take as given about the world and the toolchain.
    pub assumed: Vec<String>,
    /// Claims the design needs that nothing checks yet.
    pub not_proved: Vec<String>,
}

fn fnv1a(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, b| {
        (h ^ *b as u64).wrapping_mul(0x0100_0000_01b3)
    })
}

fn range(r: [f64; 2]) -> String {
    format!("[{}, {}]", r[0], r[1])
}

pub fn evidence(ir: &Ir) -> Result<Evidence, Vec<Violation>> {
    // Every failure in one round, so a person or agent fixing the model sees all of them. Class 3 needs a well-formed
    // IR, so a malformed one reports only its `IR-*` violations.
    let bad = if ir.validate().is_empty() {
        class3::check(ir)
    } else {
        vec![]
    };
    let timing = match class1::check(ir) {
        Ok(t) if bad.is_empty() => t,
        Ok(_) => return Err(bad),
        Err(mut e) => {
            e.extend(bad);
            return Err(e);
        }
    };
    let fw = graph::firmware(ir).map_err(|es| {
        es.into_iter()
            .map(|e| v("graph", "IR-GRAPH", e))
            .collect::<Vec<_>>()
    })?;
    let json = serde_json::to_string(ir).expect("IR serializes");

    let mut proved = vec![
        format!(
            "rate ratios: hyperperiod {} ticks of {} ns",
            timing.hyperperiod_ticks, timing.tick_ns
        ),
        format!(
            "every cross-rate read declares its sample/hold ({} edges)",
            ir.edges.iter().filter(|e| e.hold.is_some()).count()
        ),
        "no zero-delay feedback loop".into(),
        format!(
            "WCET budgets sum to {} ns <= tick {} ns",
            timing.tick_budget_ns, timing.tick_ns
        ),
        "units consistent in every block, component and firmware edge".into(),
    ];
    for s in &timing.staleness {
        if let Some(m) = s.declared_max_ns {
            proved.push(format!(
                "{}: worst-case age {} ns <= declared {m} ns",
                s.edge, s.worst_age_ns
            ));
        }
    }
    for comp in &ir.components {
        proved.push(format!(
            "component {}: invariants and output ranges hold on its own, for every input its contracts allow",
            comp.name
        ));
    }
    let insts: Vec<&str> = fw.obligations.iter().map(|o| o.at.as_str()).collect();
    if !insts.is_empty() {
        proved.push(format!(
            "every instance is fed what its component assumes: {}",
            insts.join(", ")
        ));
    }
    let computes = ir
        .blocks
        .iter()
        .filter_map(|b| Some((b.id.as_str(), b.compute.as_ref()?)))
        .chain(ir.components.iter().map(|c| (c.name.as_str(), &c.compute)));
    for (at, comp) in computes {
        for s in &comp.defs {
            if let crate::expr::Stmt::Fsm(f) = s {
                let absorbing = f.absorbing();
                proved.push(format!(
                    "{at}.{}: states {} all reachable from {}; deterministic (first enabled transition wins) and \
                     total (none enabled: stay){}",
                    f.state,
                    f.states.join(", "),
                    f.initial,
                    if absorbing.is_empty() {
                        String::new()
                    } else {
                        format!("; absorbing: {}", absorbing.join(", "))
                    }
                ));
            }
        }
    }
    let c = &fw.compute;
    for s in c.state.iter().filter_map(|s| Some((s, s.range?))) {
        proved.push(format!(
            "invariant {} in {} on every tick",
            s.0.name,
            range(s.1)
        ));
    }
    for p in c
        .outputs
        .iter()
        .chain(&fw.probes)
        .filter_map(|p| Some((p, p.range?)))
    {
        proved.push(format!(
            "{} in {}, never NaN, on every tick",
            p.0.name,
            range(p.1)
        ));
    }

    let mut assumed = Vec::new();
    for p in &c.inputs {
        assumed.push(match (p.range, p.glitch) {
            (Some(r), true) => format!(
                "input {}: good samples in {}; bad ones may be NaN or +-inf",
                p.name,
                range(r)
            ),
            (Some(r), false) => format!("input {}: always finite, in {}", p.name, range(r)),
            (None, _) => format!("input {}: nothing (any value, NaN included)", p.name),
        });
    }
    for b in &ir.blocks {
        if let Some(s) = b.sensor {
            assumed.push(format!(
                "{}: latency {} samples, dropout runs at most {} samples",
                b.id, s.latency_ticks, s.max_dropout_run
            ));
        }
    }
    assumed.push("target FPU: IEEE binary32, round-to-nearest, no flush-to-zero".into());
    assumed.push(
        "rustc compiles the generated Rust as written (it is the reference walk printed as Rust)"
            .into(),
    );

    let mut not_proved: Vec<String> = ir
        .blocks
        .iter()
        .filter(|b| b.wcet_budget_ns > 0)
        .map(|b| {
            format!(
                "{}: WCET <= {} ns (no WCET bound yet, NOTES.md D-001)",
                b.id, b.wcet_budget_ns
            )
        })
        .collect();
    let outside: Vec<&str> = ir
        .blocks
        .iter()
        .filter(|b| b.compute.is_none())
        .map(|b| b.id.as_str())
        .collect();
    not_proved.push(format!(
        "outside world ({}): simulated, not verified",
        outside.join(", ")
    ));

    Ok(Evidence {
        ir_hash: format!("fnv1a64:{:016x}", fnv1a(json.as_bytes())),
        ir_version: ir.version,
        proved,
        assumed,
        not_proved,
    })
}
