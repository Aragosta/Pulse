//! Codegen: the Copper task graph (`copperconfig.ron`) derived from the IR.

use crate::Ir;
use std::fmt::Write;

pub fn emit(ir: &Ir) -> Result<String, String> {
    if let Some(bad) = ir.validate().first() {
        return Err(bad.msg.clone());
    }
    let is_task = |id: &str| ir.block(id).is_some_and(|b| b.imp.is_some());
    let mut s = String::from("// GENERATED from the Pulse IR. Do not edit.\n(\n    tasks: [\n");
    for b in &ir.blocks {
        if let Some(imp) = &b.imp {
            writeln!(s, "        (id: \"{}\", type: \"{imp}\"),", b.id).unwrap();
        }
    }
    s.push_str("    ],\n    cnx: [\n");
    for e in ir
        .edges
        .iter()
        .filter(|e| is_task(&e.from) && is_task(&e.to))
    {
        let msg = e
            .msg
            .as_ref()
            .ok_or(format!("{} -> {}: no msg type", e.from, e.to))?;
        if let Some(h) = e.hold {
            writeln!(s, "        // hold: {h:?}").unwrap();
        }
        writeln!(
            s,
            "        (src: \"{}\", dst: \"{}\", msg: \"{msg}\"),",
            e.from, e.to
        )
        .unwrap();
    }
    writeln!(
        s,
        "    ],\n    runtime: (rate_target_hz: {}),\n)",
        ir.base_rate_hz
    )
    .unwrap();
    Ok(s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::*;

    fn blk(id: &str, imp: Option<&str>) -> Block {
        Block {
            id: id.into(),
            formalism: Formalism::Discrete,
            rate_hz: 100,
            wcet_budget_ns: 0,
            sensor: None,
            imp: imp.map(Into::into),
            span: None,
        }
    }
    fn edge(from: &str, to: &str, msg: Option<&str>) -> Edge {
        Edge {
            from: from.into(),
            to: to.into(),
            hold: None,
            max_age_ns: None,
            delay_ticks: 0,
            msg: msg.map(Into::into),
            span: None,
        }
    }

    #[test]
    fn emits_tasks_and_task_edges_only() {
        let mut ir = Ir {
            version: IR_VERSION,
            base_rate_hz: 100,
            blocks: vec![
                blk("plant", None),
                blk("a", Some("t::A")),
                blk("b", Some("t::B")),
            ],
            edges: vec![edge("plant", "a", None), edge("a", "b", Some("M"))],
        };
        let out = emit(&ir).unwrap();
        assert!(out.contains("(id: \"a\", type: \"t::A\"),"));
        assert!(out.contains("(src: \"a\", dst: \"b\", msg: \"M\"),"));
        assert!(!out.contains("plant"));
        assert!(out.contains("rate_target_hz: 100"));
        ir.edges[1].msg = None;
        assert!(emit(&ir).is_err());
    }
}
