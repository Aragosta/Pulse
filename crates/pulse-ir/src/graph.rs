//! The firmware: every block with `compute`, composed into one flat `Compute` that runs once per base tick. This is
//! the meaning of an IR graph (NOTES.md D-008); the interpreter, the proofs and codegen all read this one object.
//!
//! - Blocks with `compute` are firmware; blocks without are the outside world (plant, sensors, commands,
//!   actuators). Edges from outside become firmware inputs `{block}__{port}` carrying the consumer's contract;
//!   edges to outside become outputs `{block}__{port}`.
//! - Every base tick the firmware blocks run in zero-delay dependency order. A block of period `p` base ticks
//!   fires when its counter is 0 (ticks 0, p, 2p, ...); between firings its state and outputs are held.
//! - A reader sees its producer's current output. `Decimate`/`Zoh` holds declare that crossing; they need no code.
//! - Block names appear as `{block}__{name}`; each block output as `{block}__{port}__now`.

use crate::Ir;
use crate::expr::{Compute, Def, Expr, Port, StateVar, Stmt, Ty, num, select, var};

pub struct Firmware {
    pub compute: Compute,
    /// Every block output, exposed as `{block}__{port}__now` with that port's declared range: proof obligations
    /// that are not firmware outputs.
    pub probes: Vec<Port>,
    /// Every component instance's bound inputs against the component's contracts, in this namespace.
    pub obligations: Vec<crate::expr::Obligation>,
}

fn port_name(block: &str, port: &Option<String>) -> String {
    match port {
        Some(p) => format!("{block}__{p}"),
        None => block.to_string(),
    }
}

pub fn firmware(ir: &Ir) -> Result<Firmware, Vec<String>> {
    let mut bad = Vec::new();
    let is_fw = |id: &str| ir.block(id).is_some_and(|b| b.compute.is_some());
    let blocks: Vec<_> = ir.blocks.iter().filter(|b| b.compute.is_some()).collect();
    let mut flat = Vec::new();
    let mut block_obligations = Vec::new();
    for b in &blocks {
        match b
            .compute
            .as_ref()
            .unwrap()
            .flatten_obligations(&ir.components)
        {
            Ok((c, o)) => {
                flat.push(c);
                block_obligations.push(o);
            }
            Err(e) => bad.push(format!("{}: {e}", b.id)),
        }
    }
    let period =
        |hz: u32| (hz != 0 && ir.base_rate_hz.is_multiple_of(hz)).then(|| ir.base_rate_hz / hz);
    for b in &blocks {
        if period(b.rate_hz).is_none() {
            bad.push(format!(
                "{}: {} Hz does not divide the base rate",
                b.id, b.rate_hz
            ));
        }
    }
    if !bad.is_empty() {
        return Err(bad);
    }
    let comp = |id: &str| &flat[blocks.iter().position(|b| b.id == id).unwrap()];

    // Edges must name existing ports on firmware ends, feed each firmware input exactly once, and agree on units.
    for e in &ir.edges {
        let (f, t) = (is_fw(&e.from), is_fw(&e.to));
        // The generated step has no delay buffers yet: a delay into firmware would be silently dropped.
        if t && e.delay_ticks > 0 {
            bad.push(format!(
                "{} -> {}: delays on edges into the firmware are not supported yet",
                e.from, e.to
            ));
        }
        let out = f.then(|| {
            comp(&e.from)
                .outputs
                .iter()
                .find(|p| Some(&p.name) == e.from_port.as_ref())
        });
        let inp = t.then(|| {
            comp(&e.to)
                .inputs
                .iter()
                .find(|p| Some(&p.name) == e.to_port.as_ref())
        });
        if out == Some(None) {
            bad.push(format!(
                "{} -> {}: {} has no output {:?}",
                e.from, e.to, e.from, e.from_port
            ));
        }
        if inp == Some(None) {
            bad.push(format!(
                "{} -> {}: {} has no input {:?}",
                e.from, e.to, e.to, e.to_port
            ));
        }
        for (end, fw, p) in [(&e.from, f, &e.from_port), (&e.to, t, &e.to_port)] {
            if !fw
                && p.as_ref()
                    .is_some_and(|p| !crate::is_ident(p) || p.contains("__"))
            {
                bad.push(format!(
                    "{end}: port {p:?} is not an identifier without `__`"
                ));
            }
        }
        if let (Some(Some(o)), Some(Some(i))) = (out, inp)
            && ((o.ty == Ty::Bool) != (i.ty == Ty::Bool)
                || (o.unit.is_some() && i.unit.is_some() && o.unit != i.unit))
        {
            bad.push(format!(
                "{}.{} ({:?} {:?}) -> {}.{} ({:?} {:?}): type or unit mismatch",
                e.from, o.name, o.ty, o.unit, e.to, i.name, i.ty, i.unit
            ));
        }
    }
    for (b, c) in blocks.iter().zip(&flat) {
        for p in &c.inputs {
            let n = ir
                .edges
                .iter()
                .filter(|e| e.to == b.id && e.to_port.as_ref() == Some(&p.name))
                .count();
            if n != 1 {
                bad.push(format!(
                    "{}.{}: {n} edges drive it, need exactly 1",
                    b.id, p.name
                ));
            }
        }
    }

    // Zero-delay dependency order among firmware blocks.
    let mut order: Vec<usize> = Vec::new();
    while order.len() < blocks.len() {
        let next = (0..blocks.len()).find(|&i| {
            !order.contains(&i)
                && ir.edges.iter().all(|e| {
                    e.to != blocks[i].id
                        || !is_fw(&e.from)
                        || e.delay_ticks > 0
                        || order.iter().any(|&j| blocks[j].id == e.from)
                })
        });
        match next {
            Some(i) => order.push(i),
            None => {
                bad.push("zero-delay loop among firmware blocks".into());
                break;
            }
        }
    }
    if !bad.is_empty() {
        return Err(bad);
    }

    let mut fw = Compute {
        inputs: Vec::new(),
        params: Vec::new(),
        state: Vec::new(),
        defs: Vec::new(),
        outputs: Vec::new(),
        next: Vec::new(),
    };
    // Boundary inputs, in edge order, with the consumer's contract. One physical signal read twice is one input.
    for e in ir.edges.iter().filter(|e| !is_fw(&e.from) && is_fw(&e.to)) {
        let name = port_name(&e.from, &e.from_port);
        let contract = comp(&e.to)
            .inputs
            .iter()
            .find(|p| Some(&p.name) == e.to_port.as_ref())
            .unwrap();
        let port = Port {
            name: name.clone(),
            ..contract.clone()
        };
        match fw.inputs.iter().find(|p| p.name == name) {
            None => fw.inputs.push(port),
            Some(p) if *p != port => {
                bad.push(format!("{name}: read under two different contracts"))
            }
            Some(_) => {}
        }
    }

    // One counter per slow rate: `__tick_p` counts 0..p-1, the block fires when it is 0.
    let mut rates: Vec<u32> = blocks
        .iter()
        .map(|b| period(b.rate_hz).unwrap())
        .filter(|&p| p > 1)
        .collect();
    rates.sort();
    rates.dedup();
    for p in &rates {
        let (tick, bump, fire) = (
            format!("__tick_{p}"),
            format!("__bump_{p}"),
            format!("__fire_{p}"),
        );
        fw.state.push(StateVar {
            name: tick.clone(),
            init: num(0.0),
            unit: Some("1".into()),
            range: Some([0.0, *p as f64]),
        });
        fw.defs.push(
            Def {
                name: fire,
                expr: var(&tick).eq(num(0.0)),
            }
            .into(),
        );
        fw.defs.push(
            Def {
                name: bump.clone(),
                expr: var(&tick) + num(1.0),
            }
            .into(),
        );
        fw.next.push(Def {
            name: tick,
            expr: select(var(&bump).ge(num(*p as f64)), num(0.0), var(&bump)),
        });
    }

    let mut probes = Vec::new();
    let mut obligations = Vec::new();
    for &i in &order {
        let (b, c) = (blocks[i], &flat[i]);
        let p = period(b.rate_hz).unwrap();
        let fire = (p > 1).then(|| var(&format!("__fire_{p}")));
        let pre = |n: &str| format!("{}__{n}", b.id);
        // An input is replaced by its source: another block's current output or a boundary input.
        let source = |n: &str| -> Expr {
            let e = ir
                .edges
                .iter()
                .find(|e| e.to == b.id && e.to_port.as_deref() == Some(n))
                .unwrap();
            if is_fw(&e.from) {
                var(&format!(
                    "{}__{}__now",
                    e.from,
                    e.from_port.as_deref().unwrap()
                ))
            } else {
                var(&port_name(&e.from, &e.from_port))
            }
        };
        let rename = |e: &Expr| {
            e.map_vars(&|n| {
                if c.inputs.iter().any(|p| p.name == n) {
                    source(n)
                } else {
                    var(&pre(n))
                }
            })
        };
        obligations.extend(
            block_obligations[i]
                .iter()
                .map(|o| crate::expr::Obligation {
                    at: format!("{}.{}", b.id, o.at),
                    value: rename(&o.value),
                    ..o.clone()
                }),
        );
        fw.params
            .extend(c.params.iter().map(|x| crate::expr::Param {
                name: pre(&x.name),
                ..x.clone()
            }));
        fw.state.extend(c.state.iter().map(|s| StateVar {
            name: pre(&s.name),
            ..s.clone()
        }));
        for s in &c.defs {
            let Stmt::Let(d) = s else {
                unreachable!("flattened")
            };
            fw.defs.push(
                Def {
                    name: pre(&d.name),
                    expr: rename(&d.expr),
                }
                .into(),
            );
        }
        let gate = |e: Expr, held: Expr| match &fire {
            Some(f) => select(f.clone(), e, held),
            None => e,
        };
        for s in &c.state {
            let new = c
                .next
                .iter()
                .find(|n| n.name == s.name)
                .map_or(var(&pre(&s.name)), |n| rename(&n.expr));
            fw.next.push(Def {
                name: pre(&s.name),
                expr: gate(new, var(&pre(&s.name))),
            });
        }
        // Outputs: current value `__now`; a slow block holds its last one in `__held` between firings.
        for o in &c.outputs {
            let now = pre(&format!("{}__now", o.name));
            let held = pre(&format!("{}__held", o.name));
            let value = rename(&var(&o.name));
            if fire.is_some() {
                fw.state.push(StateVar {
                    name: held.clone(),
                    // Never observed (every block fires on tick 0); inside the range so the invariant holds.
                    init: match (o.ty, o.range) {
                        (Ty::Bool, _) => Expr::Bool(false),
                        (_, Some([lo, hi])) => num(0.0f64.max(lo).min(hi)),
                        _ => num(0.0),
                    },
                    unit: o.unit.clone(),
                    range: o.range,
                });
                fw.next.push(Def {
                    name: held.clone(),
                    expr: var(&now),
                });
            }
            fw.defs.push(
                Def {
                    name: now.clone(),
                    expr: gate(value, var(&held)),
                }
                .into(),
            );
            probes.push(Port {
                name: now,
                ..o.clone()
            });
        }
    }

    // Boundary outputs, in edge order, with the producer's declared range.
    for e in ir.edges.iter().filter(|e| is_fw(&e.from) && !is_fw(&e.to)) {
        let name = port_name(&e.to, &e.to_port);
        let src = e.from_port.as_deref().unwrap();
        let o = comp(&e.from)
            .outputs
            .iter()
            .find(|p| p.name == src)
            .unwrap();
        fw.defs.push(
            Def {
                name: name.clone(),
                expr: var(&format!("{}__{src}__now", e.from)),
            }
            .into(),
        );
        fw.outputs.push(Port { name, ..o.clone() });
    }
    if !bad.is_empty() {
        return Err(bad);
    }
    bad.extend(fw.validate());
    if bad.is_empty() {
        Ok(Firmware {
            compute: fw,
            probes,
            obligations,
        })
    } else {
        Err(bad)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::expr::{F64, Val};
    use crate::{Block, Edge, Formalism, Hold, IR_VERSION};

    fn port(name: &str) -> Port {
        Port {
            name: name.into(),
            ty: Ty::F32,
            unit: Some("1".into()),
            range: None,
            glitch: false,
        }
    }
    fn block(id: &str, rate_hz: u32, compute: Option<Compute>) -> Block {
        Block {
            id: id.into(),
            formalism: Formalism::Discrete,
            rate_hz,
            wcet_budget_ns: 0,
            sensor: None,
            span: None,
            compute,
        }
    }
    fn edge(from: &str, fp: &str, to: &str, tp: &str, hold: Option<Hold>) -> Edge {
        Edge {
            from: from.into(),
            to: to.into(),
            from_port: Some(fp.into()),
            to_port: Some(tp.into()),
            hold,
            max_age_ns: None,
            delay_ticks: 0,
            span: None,
        }
    }
    /// `slow` (every 4th tick) counts its firings and outputs the count before incrementing; `fast` (every tick)
    /// adds it to an outside input and sends the sum out.
    fn ir() -> Ir {
        let slow = Compute {
            inputs: vec![],
            params: vec![],
            state: vec![StateVar {
                name: "n".into(),
                init: num(0.0),
                unit: Some("1".into()),
                range: None,
            }],
            defs: vec![
                Def {
                    name: "count".into(),
                    expr: var("n"),
                }
                .into(),
            ],
            outputs: vec![port("count")],
            next: vec![Def {
                name: "n".into(),
                expr: var("n") + num(1.0),
            }],
        };
        let fast = Compute {
            inputs: vec![port("x"), port("c")],
            params: vec![],
            state: vec![],
            defs: vec![
                Def {
                    name: "y".into(),
                    expr: var("x") + var("c"),
                }
                .into(),
            ],
            outputs: vec![port("y")],
            next: vec![],
        };
        Ir {
            version: IR_VERSION,
            base_rate_hz: 8,
            blocks: vec![
                block("src", 8, None),
                block("slow", 2, Some(slow)),
                block("fast", 8, Some(fast)),
                block("out", 8, None),
            ],
            edges: vec![
                edge("src", "x", "fast", "x", None),
                edge("slow", "count", "fast", "c", Some(Hold::Zoh)),
                edge("fast", "y", "out", "y", None),
            ],
            components: vec![],
        }
    }

    #[test]
    fn slow_block_fires_every_period_and_holds_between() {
        let c = firmware(&ir()).unwrap().compute;
        assert_eq!(
            c.inputs.iter().map(|p| p.name.as_str()).collect::<Vec<_>>(),
            ["src__x"]
        );
        assert_eq!(c.outputs[0].name, "out__y");
        let mut s = c.init(&mut F64);
        let mut ys = Vec::new();
        for k in 0..10 {
            let (o, n) = c.step(&mut F64, vec![Val::N(k as f64 * 100.0)], s);
            s = n;
            ys.push(o[0].clone());
        }
        let want = [
            0.0, 100.0, 200.0, 300.0, 401.0, 501.0, 601.0, 701.0, 802.0, 902.0,
        ];
        assert_eq!(ys, want.map(Val::N));
    }

    /// A slow block's held output starts inside the output's range, so a range excluding 0 is still provable.
    #[test]
    fn held_output_range_need_not_contain_zero() {
        let c = Compute {
            inputs: vec![],
            params: vec![],
            state: vec![],
            defs: vec![
                Def {
                    name: "k".into(),
                    expr: num(5.0),
                }
                .into(),
            ],
            outputs: vec![Port {
                range: Some([1.0, 10.0]),
                ..port("k")
            }],
            next: vec![],
        };
        let ir = Ir {
            version: IR_VERSION,
            base_rate_hz: 8,
            blocks: vec![block("slow", 2, Some(c)), block("out", 8, None)],
            edges: vec![edge("slow", "k", "out", "k", Some(Hold::Zoh))],
            components: vec![],
        };
        assert_eq!(crate::class3::check(&ir), vec![]);
    }

    /// A component assuming its input is in [0, 1] (never NaN) is fed an outside input declared [0, 1] (fine), then
    /// one declared [0, 2] (refused at the instance).
    #[test]
    fn instance_obligation_for_a_plain_range_contract() {
        use crate::expr::{Component, Use};
        let half = Component {
            name: "half".into(),
            compute: Compute {
                inputs: vec![Port {
                    range: Some([0.0, 1.0]),
                    ..port("x")
                }],
                params: vec![],
                state: vec![],
                defs: vec![
                    Def {
                        name: "y".into(),
                        expr: var("x") * num(0.5),
                    }
                    .into(),
                ],
                outputs: vec![port("y")],
                next: vec![],
            },
        };
        let user = |lo: f64, hi: f64| Compute {
            inputs: vec![Port {
                range: Some([lo, hi]),
                ..port("a")
            }],
            params: vec![],
            state: vec![],
            defs: vec![
                Stmt::Use(Use {
                    name: "h".into(),
                    component: "half".into(),
                    bind: vec![Def {
                        name: "x".into(),
                        expr: var("a"),
                    }],
                }),
                Def {
                    name: "b".into(),
                    expr: var("h__y"),
                }
                .into(),
            ],
            outputs: vec![port("b")],
            next: vec![],
        };
        let ir = |hi| Ir {
            version: IR_VERSION,
            base_rate_hz: 8,
            blocks: vec![
                block("src", 8, None),
                block("u", 8, Some(user(0.0, hi))),
                block("out", 8, None),
            ],
            edges: vec![
                edge("src", "a", "u", "a", None),
                edge("u", "b", "out", "b", None),
            ],
            components: vec![half.clone()],
        };
        assert_eq!(crate::class3::check(&ir(1.0)), vec![]);
        let bad = crate::class3::check(&ir(2.0));
        assert!(
            bad.len() == 1
                && bad[0].code == "C3-CONTRACT"
                && bad[0].msg.starts_with("u.h.x (half)"),
            "{bad:?}"
        );
    }

    #[test]
    fn wiring_errors_are_refused() {
        let err = |f: &dyn Fn(&mut Ir)| {
            let mut ir = ir();
            f(&mut ir);
            firmware(&ir).err().unwrap_or_default().join("; ")
        };
        assert!(
            err(&|ir| {
                ir.edges.remove(0);
            })
            .contains("fast.x: 0 edges")
        );
        assert!(
            err(&|ir| ir.edges.push(edge("src", "x2", "fast", "x", None)))
                .contains("fast.x: 2 edges")
        );
        assert!(
            err(&|ir| ir.edges[1].from_port = Some("nope".into())).contains("slow has no output")
        );
        assert!(err(&|ir| ir.edges[0].to_port = Some("nope".into())).contains("fast has no input"));
        assert!(
            err(&|ir| ir.edges[1].delay_ticks = 1).contains("delays on edges into the firmware")
        );
        assert!(
            err(&|ir| ir.edges[0].delay_ticks = 1).contains("delays on edges into the firmware")
        );
        assert!(
            err(&|ir| ir.edges[0].from_port = Some("a__b".into())).contains("not an identifier")
        );
        let unit = err(&|ir| {
            let fast = ir.blocks[2].compute.as_mut().unwrap();
            fast.inputs[1].unit = Some("V".into());
        });
        assert!(unit.contains("type or unit mismatch"), "{unit}");
        assert!(err(&|ir| ir.blocks[1].rate_hz = 3).contains("does not divide"));
    }
}
