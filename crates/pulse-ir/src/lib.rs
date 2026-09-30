//! Pulse IR: blocks, rates, and the edges between them.
//! Frontends (hand-written today; Python/Modelica later) build an `Ir`; `class1::check` proves temporal properties from it.

pub mod class1;
pub mod class3;
pub mod copper;
pub mod expr;
pub mod rust;

use serde::{Deserialize, Serialize};

/// Bumped on any breaking change to the serialized shape.
pub const IR_VERSION: u32 = 3;

#[derive(Debug, PartialEq, Serialize)]
pub struct Violation {
    pub check: &'static str,
    /// Stable, machine-readable id (CLI, CI and MCP key on this, never on `msg`).
    pub code: &'static str,
    pub msg: String,
}

pub(crate) fn v(check: &'static str, code: &'static str, msg: String) -> Violation {
    Violation { check, code, msg }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Formalism {
    Continuous,
    Discrete,
    StateMachine,
}

/// A sensor is not an ideal read of plant state: it has latency, quantization, dropout.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorSpec {
    /// Counted in the sensor's own samples (one per firing), like `max_dropout_run`.
    pub latency_ticks: u32,
    pub quant_step: f32,
    pub dropout_p: f32,
    /// Contract: never more than this many consecutive dropouts. Without it staleness has no deterministic bound.
    pub max_dropout_run: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Block {
    pub id: String,
    pub formalism: Formalism,
    pub rate_hz: u32,
    /// Timing contract: every call must finish within this, including the republish calls of a slow block between
    /// its firings (Copper runs every task on every base tick). Proving WCET <= budget is a separate step (NOTES.md D-001).
    pub wcet_budget_ns: u64,
    pub sensor: Option<SensorSpec>,
    /// Rust type implementing this block as a runtime task (e.g. `tasks::Sensor`). `None`: not a task (simulated plant).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub imp: Option<String>,
    /// Where the frontend declared this (e.g. `model.py:42`), so violations point at the source.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub span: Option<String>,
    /// What the block computes on each firing, as data: codegen, proofs and the reference model all read this.
    /// `None`: behaviour lives only in `imp` (hand-written, not verified).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub compute: Option<expr::Compute>,
}

/// How a cross-rate read is made legal. Never implicit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Hold {
    /// Fast producer, slow consumer: latch every `factor`-th sample.
    Decimate(u32),
    /// Slow producer, fast consumer: producer's last output is held between activations.
    Zoh,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Edge {
    pub from: String,
    pub to: String,
    pub hold: Option<Hold>,
    /// Declared bound on the age of the value at the consumer, from the producer computing (or sampling) it to the end
    /// of the last tick the consumer still acts on it; checked against the derived worst case.
    pub max_age_ns: Option<u64>,
    /// Ticks between the producer writing and the consumer seeing the value (Modelica `previous`). Every feedback
    /// loop needs at least one edge with a delay, otherwise the loop is an algebraic loop with no evaluation order.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub delay_ticks: u32,
    /// Payload type carried on this edge (e.g. `crate::tasks::SensorSample`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub msg: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub span: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ir {
    pub version: u32,
    pub base_rate_hz: u32,
    pub blocks: Vec<Block>,
    pub edges: Vec<Edge>,
    /// Reusable behaviour that block computes (and other components) instantiate by name.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub components: Vec<expr::Component>,
}

fn is_zero(n: &u32) -> bool {
    *n == 0
}

pub(crate) fn is_ident(s: &str) -> bool {
    let mut c = s.chars();
    c.next()
        .is_some_and(|f| f.is_ascii_alphabetic() || f == '_')
        && c.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

/// `a::b::C`: what `imp` and `msg` must look like, since codegen pastes them into Rust.
fn is_path(s: &str) -> bool {
    s.split("::").all(is_ident)
}

impl Ir {
    /// A block's behaviour with every component inlined: what proofs, the interpreter and codegen read.
    pub fn flat(&self, c: &expr::Compute) -> Result<expr::Compute, String> {
        c.flatten(&self.components)
    }

    /// Names are identifiers without `__` (reserved for flattened names); the flat form is well-typed.
    fn check_compute(&self, c: &expr::Compute) -> Vec<String> {
        use expr::Stmt;
        let mut names: Vec<&str> = c.inputs.iter().map(|p| p.name.as_str()).collect();
        names.extend(c.params.iter().map(|p| p.name.as_str()));
        names.extend(c.state.iter().map(|s| s.name.as_str()));
        names.extend(c.defs.iter().map(|s| match s {
            Stmt::Let(d) => d.name.as_str(),
            Stmt::Use(u) => u.name.as_str(),
        }));
        let mut bad: Vec<String> = names
            .iter()
            .filter(|n| n.contains("__"))
            .map(|n| format!("{n:?}: `__` is reserved for flattened names"))
            .collect();
        match self.flat(c) {
            Ok(f) => bad.extend(f.validate()),
            Err(e) => bad.push(e),
        }
        bad
    }

    pub fn block(&self, id: &str) -> Option<&Block> {
        self.blocks.iter().find(|b| b.id == id)
    }

    /// Well-formedness every pass relies on: known version, unique identifier ids, Rust-path `imp`/`msg`.
    /// The IR arrives from frontends and agents, so this is the trust boundary.
    pub fn validate(&self) -> Vec<Violation> {
        let mut bad = Vec::new();
        if self.version != IR_VERSION {
            bad.push(v(
                "ir",
                "IR-VERSION",
                format!("IR version {}, expected {IR_VERSION}", self.version),
            ));
        }
        for (i, b) in self.blocks.iter().enumerate() {
            if !is_ident(&b.id) {
                bad.push(v(
                    "ir",
                    "IR-NAME",
                    format!("block id {:?} is not an identifier", b.id),
                ));
            }
            if self.blocks[..i].iter().any(|o| o.id == b.id) {
                bad.push(v(
                    "ir",
                    "IR-DUP-ID",
                    format!("block id {:?} declared twice", b.id),
                ));
            }
            if let Some(c) = &b.compute {
                for e in self.check_compute(c) {
                    bad.push(v("ir", "IR-COMPUTE", format!("{}: {e}", b.id)));
                }
            }
            if b.imp.as_deref().is_some_and(|p| !is_path(p)) {
                bad.push(v(
                    "ir",
                    "IR-NAME",
                    format!("{}: imp {:?} is not a Rust path", b.id, b.imp),
                ));
            }
        }
        for comp in &self.components {
            if !is_ident(&comp.name)
                || self
                    .components
                    .iter()
                    .filter(|c| c.name == comp.name)
                    .count()
                    > 1
            {
                bad.push(v(
                    "ir",
                    "IR-NAME",
                    format!("component {:?}: not a unique identifier", comp.name),
                ));
            }
            for e in self.check_compute(&comp.compute) {
                bad.push(v(
                    "ir",
                    "IR-COMPUTE",
                    format!("component {}: {e}", comp.name),
                ));
            }
        }
        for e in &self.edges {
            if e.msg.as_deref().is_some_and(|p| !is_path(p)) {
                bad.push(v(
                    "ir",
                    "IR-NAME",
                    format!("{} -> {}: msg {:?} is not a Rust path", e.from, e.to, e.msg),
                ));
            }
        }
        bad
    }
}
