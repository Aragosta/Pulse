//! Pulse IR: blocks, rates, and the edges between them.
//! Frontends (hand-written today; Python/Modelica later) build an `Ir`; `class1::check` proves temporal properties from it.

pub mod class1;
pub mod copper;

use serde::{Deserialize, Serialize};

/// Bumped on any breaking change to the serialized shape.
pub const IR_VERSION: u32 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Formalism {
    Continuous,
    Discrete,
    StateMachine,
}

/// A sensor is not an ideal read of plant state: it has latency, quantization, dropout.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SensorSpec {
    pub latency_ticks: u32,
    pub quant_step: f32,
    pub dropout_p: f32,
    /// Contract: never more than this many consecutive dropouts. Without it staleness has no deterministic bound.
    pub max_dropout_run: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Block {
    pub id: String,
    pub formalism: Formalism,
    pub rate_hz: u32,
    /// Timing contract: the block must finish within this per activation. Proving WCET <= budget is a separate step (NOTES.md D-001).
    pub wcet_budget_ns: u64,
    pub sensor: Option<SensorSpec>,
    /// Rust type implementing this block as a runtime task (e.g. `tasks::Sensor`). `None`: not a task (simulated plant).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub imp: Option<String>,
    /// Where the frontend declared this (e.g. `model.py:42`), so violations point at the source.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub span: Option<String>,
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
pub struct Edge {
    pub from: String,
    pub to: String,
    pub hold: Option<Hold>,
    /// Declared bound on the age of the value at the consumer; checked against the derived worst case.
    pub max_age_ns: Option<u64>,
    /// Payload type carried on this edge (e.g. `crate::tasks::SensorSample`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub msg: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub span: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ir {
    pub version: u32,
    pub base_rate_hz: u32,
    pub blocks: Vec<Block>,
    pub edges: Vec<Edge>,
}

impl Ir {
    pub fn block(&self, id: &str) -> Option<&Block> {
        self.blocks.iter().find(|b| b.id == id)
    }
}
