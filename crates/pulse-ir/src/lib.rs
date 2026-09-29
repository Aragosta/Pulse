//! Pulse IR: blocks, rates, and the edges between them.
//! Frontends (hand-written today; Python/Modelica later) build an `Ir`; `class1::check` proves temporal properties from it.

pub mod class1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Formalism {
    Continuous,
    Discrete,
    StateMachine,
}

/// A sensor is not an ideal read of plant state: it has latency, quantization, dropout.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorSpec {
    pub latency_ticks: u32,
    pub quant_step: f32,
    pub dropout_p: f32,
    /// Contract: never more than this many consecutive dropouts. Without it staleness has no deterministic bound.
    pub max_dropout_run: u32,
}

#[derive(Clone, Debug)]
pub struct Block {
    pub id: &'static str,
    pub formalism: Formalism,
    pub rate_hz: u32,
    /// Timing contract: the block must finish within this per activation. Proving WCET <= budget is a separate (LLVMTA) step.
    pub wcet_budget_ns: u64,
    pub sensor: Option<SensorSpec>,
}

/// How a cross-rate read is made legal. Never implicit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hold {
    /// Fast producer, slow consumer: latch every `factor`-th sample.
    Decimate(u32),
    /// Slow producer, fast consumer: producer's last output is held between activations.
    Zoh,
}

#[derive(Clone, Debug)]
pub struct Edge {
    pub from: &'static str,
    pub to: &'static str,
    pub hold: Option<Hold>,
    /// Declared bound on the age of the value at the consumer; checked against the derived worst case.
    pub max_age_ns: Option<u64>,
}

#[derive(Clone, Debug)]
pub struct Ir {
    pub base_rate_hz: u32,
    pub blocks: Vec<Block>,
    pub edges: Vec<Edge>,
}

impl Ir {
    pub fn block(&self, id: &str) -> Option<&Block> {
        self.blocks.iter().find(|b| b.id == id)
    }
}
