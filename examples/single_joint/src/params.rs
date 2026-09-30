//! Host-only numbers: plant physics, the stall scenario, timing budgets. Controller/sensor constants live in
//! `pulse_joint::params` (shared with the target) and are re-exported here.
//! Scenario: the joint is commanded 1 rad but a stiff wall at 0.2 rad stalls it, so the loop pushes maximum current
//! until the thermal FSM goes nominal -> derating -> fault.

pub use pulse_joint::params::*;

/// Host timing probes ignore the first 0.5 s (cold caches, page faults).
pub const WARMUP_TICKS: u64 = BASE_HZ as u64 / 2;

// DC motor (electrical time constant L/R = 1 ms = 8 base ticks)
pub const R: f64 = 0.5;
pub const L: f64 = 0.5e-3;
pub const KE: f64 = 0.05;
pub const KT: f64 = 0.05;
pub const J: f64 = 1e-4;
pub const B: f64 = 1e-5;

// Thermal (time constant C*R_TH = 3 s, shortened from a real winding so the demo finishes in seconds)
pub const C_TH: f64 = 0.6;
pub const R_TH: f64 = 5.0;
pub const T_AMB: f64 = 25.0;

// Stiff wall the joint is commanded into
pub const WALL_POS: f64 = 0.2;
pub const WALL_K: f64 = 100.0;
pub const WALL_D: f64 = 0.05;
pub const THETA_REF: f32 = 1.0;

// Timing contracts (WCET budgets), sized for a ~150 MHz Cortex-M33 class target. The firmware runs every block every
// tick (slow ones compute and are gated), and the sensor read and actuator write are drivers, so all must sum <= 125 us.
pub const SENSOR_BUDGET_NS: u64 = 15_000;
pub const POSITION_BUDGET_NS: u64 = 20_000;
pub const THERMAL_BUDGET_NS: u64 = 10_000;
pub const CURRENT_BUDGET_NS: u64 = 30_000;
pub const ACTUATOR_BUDGET_NS: u64 = 10_000;
