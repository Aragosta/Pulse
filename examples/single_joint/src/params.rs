//! Every generic number for the single-joint example, in one place. Replace with real motor/thermal data later.
//! Scenario: the joint is commanded 1 rad but a stiff wall at 0.2 rad stalls it, so the loop pushes maximum current
//! until the thermal FSM goes nominal -> derating -> fault.

pub const BASE_HZ: u32 = 8000;
pub const DT: f64 = 1.0 / BASE_HZ as f64;
pub const DECIMATION: u64 = 40; // 8 kHz -> 200 Hz
/// Host timing probes ignore the first 0.5 s (cold caches, page faults).
pub const WARMUP_TICKS: u64 = BASE_HZ as u64 / 2;
pub const OUTER_DT: f32 = DECIMATION as f32 / BASE_HZ as f32;

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

// Electrical limits
pub const V_BUS: f32 = 24.0;
pub const I_MAX: f32 = 8.0;

// Thermal FSM
pub const T_DERATE: f32 = 80.0;
pub const T_RECOVER: f32 = 70.0;
pub const T_FAULT: f32 = 100.0;
pub const DERATE_SCALE: f32 = 0.75;

// Sensor (12-bit encoder, 0.01 A ADC step, 0.25 K thermistor step)
pub const LATENCY_TICKS: u32 = 2;
pub const POS_QUANT: f32 = core::f32::consts::TAU / 4096.0;
pub const CUR_QUANT: f32 = 0.01;
pub const TEMP_QUANT: f32 = 0.25;
pub const DROPOUT_P: f32 = 0.02;
pub const MAX_DROPOUT_RUN: u32 = 3;

// Controllers: position PID (200 Hz) commands current; current PI (8 kHz) commands volts.
pub const POS_KP: f32 = 1.8; // wn ~ 30 rad/s against Kt/J = 500 rad/s^2/A
pub const POS_KI: f32 = 6.0; // A/(rad*s): stall current ramps to I_MAX in ~1.5 s
pub const POS_KD: f32 = 0.108;
pub const CUR_KP: f32 = 1.26; // 400 Hz bandwidth, pole-zero cancelled against R/L
pub const CUR_KI: f32 = 1257.0;

// Timing contracts (WCET budgets), sized for a ~150 MHz Cortex-M33 class target. Summed on the busiest tick they must fit in 125 us.
pub const BUDGET_NS: [u64; 6] = [15_000, 5_000, 20_000, 10_000, 30_000, 10_000];
