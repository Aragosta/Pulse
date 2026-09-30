//! Controller, sensor and limit constants shared by host and target. Plant physics and the stall scenario live in
//! the host example only.

pub const BASE_HZ: u32 = 8000;
pub const DT: f64 = 1.0 / BASE_HZ as f64;
pub const DECIMATION: u64 = 40; // 8 kHz -> 200 Hz
pub const OUTER_DT: f32 = DECIMATION as f32 / BASE_HZ as f32;

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
/// Current sense full scale: a good current sample is within +-this. A bad one may be anything (NaN, inf).
pub const I_SENSE_MAX: f32 = 64.0;
pub const DROPOUT_P: f32 = 0.02;
pub const MAX_DROPOUT_RUN: u32 = 3;

// Controllers: position PID (200 Hz) commands current; current PI (8 kHz) commands volts.
pub const POS_KP: f32 = 1.8; // wn ~ 30 rad/s against Kt/J = 500 rad/s^2/A
pub const POS_KI: f32 = 6.0; // A/(rad*s): stall current ramps to I_MAX in ~1.5 s
pub const POS_KD: f32 = 0.108;
pub const CUR_KP: f32 = 1.26; // 400 Hz bandwidth, pole-zero cancelled against R/L
pub const CUR_KI: f32 = 1257.0;
