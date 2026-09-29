//! Host-only simulation state and measurement probes. On target the sensor/actuator tasks read ADC/encoder and drive PWM
//! instead, and these probes become cycle-counter reads. Nothing here allocates.

use crate::params::*;
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
use std::sync::{Mutex, OnceLock};
use std::time::Instant;

/// Task indices into the probe arrays; same order as `params::BUDGET_NS` and `ir::single_joint()`.
pub const SENSOR: usize = 0;
pub const HOLD: usize = 1;
pub const POS: usize = 2;
pub const FSM: usize = 3;
pub const CUR: usize = 4;
pub const ACT: usize = 5;
pub const NAMES: [&str; 6] = [
    "sensor",
    "hold_200",
    "position_loop",
    "thermal_fsm",
    "current_loop",
    "actuator",
];

/// Continuous plant state [current A, speed rad/s, position rad, winding temp C] plus the voltage held for this tick.
pub struct Sim {
    pub x: [f64; 4],
    pub v_cmd: f64,
}
pub static SIM: Mutex<Sim> = Mutex::new(Sim {
    x: [0.0, 0.0, 0.0, T_AMB],
    v_cmd: 0.0,
});

fn deriv(x: [f64; 4], v: f64) -> [f64; 4] {
    let [i, w, th, t] = x;
    let wall = if th > WALL_POS {
        -WALL_K * (th - WALL_POS) - WALL_D * w
    } else {
        0.0
    };
    [
        (v - R * i - KE * w) / L,
        (KT * i - B * w + wall) / J,
        w,
        (i * i * R - (t - T_AMB) / R_TH) / C_TH,
    ]
}

/// One RK4 step of the plant over a base tick with the voltage held constant (zero-order hold).
pub fn step(s: &mut Sim) {
    let (x, v) = (s.x, s.v_cmd);
    let add = |a: [f64; 4], k: [f64; 4], h: f64| {
        [
            a[0] + h * k[0],
            a[1] + h * k[1],
            a[2] + h * k[2],
            a[3] + h * k[3],
        ]
    };
    let k1 = deriv(x, v);
    let k2 = deriv(add(x, k1, DT / 2.0), v);
    let k3 = deriv(add(x, k2, DT / 2.0), v);
    let k4 = deriv(add(x, k3, DT), v);
    for n in 0..4 {
        s.x[n] += DT / 6.0 * (k1[n] + 2.0 * k2[n] + 2.0 * k3[n] + k4[n]);
    }
}

pub struct Stat {
    pub max_ns: AtomicU64,
    pub max_crit_ns: AtomicU64,
}
pub static TASK: [Stat; 6] = [const {
    Stat {
        max_ns: AtomicU64::new(0),
        max_crit_ns: AtomicU64::new(0),
    }
}; 6];

/// Run `f`, recording its wall time. `critical` = the tick where every rate fires.
pub fn timed<R>(idx: usize, critical: bool, f: impl FnOnce() -> R) -> R {
    let t = Instant::now();
    let r = f();
    let ns = t.elapsed().as_nanos() as u64;
    if TICKS.load(Relaxed) < WARMUP_TICKS {
        return r;
    }
    TASK[idx].max_ns.fetch_max(ns, Relaxed);
    if critical {
        TASK[idx].max_crit_ns.fetch_max(ns, Relaxed);
    }
    r
}

/// First tick at which the thermal FSM was in each state (nominal, derating, fault); u64::MAX = never.
pub static STATE_FIRST_TICK: [AtomicU64; 3] = [
    AtomicU64::new(u64::MAX),
    AtomicU64::new(u64::MAX),
    AtomicU64::new(u64::MAX),
];
/// Largest current setpoint (mA) the current loop accepted while derating.
pub static DERATE_SETPOINT_MAX_MA: AtomicU64 = AtomicU64::new(0);
pub static TICKS: AtomicU64 = AtomicU64::new(0);
pub static AGE_MAX_TICKS: AtomicU64 = AtomicU64::new(0);
pub static SPAN_MAX_NS: AtomicU64 = AtomicU64::new(0);
pub static SPAN_CRIT_MAX_NS: AtomicU64 = AtomicU64::new(0);
pub static JITTER_MAX_NS: AtomicU64 = AtomicU64::new(0);
static TICK_START_NS: AtomicU64 = AtomicU64::new(0);
static PREV_START_NS: AtomicU64 = AtomicU64::new(0);
static EPOCH: OnceLock<Instant> = OnceLock::new();

fn now_ns() -> u64 {
    EPOCH.get_or_init(Instant::now).elapsed().as_nanos() as u64
}

/// Called first each tick (by the sensor): records tick-start jitter against the nominal period.
pub fn tick_start() {
    let now = now_ns();
    let prev = PREV_START_NS.swap(now, Relaxed);
    TICK_START_NS.store(now, Relaxed);
    TICKS.fetch_add(1, Relaxed);
    if prev != 0 && TICKS.load(Relaxed) > WARMUP_TICKS {
        let period = 1_000_000_000 / BASE_HZ as u64;
        JITTER_MAX_NS.fetch_max((now - prev).abs_diff(period), Relaxed);
    }
}

/// Called last each tick (by the actuator): span of the whole graph within this tick.
pub fn tick_end(critical: bool) {
    if TICKS.load(Relaxed) < WARMUP_TICKS {
        return;
    }
    let span = now_ns() - TICK_START_NS.load(Relaxed);
    SPAN_MAX_NS.fetch_max(span, Relaxed);
    if critical {
        SPAN_CRIT_MAX_NS.fetch_max(span, Relaxed);
    }
}
