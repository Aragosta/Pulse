//! GENERATED from the Pulse IR (`examples/single_joint/src/ir.rs`). Do not edit; regenerate with
//! `PULSE_BLESS=1 cargo test -p single_joint`.

pub struct CurrentLoop {
    pub pid__integral: f32,
    pub pid__prev_meas: f32,
    pub pid__has_prev: bool,
    pub pid__filt: f32,
    pub pid__filt_init: bool,
}

impl Default for CurrentLoop {
    fn default() -> Self {
        Self::new()
    }
}

impl CurrentLoop {
    pub const fn new() -> Self {
        Self {
            pid__integral: 0.0_f32,
            pid__prev_meas: 0.0_f32,
            pid__has_prev: false,
            pid__filt: 0.0_f32,
            pid__filt_init: false,
        }
    }

    pub fn step(&mut self, v_wanted: f32, v_measured: f32, v_scale: f32, v_state: u8) -> (f32, f32,) {
        let v_state = v_state as f32;
        let v_lim = (if (v_scale >= 0.0_f32) { (8.0_f32 * v_scale.min(1.0_f32)) } else { 0.0_f32 });
        let v_setpoint = (if v_wanted.is_finite() { v_wanted.max((-v_lim)).min(v_lim) } else { 0.0_f32 });
        let v_fault = (v_state == 2.0_f32);
        let v_pid__meas = (if v_measured.is_finite() { v_measured } else { self.pid__prev_meas });
        let v_pid__error = (v_setpoint - v_pid__meas);
        let v_pid__p_term = (1.26_f32 * v_pid__error);
        let v_pid__raw_d = (if self.pid__has_prev { ((self.pid__prev_meas - v_pid__meas) / 0.000125_f32) } else { 0.0_f32 });
        let v_pid__filt_d = (if self.pid__filt_init { ((1.0_f32 * v_pid__raw_d) + ((1.0_f32 - 1.0_f32) * self.pid__filt)) } else { v_pid__raw_d });
        let v_pid__d_term = (0.0_f32 * v_pid__filt_d);
        let v_pid__cand = (self.pid__integral + ((1257.0_f32 * v_pid__error) * 0.000125_f32));
        let v_pid__unsat = ((v_pid__p_term + v_pid__cand) + v_pid__d_term);
        let v_pid__out = v_pid__unsat.max((-24.0_f32)).min(24.0_f32);
        let v_pid__deeper = (((v_pid__unsat > 24.0_f32) && (v_pid__error > 0.0_f32)) || ((v_pid__unsat < (-24.0_f32)) && (v_pid__error < 0.0_f32)));
        let v_volts = (if v_fault { 0.0_f32 } else { v_pid__out });
        let n_pid__integral = (if v_fault { 0.0_f32 } else { (if v_pid__deeper { self.pid__integral } else { v_pid__cand }).max((-24.0_f32)).min(24.0_f32) });
        let n_pid__prev_meas = (if v_fault { 0.0_f32 } else { v_pid__meas });
        let n_pid__has_prev = (!v_fault);
        let n_pid__filt = (if v_fault { 0.0_f32 } else { v_pid__filt_d });
        let n_pid__filt_init = (!v_fault);
        let out = (v_volts, v_setpoint,);
        self.pid__integral = n_pid__integral;
        self.pid__prev_meas = n_pid__prev_meas;
        self.pid__has_prev = n_pid__has_prev;
        self.pid__filt = n_pid__filt;
        self.pid__filt_init = n_pid__filt_init;
        out
    }
}
