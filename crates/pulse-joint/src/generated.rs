//! GENERATED from the Pulse IR (`examples/single_joint/src/ir.rs`). Do not edit; regenerate with
//! `PULSE_BLESS=1 cargo test -p single_joint`.

pub struct CurrentLoop {
    pub integral: f32,
    pub prev_meas: f32,
    pub has_prev: bool,
    pub filt: f32,
    pub filt_init: bool,
}

impl Default for CurrentLoop {
    fn default() -> Self {
        Self::new()
    }
}

impl CurrentLoop {
    pub const fn new() -> Self {
        Self {
            integral: 0.0_f32,
            prev_meas: 0.0_f32,
            has_prev: false,
            filt: 0.0_f32,
            filt_init: false,
        }
    }

    pub fn step(&mut self, v_wanted: f32, v_measured: f32, v_scale: f32, v_state: u8) -> (f32, f32,) {
        let v_state = v_state as f32;
        let v_lim = (if (v_scale >= 0.0_f32) { (8.0_f32 * v_scale.min(1.0_f32)) } else { 0.0_f32 });
        let v_setpoint = (if v_wanted.is_finite() { v_wanted.max((-v_lim)).min(v_lim) } else { 0.0_f32 });
        let v_meas = (if v_measured.is_finite() { v_measured } else { self.prev_meas });
        let v_error = (v_setpoint - v_meas);
        let v_p_term = (1.26_f32 * v_error);
        let v_raw_d = (if self.has_prev { ((self.prev_meas - v_meas) / 0.000125_f32) } else { 0.0_f32 });
        let v_filt_d = (if self.filt_init { ((1.0_f32 * v_raw_d) + ((1.0_f32 - 1.0_f32) * self.filt)) } else { v_raw_d });
        let v_d_term = (0.0_f32 * v_filt_d);
        let v_cand = (self.integral + ((1257.0_f32 * v_error) * 0.000125_f32));
        let v_unsat = ((v_p_term + v_cand) + v_d_term);
        let v_pid_out = v_unsat.max((-24.0_f32)).min(24.0_f32);
        let v_deeper = (((v_unsat > 24.0_f32) && (v_error > 0.0_f32)) || ((v_unsat < (-24.0_f32)) && (v_error < 0.0_f32)));
        let v_reset = (v_state == 2.0_f32);
        let v_volts = (if v_reset { 0.0_f32 } else { v_pid_out });
        let n_integral = (if v_reset { 0.0_f32 } else { (if v_deeper { self.integral } else { v_cand }).max((-24.0_f32)).min(24.0_f32) });
        let n_prev_meas = (if v_reset { 0.0_f32 } else { v_meas });
        let n_has_prev = (!v_reset);
        let n_filt = (if v_reset { 0.0_f32 } else { v_filt_d });
        let n_filt_init = (!v_reset);
        let out = (v_volts, v_setpoint,);
        self.integral = n_integral;
        self.prev_meas = n_prev_meas;
        self.has_prev = n_has_prev;
        self.filt = n_filt;
        self.filt_init = n_filt_init;
        out
    }
}
