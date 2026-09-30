//! Host-only plant: a DC motor driven into a stiff wall, with winding temperature. On target this is the real joint.

use crate::params::*;

/// Continuous state [current A, speed rad/s, position rad, winding temp C] plus the voltage held for this tick.
pub struct Plant {
    pub x: [f64; 4],
    pub v_cmd: f64,
}

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

impl Default for Plant {
    fn default() -> Self {
        Self {
            x: [0.0, 0.0, 0.0, T_AMB],
            v_cmd: 0.0,
        }
    }
}

impl Plant {
    /// One RK4 step over a base tick with the voltage held constant (zero-order hold).
    pub fn step(&mut self) {
        let (x, v) = (self.x, self.v_cmd);
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
            self.x[n] += DT / 6.0 * (k1[n] + 2.0 * k2[n] + 2.0 * k3[n] + k4[n]);
        }
    }
}
