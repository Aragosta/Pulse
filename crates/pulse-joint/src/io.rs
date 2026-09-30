//! The boundary between the chip's peripherals and the generated step: raw readings into the units and contracts the
//! IR declares, and the step's volts into PWM. Stateless on purpose: behaviour with memory belongs in the IR, where it
//! is proved; this only converts and enforces, and its tests cover every ADC code.
//!
//! Calibration is for the reference wiring below (Raspberry Pi Pico 2). Different parts: change the constants, the
//! tests re-check the contracts.
//!   current  bidirectional sense amp, 0 A at mid-rail, 0.1 V/A (INA240A1-class, 5 mOhm shunt) on ADC0 / GPIO26
//!   temp     10 kOhm NTC (B 3950) to ground, 10 kOhm pull-up to 3.3 V, on ADC1 / GPIO27
//!   command  potentiometer across 3.3 V on ADC2 / GPIO28, full travel = +-COMMAND_SPAN rad
//!   theta    quadrature encoder, COUNTS_PER_REV counts per revolution, counted by PIO
//!   volts    two-input H-bridge (DRV8871-class) on PWM slice 0: IN1 = GPIO16, IN2 = GPIO17, 20 kHz

use crate::generated::{SENSOR__CURRENT, SENSOR__TEMP, SENSOR__THETA};
use crate::params::V_BUS;

/// 12-bit ADC against 3.3 V.
pub const ADC_MAX: u16 = 4095;
const VREF: f32 = 3.3;

// ---- calibration knobs -----------------------------------------------------------------------------------------------
/// ADC code at 0 A. Measure with the motor disconnected.
pub const CURRENT_ZERO: f32 = 2048.0;
/// Amps per ADC code: VREF / 4096 / (0.1 V/A). Negate if the sense amp is wired the other way round.
pub const AMPS_PER_CODE: f32 = VREF / 4096.0 / 0.1;
pub const NTC_R25: f32 = 10_000.0;
pub const NTC_BETA: f32 = 3950.0;
pub const NTC_PULLUP: f32 = 10_000.0;
pub const COUNTS_PER_REV: f32 = 4096.0;
/// +1 or -1 so that positive volts turn the encoder positive. Wrong sign = positive feedback: check it first.
pub const THETA_SIGN: f32 = 1.0;
pub const COMMAND_SPAN: f32 = core::f32::consts::PI;
/// PWM wrap for 20 kHz at a 150 MHz system clock: 150e6 / 20e3 - 1.
pub const PWM_TOP: u16 = 7499;

/// A glitchy input's contract (SEMANTICS.md §6): a good sample inside its range, a bad one NaN. Whatever else the
/// hardware produces (an open or shorted sensor, a runaway count) becomes NaN, which the step is proved to handle.
pub fn admit(x: f32, [lo, hi]: [f32; 2]) -> f32 {
    if x >= lo && x <= hi { x } else { f32::NAN }
}

pub fn current_amps(code: u16) -> f32 {
    admit(
        (code as f32 - CURRENT_ZERO) * AMPS_PER_CODE,
        SENSOR__CURRENT,
    )
}

/// Beta equation. An open (full scale) or shorted (zero) thermistor gives NaN, which the thermal FSM treats as fault.
pub fn temp_c(code: u16) -> f32 {
    let code = code.min(ADC_MAX) as f32;
    let r = NTC_PULLUP * code / (ADC_MAX as f32 - code);
    let inv_t = 1.0 / 298.15 + libm::logf(r / NTC_R25) / NTC_BETA;
    admit(1.0 / inv_t - 273.15, SENSOR__TEMP)
}

pub fn theta_rad(count: i32) -> f32 {
    admit(
        THETA_SIGN * count as f32 * (core::f32::consts::TAU / COUNTS_PER_REV),
        SENSOR__THETA,
    )
}

/// The command has no contract (the step clamps it and holds the last good one), so this only scales.
pub fn command_rad(code: u16) -> f32 {
    (code.min(ADC_MAX) as f32 / ADC_MAX as f32 * 2.0 - 1.0) * COMMAND_SPAN
}

/// Compare values for (IN1, IN2): positive volts drive IN1, negative IN2, the other held low. Anything that is not a
/// finite voltage coasts (both low). The step is proved never to output NaN; this does not rely on it.
pub fn pwm(volts: f32) -> [u16; 2] {
    if !volts.is_finite() {
        return [0, 0];
    }
    // `as u16` saturates, and the duty is already within [0, 1].
    let c = ((volts.abs() / V_BUS).min(1.0) * PWM_TOP as f32) as u16;
    if volts > 0.0 { [c, 0] } else { [0, c] }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::generated::Firmware;

    fn meets(x: f32, [lo, hi]: [f32; 2]) -> bool {
        x.is_nan() || (lo <= x && x <= hi)
    }

    /// Every one of the 4096 codes: the value handed to the step meets its contract, so the proof's assumption
    /// about these inputs is discharged here rather than trusted.
    #[test]
    fn every_adc_code_meets_its_contract() {
        for code in 0..=ADC_MAX {
            assert!(meets(current_amps(code), SENSOR__CURRENT), "current {code}");
            assert!(meets(temp_c(code), SENSOR__TEMP), "temp {code}");
            assert!(command_rad(code).is_finite(), "command {code}");
        }
        // Out-of-spec codes (a 16-bit read of a 12-bit ADC) too.
        for code in [4096, u16::MAX] {
            assert!(meets(current_amps(code), SENSOR__CURRENT));
            assert!(meets(temp_c(code), SENSOR__TEMP));
        }
    }

    #[test]
    fn calibration_lands_where_the_parts_say() {
        assert_eq!(current_amps(2048), 0.0);
        assert!((current_amps(2048 + 124) - 1.0).abs() < 0.01, "0.1 V/A");
        assert!((temp_c(2048) - 25.0).abs() < 0.1, "divider midpoint is R25");
        assert!(
            temp_c(0).is_nan() && temp_c(ADC_MAX).is_nan(),
            "short, open"
        );
        assert_eq!(theta_rad(0), 0.0);
        assert!((theta_rad(4096) - core::f32::consts::TAU).abs() < 1e-5);
        assert!((command_rad(0) + COMMAND_SPAN).abs() < 1e-6);
        assert!((command_rad(ADC_MAX) - COMMAND_SPAN).abs() < 1e-6);
    }

    #[test]
    fn a_runaway_count_is_a_bad_sample() {
        for c in [i32::MIN, -65_202, 65_202, i32::MAX] {
            assert!(theta_rad(c).is_nan(), "{c}");
        }
        for c in [-65_000, -1, 1, 65_000] {
            assert!(theta_rad(c).is_finite(), "{c}");
        }
    }

    #[test]
    fn admit_is_inclusive_and_refuses_non_finite() {
        let r = [-1.0, 1.0];
        assert_eq!(admit(-1.0, r), -1.0);
        assert_eq!(admit(1.0, r), 1.0);
        for x in [
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            1.000_001,
            -1.000_001,
        ] {
            assert!(admit(x, r).is_nan(), "{x}");
        }
    }

    /// PWM over every sign/magnitude class and a million random f32 bit patterns: never above TOP, never both
    /// half-bridges driven, direction follows the sign, non-finite coasts.
    #[test]
    fn pwm_is_safe_for_any_f32() {
        let mut s = 0x9e37_79b9_7f4a_7c15u64;
        let check = |v: f32| {
            let [a, b] = pwm(v);
            assert!(a <= PWM_TOP && b <= PWM_TOP, "{v}");
            assert!(a == 0 || b == 0, "{v}: both driven");
            if !v.is_finite() {
                assert_eq!([a, b], [0, 0], "{v}");
            } else if v > 0.0 {
                assert_eq!(b, 0, "{v}");
            } else {
                assert_eq!(a, 0, "{v}");
            }
        };
        for v in [
            0.0,
            -0.0,
            1e-45,
            -1e-45,
            12.0,
            -12.0,
            24.0,
            -24.0,
            1e30,
            f32::NAN,
        ] {
            check(v);
        }
        for _ in 0..1_000_000 {
            s ^= s >> 12;
            s ^= s << 25;
            s ^= s >> 27;
            check(f32::from_bits(
                (s.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 32) as u32,
            ));
        }
        assert_eq!(pwm(24.0), [PWM_TOP, 0]);
        assert_eq!(pwm(-24.0), [0, PWM_TOP]);
        assert_eq!(pwm(12.0)[0], PWM_TOP / 2);
    }

    /// A thermistor that comes loose is a hardware fault the proofs never saw by name: through this boundary it is
    /// a NaN sample, and the generated step answers it with the fault state and zero volts.
    #[test]
    fn a_loose_thermistor_faults_the_joint() {
        for code in [0, ADC_MAX] {
            let mut fw = Firmware::new();
            let (mut volts, mut state) = (f32::NAN, 0.0);
            for _ in 0..40 {
                (volts, state, _) = fw.step(theta_rad(0), current_amps(2048), temp_c(code), 1.0);
            }
            assert_eq!((state, volts), (2.0, 0.0), "code {code}");
            assert_eq!(pwm(volts), [0, 0]);
        }
    }
}
