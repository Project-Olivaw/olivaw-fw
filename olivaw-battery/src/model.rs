//! Raw ADC code → millivolts at the pin.
//!
//! The classic ESP32's ADC is non-linear and has a per-chip offset, and the
//! esp-hal 1.1 line ships no calibration for it. A two-point linear fit
//! measured once with a multimeter is accurate to ~1 % over the 1.5–2.3 V
//! span the divider produces, which is all the car needs. Replace with
//! `AdcCalLine` when the firmware moves to esp-hal 1.2.

/// Two-point linear model `raw → mV`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdcModel {
    /// Raw code at the low calibration point.
    pub raw_lo: u16,
    /// Millivolts at the low calibration point.
    pub mv_lo: u32,
    /// Raw code at the high calibration point.
    pub raw_hi: u16,
    /// Millivolts at the high calibration point.
    pub mv_hi: u32,
}

impl AdcModel {
    /// Nominal, uncalibrated ESP32 at 11 dB / 12-bit: ~0.14 V offset,
    /// ~3.1 V full scale. Good to ±5 %; calibrate for better.
    pub const ESP32_11DB_NOMINAL: AdcModel = AdcModel {
        raw_lo: 190,
        mv_lo: 150,
        raw_hi: 4095,
        mv_hi: 3100,
    };

    /// Millivolts at the pin for `raw`. Clamps below the low point to zero
    /// and extrapolates linearly above the high point.
    #[must_use]
    pub fn pin_mv(&self, raw: u16) -> u32 {
        let (raw_lo, raw_hi) = (i64::from(self.raw_lo), i64::from(self.raw_hi));
        let (mv_lo, mv_hi) = (i64::from(self.mv_lo), i64::from(self.mv_hi));
        if raw_hi <= raw_lo {
            return 0;
        }
        let raw = i64::from(raw);
        if raw < raw_lo {
            return 0;
        }
        let mv = mv_lo + (raw - raw_lo) * (mv_hi - mv_lo) / (raw_hi - raw_lo);
        u32::try_from(mv.max(0)).unwrap_or(0)
    }
}

/// Integer mean of ADC samples (empty → 0).
#[must_use]
pub fn mean_u16(samples: &[u16]) -> u16 {
    if samples.is_empty() {
        return 0;
    }
    let sum: u32 = samples.iter().map(|&s| u32::from(s)).sum();
    let n = u32::try_from(samples.len()).unwrap_or(u32::MAX);
    u16::try_from(sum / n).unwrap_or(u16::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_model_hits_both_calibration_points() {
        let m = AdcModel::ESP32_11DB_NOMINAL;
        assert_eq!(m.pin_mv(m.raw_lo), m.mv_lo);
        assert_eq!(m.pin_mv(m.raw_hi), m.mv_hi);
    }

    #[test]
    fn is_monotonic_and_clamps_low() {
        let m = AdcModel::ESP32_11DB_NOMINAL;
        assert_eq!(m.pin_mv(0), 0);
        assert!(m.pin_mv(2000) < m.pin_mv(3000));
    }

    #[test]
    fn degenerate_points_read_zero() {
        let m = AdcModel {
            raw_lo: 100,
            mv_lo: 0,
            raw_hi: 100,
            mv_hi: 1,
        };
        assert_eq!(m.pin_mv(4000), 0);
    }

    #[test]
    fn mean_of_samples() {
        assert_eq!(mean_u16(&[]), 0);
        assert_eq!(mean_u16(&[10, 20, 30]), 20);
        assert_eq!(mean_u16(&[u16::MAX; 4]), u16::MAX);
    }
}
