//! The resistive divider between the pack and the ADC pin.
//!
//! ```text
//! PACK+ ──[ r_top ]──┬──[ r_bottom ]── GND
//!                    └── ADC pin (+100 nF to GND)
//! ```
//!
//! For the car: 100 kΩ / 22 kΩ → 12.6 V full 3S reads 2.27 V at the pin,
//! under the ESP32's ~2.5 V linear limit at 11 dB attenuation.

// Every u64→u32 cast in this file goes through `saturate`.
#![allow(clippy::cast_possible_truncation)]

/// A two-resistor divider, ohms.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Divider {
    /// Resistor between pack positive and the ADC node.
    pub r_top_ohm: u32,
    /// Resistor between the ADC node and ground.
    pub r_bottom_ohm: u32,
}

impl Divider {
    /// The car's divider: 100 kΩ over 22 kΩ.
    pub const CAR_3S: Divider = Divider {
        r_top_ohm: 100_000,
        r_bottom_ohm: 22_000,
    };

    /// Pack voltage from the voltage at the ADC node, millivolts.
    #[must_use]
    pub const fn pack_mv(&self, pin_mv: u32) -> u32 {
        if self.r_bottom_ohm == 0 {
            return 0;
        }
        // u64 keeps 3300 mV × 122 kΩ comfortably inside range; the result
        // is back under u32 for any real divider, saturate otherwise.
        let num = pin_mv as u64 * (self.r_top_ohm as u64 + self.r_bottom_ohm as u64);
        saturate(num / self.r_bottom_ohm as u64)
    }

    /// ADC-node voltage for a given pack voltage, millivolts (for tests and
    /// for sanity-checking a divider against the ADC's range).
    #[must_use]
    pub const fn pin_mv(&self, pack_mv: u32) -> u32 {
        let total = self.r_top_ohm as u64 + self.r_bottom_ohm as u64;
        if total == 0 {
            return 0;
        }
        saturate(pack_mv as u64 * self.r_bottom_ohm as u64 / total)
    }
}

const fn saturate(v: u64) -> u32 {
    if v > u32::MAX as u64 {
        u32::MAX
    } else {
        v as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn car_divider_keeps_full_3s_under_the_adc_limit() {
        let pin = Divider::CAR_3S.pin_mv(12_600);
        assert!((2_250..=2_300).contains(&pin), "pin {pin} mV");
        assert!(pin < 2_500);
    }

    #[test]
    fn round_trips_within_rounding() {
        for pack in [9_000, 11_100, 12_600] {
            let back = Divider::CAR_3S.pack_mv(Divider::CAR_3S.pin_mv(pack));
            assert!(
                (i64::from(back) - i64::from(pack)).abs() <= 6,
                "{pack} → {back}"
            );
        }
    }

    #[test]
    fn degenerate_divider_reads_zero() {
        let d = Divider {
            r_top_ohm: 0,
            r_bottom_ohm: 0,
        };
        assert_eq!(d.pack_mv(1000), 0);
        assert_eq!(d.pin_mv(1000), 0);
    }
}
