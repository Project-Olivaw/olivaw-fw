//! The battery divider on ADC1 channel 6 (GPIO34), 11 dB attenuation.

use esp_hal::Blocking;
use esp_hal::analog::adc::{Adc, AdcConfig, AdcPin, Attenuation};
use esp_hal::peripherals::{ADC1, GPIO34};
use olivaw_battery::mean_u16;

/// Owns ADC1 and the sense pin; produces raw 12-bit codes.
pub struct BatteryAdc {
    adc: Adc<'static, ADC1<'static>, Blocking>,
    pin: AdcPin<GPIO34<'static>, ADC1<'static>>,
}

impl BatteryAdc {
    /// Configure the pin at 11 dB (full-scale ~3.1 V; the divider keeps a
    /// full 3S pack at 2.27 V, inside the ADC's linear-ish region).
    #[must_use]
    pub fn new(adc1: ADC1<'static>, sense: GPIO34<'static>) -> Self {
        let mut cfg = AdcConfig::new();
        let pin = cfg.enable_pin(sense, Attenuation::_11dB);
        let adc = Adc::new(adc1, cfg);
        Self { adc, pin }
    }

    /// One blocking conversion, raw 12-bit code. `0` on a conversion error
    /// (never observed on ESP32; kept so callers need no `Result`).
    pub fn read_raw(&mut self) -> u16 {
        nb::block!(self.adc.read_oneshot(&mut self.pin)).unwrap_or(0)
    }

    /// Mean of `N` consecutive conversions (a few hundred microseconds each).
    pub fn read_mean<const N: usize>(&mut self) -> u16 {
        let mut samples = [0u16; N];
        for s in &mut samples {
            *s = self.read_raw();
        }
        mean_u16(&samples)
    }
}
