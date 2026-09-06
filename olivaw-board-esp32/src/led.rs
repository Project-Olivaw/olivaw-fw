//! The onboard status LED (GPIO2, active high on `DevKit V1`).

use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::GPIO2;

/// Wrap GPIO2 as a push-pull output, initially off.
#[must_use]
pub fn init(pin: GPIO2<'static>) -> Output<'static> {
    Output::new(pin, Level::Low, OutputConfig::default())
}
