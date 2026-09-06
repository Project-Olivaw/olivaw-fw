//! Onboard LED: slow blink while advertising, solid while a phone is
//! connected, fast blink while E-stopped or the battery is critical.

use embassy_time::{Duration, Timer};
use esp_hal::gpio::Output;
use olivaw_proto::Flags;

use crate::shared::with_state;

/// Owns GPIO2.
#[embassy_executor::task]
pub async fn run(mut led: Output<'static>) {
    loop {
        let flags = with_state(|s| s.flags);
        let fault = flags.contains(Flags::E_STOP) || flags.contains(Flags::CRITICAL_BATTERY);
        let (on_ms, off_ms) = if fault {
            (100, 100)
        } else if flags.contains(Flags::BLE_CONNECTED) {
            (1000, 0)
        } else {
            (100, 900)
        };
        led.set_high();
        Timer::after(Duration::from_millis(on_ms)).await;
        if off_ms > 0 {
            led.set_low();
            Timer::after(Duration::from_millis(off_ms)).await;
        }
    }
}
