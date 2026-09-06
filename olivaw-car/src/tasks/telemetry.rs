//! Snapshots the shared state into a `Telemetry` frame at 5 Hz and logs a
//! one-line summary every 5 s.

use embassy_time::{Duration, Ticker};

use crate::config;
use crate::shared::{TELEMETRY, uptime_s, with_state};

/// Publishes to the `TELEMETRY` watch.
#[embassy_executor::task]
pub async fn run() {
    let mut ticker = Ticker::every(Duration::from_millis(config::TELEMETRY_PERIOD_MS));
    let sender = TELEMETRY.sender();
    let mut tick: u32 = 0;
    loop {
        ticker.next().await;
        let t = with_state(|s| s.snapshot(uptime_s()));
        sender.send(t);
        tick = tick.wrapping_add(1);
        if tick.is_multiple_of(config::BATTERY_LEVEL_EVERY_TICKS) {
            log::info!(
                "t={}s batt {} mV {}% L{} R{} cap{} flags={:#04x} rssi {} lidar {:?} wifi {:?} mqtt {:?}",
                t.uptime_s,
                t.battery_mv,
                t.battery_pct,
                t.left,
                t.right,
                t.max_duty_permille,
                t.flags.0,
                t.rssi_dbm,
                t.lidar,
                t.wifi,
                t.mqtt
            );
        }
    }
}
