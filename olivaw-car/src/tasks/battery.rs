//! 2 Hz battery sampling: ADC → mV → % → Ok/Low/Critical, with the critical
//! state inhibiting the drive.

use embassy_time::{Duration, Ticker};
use olivaw_battery::{BatteryMonitor, BatteryState, pack_percent};
use olivaw_board_esp32::BatteryAdc;
use olivaw_proto::Flags;

use crate::config;
use crate::shared::{DRIVE_EVENTS, DriveEvent, with_state};

/// Owns the ADC.
#[embassy_executor::task]
pub async fn run(mut adc: BatteryAdc) {
    let mut monitor = BatteryMonitor::new(config::BATTERY_MONITOR);
    let mut ticker = Ticker::every(Duration::from_millis(config::BATTERY_PERIOD_MS));
    let mut last_state = BatteryState::Ok;

    loop {
        ticker.next().await;
        let raw = adc.read_mean::<{ config::BATTERY_SAMPLES }>();
        let pin_mv = config::BATTERY_ADC_MODEL.pin_mv(raw);
        let pack_mv = config::BATTERY_DIVIDER.pack_mv(pin_mv);
        let pct = pack_percent(pack_mv, config::BATTERY_CELLS);
        let state = monitor.update(pack_mv);

        with_state(|s| {
            s.battery_mv = u16::try_from(pack_mv).unwrap_or(u16::MAX);
            s.battery_pct = pct;
            s.battery_state = state;
            s.flags.set(
                Flags::LOW_BATTERY,
                matches!(state, BatteryState::Low | BatteryState::Critical),
            );
            s.flags
                .set(Flags::CRITICAL_BATTERY, state == BatteryState::Critical);
        });

        if state != last_state {
            log::warn!("battery: {pack_mv} mV ({pct} %), raw {raw} → {state:?}");
            let inhibit = state == BatteryState::Critical;
            if DRIVE_EVENTS.try_send(DriveEvent::Inhibit(inhibit)).is_err() {
                log::error!("drive event queue full; battery inhibit not delivered");
            }
            last_state = state;
        }
    }
}
