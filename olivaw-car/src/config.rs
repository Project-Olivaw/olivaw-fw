//! Every tunable number in the firmware, named and documented.
//!
//! Change behaviour here, not inside the tasks.

use olivaw_battery::{AdcModel, Divider, MonitorConfig};
use olivaw_drive::DriveConfig;

/// Identifier used in MQTT topics (`olivaw/<CAR_ID>/…`) and as MQTT client id.
#[cfg_attr(not(feature = "uplink"), allow(dead_code))]
pub const CAR_ID: &str = "car-01";

/// BLE advertised name. The app filters on this.
pub const BLE_NAME: &str = "OLIVAW-CAR";
/// Static random BLE address (top two bits set). Change per car.
pub const BLE_ADDRESS: [u8; 6] = [0xC2, 0x01, 0x1A, 0xC0, 0xFF, 0xEE];

/// Drive pipeline: 500 ms watchdog, 55 % duty cap for 6 V TT motors on a
/// 3S pack through an L298N, ramp of 60 ‰ per tick (= 0 → full in ~0.35 s).
pub const DRIVE: DriveConfig = DriveConfig {
    watchdog_ms: 500,
    max_duty_permille: 550,
    slew_step_permille: 60,
};
/// Drive loop period, milliseconds (50 Hz).
pub const DRIVE_TICK_MS: u64 = 20;

/// Battery sampling period, milliseconds (2 Hz).
pub const BATTERY_PERIOD_MS: u64 = 500;
/// ADC samples averaged per reading.
pub const BATTERY_SAMPLES: usize = 8;
/// Series cells in the pack.
pub const BATTERY_CELLS: u8 = 3;
/// The divider on the board.
pub const BATTERY_DIVIDER: Divider = Divider::CAR_3S;
/// Raw-code → millivolt model. Calibrate with a multimeter and replace.
pub const BATTERY_ADC_MODEL: AdcModel = AdcModel::ESP32_11DB_NOMINAL;
/// Low / critical thresholds and hysteresis.
pub const BATTERY_MONITOR: MonitorConfig = MonitorConfig::CAR_3S;

/// Telemetry snapshot period, milliseconds (5 Hz → BLE notify rate).
pub const TELEMETRY_PERIOD_MS: u64 = 200;
/// Battery Level (0x2A19) notify period, in telemetry ticks (every 5 s).
pub const BATTERY_LEVEL_EVERY_TICKS: u32 = 25;
/// RSSI read period, in telemetry ticks (every 1 s).
pub const RSSI_EVERY_TICKS: u32 = 5;

/// MQTT telemetry publish period, in telemetry ticks (1 Hz).
#[cfg_attr(not(feature = "uplink"), allow(dead_code))]
pub const MQTT_TELEMETRY_EVERY_TICKS: u32 = 5;
/// MQTT keep-alive, seconds.
#[cfg_attr(not(feature = "uplink"), allow(dead_code))]
pub const MQTT_KEEP_ALIVE_S: u16 = 30;
/// Reconnect back-off after a network failure, seconds.
#[cfg_attr(not(feature = "uplink"), allow(dead_code))]
pub const NET_RETRY_S: u64 = 5;
