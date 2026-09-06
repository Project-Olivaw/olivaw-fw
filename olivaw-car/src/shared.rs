//! State and channels shared between tasks. One place, so the data flow is
//! readable: who signals what, who consumes it.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use olivaw_battery::BatteryState;
use olivaw_drive::DriveCommand;
use olivaw_proto::{Flags, LinkState, Telemetry};

/// The mutex flavour used for everything shared.
pub type Raw = CriticalSectionRawMutex;

/// Latest drive frame from any link (BLE today, MQTT `cmd` later).
pub static DRIVE_CMD: Signal<Raw, DriveCommand> = Signal::new();
/// Events for the drive task.
pub static DRIVE_EVENTS: Channel<Raw, DriveEvent, 8> = Channel::new();
/// Lidar on/off requests.
pub static LIDAR_CTL: Signal<Raw, bool> = Signal::new();
/// Uplink publish on/off requests.
pub static UPLINK_CTL: Signal<Raw, bool> = Signal::new();
/// Telemetry snapshots at `config::TELEMETRY_PERIOD_MS`. Receivers: BLE notify, MQTT.
pub static TELEMETRY: Watch<Raw, Telemetry, 2> = Watch::new();
/// Whole lidar rotations, drop-oldest.
#[cfg(feature = "lidar")]
pub static SCANS: Channel<Raw, olivaw_proto::ScanFrame, 2> = Channel::new();
/// Mutable car state. Lock briefly; never `await` while holding it.
pub static STATE: Mutex<Raw, RefCell<CarState>> = Mutex::new(RefCell::new(CarState::new()));

/// Things that change how the drive task behaves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveEvent {
    /// Brake and latch.
    EStop,
    /// Release the latch.
    Clear,
    /// New duty cap, per-mille.
    SetMaxDuty(u16),
    /// The control link dropped: forget the last command now.
    LinkLost,
    /// Inhibit driving (critical battery) or release the inhibit.
    Inhibit(bool),
}

/// What every task reports into; the telemetry task snapshots it.
#[derive(Debug, Clone, Copy)]
pub struct CarState {
    /// Pack voltage, millivolts.
    pub battery_mv: u16,
    /// State of charge, percent.
    pub battery_pct: u8,
    /// Battery health verdict.
    pub battery_state: BatteryState,
    /// Commanded left/right duty after the governor, per-mille.
    pub left: i16,
    /// See `left`.
    pub right: i16,
    /// Duty cap in force, per-mille.
    pub max_duty: u16,
    /// Status bits.
    pub flags: Flags,
    /// Last BLE RSSI, dBm (0 = unknown).
    pub rssi_dbm: i8,
    /// Lidar link state.
    pub lidar: LinkState,
    /// Wi-Fi link state.
    pub wifi: LinkState,
    /// MQTT link state.
    pub mqtt: LinkState,
    /// Lidar UART FIFO overruns since boot.
    #[cfg_attr(not(feature = "lidar"), allow(dead_code))]
    pub uart_overruns: u32,
}

impl CarState {
    /// Boot state: everything off, unknown battery.
    pub const fn new() -> Self {
        Self {
            battery_mv: 0,
            battery_pct: 0,
            battery_state: BatteryState::Ok,
            left: 0,
            right: 0,
            max_duty: crate::config::DRIVE.max_duty_permille,
            flags: Flags(0),
            rssi_dbm: 0,
            lidar: LinkState::Off,
            wifi: LinkState::Off,
            mqtt: LinkState::Off,
            uart_overruns: 0,
        }
    }

    /// Build the wire telemetry frame.
    pub fn snapshot(&self, uptime_s: u32) -> Telemetry {
        Telemetry {
            flags: self.flags,
            battery_mv: self.battery_mv,
            battery_pct: self.battery_pct,
            left: self.left,
            right: self.right,
            uptime_s,
            rssi_dbm: self.rssi_dbm,
            lidar: self.lidar,
            wifi: self.wifi,
            mqtt: self.mqtt,
            max_duty_permille: self.max_duty,
        }
    }
}

/// Run `f` with the state locked.
pub fn with_state<R>(f: impl FnOnce(&mut CarState) -> R) -> R {
    STATE.lock(|cell| f(&mut cell.borrow_mut()))
}

/// Milliseconds since boot, wrapping (what the drive watchdog expects).
pub fn now_ms() -> u32 {
    // Truncation is the point: a wrapping u32 millisecond clock.
    #[allow(clippy::cast_possible_truncation)]
    {
        embassy_time::Instant::now().as_millis() as u32
    }
}

/// Seconds since boot, saturating.
pub fn uptime_s() -> u32 {
    u32::try_from(embassy_time::Instant::now().as_secs()).unwrap_or(u32::MAX)
}
