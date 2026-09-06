//! One file per task. Each task owns its hardware and talks to the others
//! only through `crate::shared`.

pub mod battery;
pub mod ble;
pub mod drive;
#[cfg(feature = "lidar")]
pub mod lidar;
#[cfg(feature = "uplink")]
pub mod mqtt;
pub mod status_led;
pub mod telemetry;
#[cfg(feature = "uplink")]
pub mod wifi;
