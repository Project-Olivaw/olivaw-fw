//! Board support for the Olivaw car: an ESP32-WROOM-32D `DevKit V1` driving
//! an L298N, sensing a 3S pack and (optionally) talking to an RPLIDAR C1.
//!
//! This is the "HAL on top of the HAL": it knows the pin map and turns
//! `esp-hal` peripherals into the typed parts the firmware tasks need
//! ([`CarMotors`], [`BatteryAdc`], the status LED, the lidar UART) while
//! handing the radio and timer peripherals back untouched for the binary
//! to wire into `esp-rtos` / `esp-radio`.
//!
//! Nothing here contains policy: no watchdog, no duty caps, no rates. Those
//! live in `olivaw-drive`, `olivaw-battery` and the firmware's `config.rs`.

#![no_std]

pub mod battery;
pub mod board;
pub mod led;
#[cfg(feature = "lidar")]
pub mod lidar_uart;
pub mod motors;
pub mod pins;

pub use battery::BatteryAdc;
pub use board::{Board, BoardError, Radio};
pub use motors::{CarMotors, PwmChannel};
