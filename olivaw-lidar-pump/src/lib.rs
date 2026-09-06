//! Streaming RPLIDAR pump for microcontrollers.
//!
//! `olivaw-lidar`'s `protocol` layer is pure and fixed-size (bytes in,
//! values out); its std `device` layer owns the blocking read loop. On the
//! ESP32 there is no blocking read — bytes arrive in whatever chunks the
//! UART FIFO produces — so this crate provides the push-style equivalent:
//!
//! ```text
//! UART chunks ──Pump::push_byte──▶ ScanNode ──FrameAssembler::push──▶ ScanFrame (one rotation)
//! ```
//!
//! No allocation, no floats, resynchronises one byte at a time on garbage,
//! and counts what it drops so telemetry can show it.

#![cfg_attr(not(test), no_std)]
// Tests and examples may unwrap freely; library code returns `Result`.
#![cfg_attr(test, allow(clippy::unwrap_used, clippy::expect_used))]

pub mod commands;
pub mod frame;
pub mod pump;

pub use commands::{scan_request, stop_request};
pub use frame::FrameAssembler;
pub use olivaw_lidar::protocol::scan_node::ScanNode;
pub use pump::{Pump, PumpStats};
