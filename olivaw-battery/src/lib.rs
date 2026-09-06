//! Battery sensing maths, `no_std`, zero dependencies, integer-only.
//!
//! The pipeline the firmware's battery task runs at 2 Hz:
//!
//! ```text
//! ADC raw ──AdcModel──▶ pin mV ──Divider──▶ pack mV ──lipo_percent──▶ %
//!                                              └──BatteryMonitor──▶ Ok / Low / Critical
//! ```
//!
//! Every constant is a documented config field so a future 2S rover or a
//! 4S drone reuses this crate by changing numbers, not code.

#![cfg_attr(not(test), no_std)]
// Tests and examples may unwrap freely; library code returns `Result`.
#![cfg_attr(test, allow(clippy::unwrap_used, clippy::expect_used))]

pub mod curve;
pub mod divider;
pub mod model;
pub mod monitor;

pub use curve::{cell_percent, pack_percent};
pub use divider::Divider;
pub use model::{AdcModel, mean_u16};
pub use monitor::{BatteryMonitor, BatteryState, MonitorConfig};
