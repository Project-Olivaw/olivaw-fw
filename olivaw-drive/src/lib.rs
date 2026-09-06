//! Differential-drive control for the Olivaw car.
//!
//! Pure `no_std` logic, generic over `embedded-hal 1.0`, fully testable on
//! the host. The firmware's drive task is a thin loop around
//! [`DriveController`]:
//!
//! ```text
//! BLE frame ──parse_frame──▶ Watchdog ──▶ E-stop latch ──▶ Governor ──▶ Slew ──▶ Motors
//!                             (500 ms)      (brake)        (duty cap)    (ramp)   (L298N)
//! ```
//!
//! - [`cmdvel`] and [`kinematics`] are vendored from the `olivaw-cli`
//!   registry (`comms/cmdvel-protocol`, `kinematics/differential-drive`)
//!   and [`l298n`] from `drivers/l298n`, unchanged apart from the module
//!   doc — the registry is the source of truth for fixes.
//! - [`controller`] composes them with the [`governor`] and [`slew`] stages
//!   that a 3S pack on 6 V motors makes mandatory.
//!
//! Speeds everywhere are signed per-mille of full duty, `-1000..=1000`.

#![cfg_attr(not(test), no_std)]
// Tests and examples may unwrap freely; library code returns `Result`.
#![cfg_attr(test, allow(clippy::unwrap_used, clippy::expect_used))]

pub mod cmdvel;
pub mod controller;
pub mod governor;
pub mod kinematics;
pub mod l298n;
pub mod motors;
pub mod slew;

pub use cmdvel::{DriveCommand, ParseError, SPEED_MAX, Watchdog, encode_frame, parse_frame};
pub use controller::{DriveConfig, DriveController, DriveOutput};
pub use governor::Governor;
pub use kinematics::{DifferentialDrive, MixOutput, WheelSpeeds};
pub use l298n::{L298n, Motor};
pub use motors::Motors;
pub use slew::Slew;
