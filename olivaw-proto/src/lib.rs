//! Wire types shared by every Olivaw component.
//!
//! One crate, one definition of every message, used verbatim by:
//!
//! - the ESP32 firmware (`olivaw-car`) — `no_std`, no allocator;
//! - the hub (`olivaw-hub`) — with the `std` feature, decoding what the car sends;
//! - the mobile app and the dashboard — hand-mirrored in TypeScript, checked
//!   against the byte fixtures in this crate's tests.
//!
//! # Links and encodings
//!
//! | Link | Message | Encoding | Module |
//! | --- | --- | --- | --- |
//! | BLE, phone → car | drive command | ASCII `"<left>,<right>"` per-mille (see `olivaw-drive`) | — |
//! | BLE, phone → car | control opcode | 1–3 bytes | [`control`] |
//! | BLE, car → phone | telemetry | packed little-endian, 19 bytes | [`telemetry`] |
//! | MQTT, car → hub | telemetry | JSON of [`telemetry::Telemetry`] | [`telemetry`] |
//! | MQTT, car → hub | lidar rotation | postcard of [`scan_frame::ScanFrame`] | [`scan_frame`] |
//! | MQTT, car ↔ hub | status / commands | UTF-8 strings | [`topics`] |
//!
//! Units are fixed at the wire: millivolts, per-mille duty, seconds, dBm, and
//! the lidar's native Q6 degrees / Q2 millimetres. Conversions to metres and
//! radians happen once, on the hub, never here.

#![cfg_attr(not(feature = "std"), no_std)]
// Tests and examples may unwrap freely; library code returns `Result`.
#![cfg_attr(test, allow(clippy::unwrap_used, clippy::expect_used))]

pub mod control;
pub mod scan_frame;
pub mod status;
pub mod telemetry;
pub mod topics;

pub use control::Control;
pub use scan_frame::{MAX_NODES, ScanFrame, ScanPoint};
pub use status::LinkState;
pub use telemetry::{Flags, TELEMETRY_LEN, TELEMETRY_VERSION, Telemetry};

/// Why a frame could not be decoded.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum ProtoError {
    /// Fewer bytes than the frame needs. `needed` is the minimum length.
    #[error("frame too short: got {got} bytes, need {needed}")]
    TooShort {
        /// Bytes received.
        got: usize,
        /// Bytes required.
        needed: usize,
    },
    /// The version byte is not one this crate understands.
    #[error("unsupported frame version {0}")]
    BadVersion(u8),
    /// The opcode byte is not a known control command.
    #[error("unknown control opcode 0x{0:02X}")]
    BadOpcode(u8),
    /// A postcard (de)serialisation failed.
    #[error("postcard: {0}")]
    Postcard(#[from] postcard::Error),
    /// The output buffer is too small for the encoded frame.
    #[error("output buffer too small")]
    BufferTooSmall,
}
