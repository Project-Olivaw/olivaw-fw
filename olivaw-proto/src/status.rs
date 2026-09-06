//! Small enums describing the state of the car's links, shared by telemetry
//! and the dashboard.

use serde::{Deserialize, Serialize};

/// State of an optional subsystem link (lidar, Wi-Fi, MQTT).
///
/// Encoded as one byte on the wire; unknown values decode to
/// [`LinkState::Fault`] so a newer car never crashes an older reader.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[repr(u8)]
pub enum LinkState {
    /// Disabled or never started.
    #[default]
    Off = 0,
    /// Starting up / connecting.
    Starting = 1,
    /// Running normally.
    Up = 2,
    /// Failed; will retry or needs a reset.
    Fault = 3,
}

impl LinkState {
    /// Decode from the wire byte.
    #[must_use]
    pub const fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Off,
            1 => Self::Starting,
            2 => Self::Up,
            _ => Self::Fault,
        }
    }

    /// The wire byte.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// `true` while the link is usable.
    #[must_use]
    pub const fn is_up(self) -> bool {
        matches!(self, Self::Up)
    }
}
