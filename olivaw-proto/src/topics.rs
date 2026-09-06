//! MQTT topic layout: `olivaw/<car_id>/<channel>`.
//!
//! | Topic | Direction | Payload |
//! | --- | --- | --- |
//! | `olivaw/<car>/status` | car → hub, retained + LWT | `"online"` / `"offline"` |
//! | `olivaw/<car>/telemetry` | car → hub, 1 Hz | JSON [`crate::Telemetry`] |
//! | `olivaw/<car>/scan` | car → hub, per rotation | postcard [`crate::ScanFrame`] |
//! | `olivaw/<car>/cmd` | hub → car | [`crate::Control`] bytes |

use core::fmt::Write as _;

/// Topic prefix shared by every car.
pub const PREFIX: &str = "olivaw";
/// Status payload published on connect (retained).
pub const STATUS_ONLINE: &str = "online";
/// Status payload set as the last-will message.
pub const STATUS_OFFLINE: &str = "offline";
/// Longest topic this crate will build, bytes.
pub const TOPIC_MAX_LEN: usize = 48;

/// The channels under a car's prefix.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    /// Online / offline, retained.
    Status,
    /// JSON telemetry.
    Telemetry,
    /// Binary scan frames.
    Scan,
    /// Commands from the hub.
    Cmd,
}

impl Channel {
    /// The last path segment.
    #[must_use]
    pub const fn segment(self) -> &'static str {
        match self {
            Self::Status => "status",
            Self::Telemetry => "telemetry",
            Self::Scan => "scan",
            Self::Cmd => "cmd",
        }
    }

    /// Parse the last path segment.
    #[must_use]
    pub fn from_segment(s: &str) -> Option<Self> {
        match s {
            "status" => Some(Self::Status),
            "telemetry" => Some(Self::Telemetry),
            "scan" => Some(Self::Scan),
            "cmd" => Some(Self::Cmd),
            _ => None,
        }
    }
}

/// Build `olivaw/<car_id>/<channel>` without allocating.
///
/// Returns `None` if the result would not fit [`TOPIC_MAX_LEN`] (car ids are
/// short slugs like `car-01`).
#[must_use]
pub fn topic(car_id: &str, channel: Channel) -> Option<heapless::String<TOPIC_MAX_LEN>> {
    let mut s = heapless::String::new();
    write!(s, "{PREFIX}/{car_id}/{}", channel.segment()).ok()?;
    Some(s)
}

/// The wildcard subscription that matches every car's channels.
pub const ALL_CARS: &str = "olivaw/+/+";

/// Split `olivaw/<car_id>/<channel>` back into its parts.
#[must_use]
pub fn parse(topic: &str) -> Option<(&str, Channel)> {
    let rest = topic.strip_prefix(PREFIX)?.strip_prefix('/')?;
    let (car_id, segment) = rest.split_once('/')?;
    if car_id.is_empty() || segment.contains('/') {
        return None;
    }
    Some((car_id, Channel::from_segment(segment)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builds_and_parses() {
        let t = topic("car-01", Channel::Scan).unwrap();
        assert_eq!(t.as_str(), "olivaw/car-01/scan");
        assert_eq!(parse(&t), Some(("car-01", Channel::Scan)));
    }

    #[test]
    fn rejects_foreign_topics() {
        assert_eq!(parse("other/car-01/scan"), None);
        assert_eq!(parse("olivaw//scan"), None);
        assert_eq!(parse("olivaw/car-01/nope"), None);
        assert_eq!(parse("olivaw/car-01/scan/extra"), None);
    }

    #[test]
    fn overlong_car_id_is_refused() {
        let long = "x".repeat(60);
        assert!(topic(&long, Channel::Cmd).is_none());
    }
}
