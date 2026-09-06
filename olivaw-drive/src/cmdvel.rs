//! Command-velocity text protocol + safety watchdog.
//!
//! The wire format is one UTF-8 text frame per command:
//!
//! ```text
//! "<left>,<right>"        e.g. "250,-250"  (optional trailing '\n' or '\r\n')
//! ```
//!
//! with speeds as signed per-mille (`-1000..=1000`, matching
//! `drivers/l298n`). It is transport-agnostic by design — the reference
//! firmware (Hands-On-Robotics module 07) carries these frames over a BLE
//! GATT write characteristic; a serial port or UDP socket works identically.
//!
//! [`Watchdog`] ports the firmware's 500 ms safety rule: if no valid frame
//! arrives within the timeout, the commanded speeds decay to zero, so a
//! dropped radio link stops the robot instead of letting it drive away.
//!
//! `no_std`, zero dependencies.

/// Full-scale speed magnitude accepted in a frame.
pub const SPEED_MAX: i16 = 1000;

/// A parsed drive command: per-side speeds in signed per-mille.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DriveCommand {
    /// Left track/wheel speed, `-1000..=1000`.
    pub left: i16,
    /// Right track/wheel speed, `-1000..=1000`.
    pub right: i16,
}

/// Why a frame was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ParseError {
    /// The frame is not valid UTF-8 text.
    NotUtf8,
    /// The frame is not `<int>,<int>`.
    Malformed,
    /// A speed was outside `-1000..=1000`.
    OutOfRange,
}

impl core::fmt::Display for ParseError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NotUtf8 => write!(f, "frame is not UTF-8 text"),
            Self::Malformed => write!(f, "frame is not '<left>,<right>'"),
            Self::OutOfRange => write!(f, "speed outside -1000..=1000"),
        }
    }
}

impl core::error::Error for ParseError {}

/// Parse one frame (`b"<left>,<right>"`, optional trailing newline).
///
/// # Errors
///
/// [`ParseError`] naming what was wrong; a rejected frame should be ignored,
/// keeping the previous command (the watchdog handles a silent link).
pub fn parse_frame(frame: &[u8]) -> Result<DriveCommand, ParseError> {
    let text = core::str::from_utf8(frame).map_err(|_| ParseError::NotUtf8)?;
    let text = text.trim_end_matches(['\r', '\n']).trim();
    let (left, right) = text.split_once(',').ok_or(ParseError::Malformed)?;
    let left: i16 = left.trim().parse().map_err(|_| ParseError::Malformed)?;
    let right: i16 = right.trim().parse().map_err(|_| ParseError::Malformed)?;
    if left.abs() > SPEED_MAX || right.abs() > SPEED_MAX {
        return Err(ParseError::OutOfRange);
    }
    Ok(DriveCommand { left, right })
}

/// Encode a command into `buf` as a frame (no trailing newline); returns the
/// used length. `buf` must be at least [`ENCODE_MAX`] bytes.
///
/// The inverse of [`parse_frame`], for the controller side of the link.
pub const ENCODE_MAX: usize = 12; // "-1000,-1000"

/// Encode `cmd` into `buf`; returns the frame length, or `None` if `buf` is
/// shorter than [`ENCODE_MAX`]. Speeds are clamped to `-1000..=1000`.
#[must_use]
pub fn encode_frame(cmd: DriveCommand, buf: &mut [u8]) -> Option<usize> {
    if buf.len() < ENCODE_MAX {
        return None;
    }
    let mut pos = 0;
    for (i, value) in [cmd.left, cmd.right].into_iter().enumerate() {
        if i == 1 {
            buf[pos] = b',';
            pos += 1;
        }
        let value = value.clamp(-SPEED_MAX, SPEED_MAX);
        if value < 0 {
            buf[pos] = b'-';
            pos += 1;
        }
        let mut magnitude = value.unsigned_abs();
        let mut digits = [0u8; 4];
        let mut n = 0;
        loop {
            digits[n] = b'0' + (magnitude % 10) as u8;
            magnitude /= 10;
            n += 1;
            if magnitude == 0 {
                break;
            }
        }
        while n > 0 {
            n -= 1;
            buf[pos] = digits[n];
            pos += 1;
        }
    }
    Some(pos)
}

/// The safety watchdog: commanded speeds decay to zero when the link goes
/// quiet. Time is any monotonic millisecond counter (rollover handled).
#[derive(Debug, Clone, Copy)]
pub struct Watchdog {
    timeout_ms: u32,
    target: DriveCommand,
    last_feed_ms: Option<u32>,
}

impl Watchdog {
    /// Reference firmware used 500 ms.
    #[must_use]
    pub const fn new(timeout_ms: u32) -> Self {
        Self {
            timeout_ms,
            target: DriveCommand { left: 0, right: 0 },
            last_feed_ms: None,
        }
    }

    /// Record a freshly received command.
    pub fn feed(&mut self, cmd: DriveCommand, now_ms: u32) {
        self.target = cmd;
        self.last_feed_ms = Some(now_ms);
    }

    /// Drop the current command immediately (e.g. on radio disconnect —
    /// the reference firmware zeroes the motors without waiting out the
    /// timeout).
    pub fn trip(&mut self) {
        self.target = DriveCommand::default();
        self.last_feed_ms = None;
    }

    /// The speeds the motors should run **right now**: the last command if
    /// it is fresh, zeros if the link is stale or nothing has arrived yet.
    #[must_use]
    pub fn command(&self, now_ms: u32) -> DriveCommand {
        match self.last_feed_ms {
            Some(fed) if now_ms.wrapping_sub(fed) <= self.timeout_ms => self.target,
            _ => DriveCommand::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_plain_and_padded_frames() {
        assert_eq!(
            parse_frame(b"250,-250"),
            Ok(DriveCommand {
                left: 250,
                right: -250
            })
        );
        assert_eq!(
            parse_frame(b" 1000 , 1000 \n"),
            Ok(DriveCommand {
                left: 1000,
                right: 1000
            })
        );
        assert_eq!(
            parse_frame(b"0,0\r\n"),
            Ok(DriveCommand { left: 0, right: 0 })
        );
    }

    #[test]
    fn rejects_garbage() {
        assert_eq!(parse_frame(b"250"), Err(ParseError::Malformed));
        assert_eq!(parse_frame(b"a,b"), Err(ParseError::Malformed));
        assert_eq!(parse_frame(b"250,-250,7"), Err(ParseError::Malformed));
        assert_eq!(parse_frame(b"1001,0"), Err(ParseError::OutOfRange));
        assert_eq!(parse_frame(b"0,-1001"), Err(ParseError::OutOfRange));
        assert_eq!(parse_frame(&[0xFF, 0xFE]), Err(ParseError::NotUtf8));
    }

    #[test]
    fn encode_round_trips() {
        let mut buf = [0u8; ENCODE_MAX];
        for cmd in [
            DriveCommand { left: 0, right: 0 },
            DriveCommand {
                left: -1000,
                right: 1000,
            },
            DriveCommand {
                left: 42,
                right: -7,
            },
        ] {
            let len = encode_frame(cmd, &mut buf).expect("buffer is large enough");
            assert_eq!(parse_frame(&buf[..len]), Ok(cmd));
        }
    }

    #[test]
    fn encode_needs_a_big_enough_buffer() {
        let mut tiny = [0u8; 4];
        assert_eq!(encode_frame(DriveCommand::default(), &mut tiny), None);
    }

    #[test]
    fn watchdog_zeroes_after_timeout() {
        let mut dog = Watchdog::new(500);
        // Nothing received yet → stopped.
        assert_eq!(dog.command(0), DriveCommand::default());

        let cmd = DriveCommand {
            left: 300,
            right: 300,
        };
        dog.feed(cmd, 1000);
        assert_eq!(dog.command(1400), cmd, "fresh command passes through");
        assert_eq!(dog.command(1501), DriveCommand::default(), "stale → stop");
    }

    #[test]
    fn watchdog_trip_stops_immediately() {
        let mut dog = Watchdog::new(500);
        dog.feed(
            DriveCommand {
                left: 500,
                right: 500,
            },
            0,
        );
        dog.trip();
        assert_eq!(dog.command(1), DriveCommand::default());
    }

    #[test]
    fn watchdog_survives_rollover() {
        let mut dog = Watchdog::new(500);
        let near_wrap = u32::MAX - 100;
        dog.feed(
            DriveCommand {
                left: 100,
                right: 100,
            },
            near_wrap,
        );
        assert_ne!(
            dog.command(near_wrap.wrapping_add(400)),
            DriveCommand::default()
        );
        assert_eq!(
            dog.command(near_wrap.wrapping_add(601)),
            DriveCommand::default()
        );
    }
}
