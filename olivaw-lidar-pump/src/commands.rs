//! The two request frames the car sends: start scanning, stop.

use olivaw_lidar::protocol::{Command, MAX_REQUEST_LEN};

/// Encoded request: bytes + length.
#[derive(Debug, Clone, Copy)]
pub struct Request {
    buf: [u8; MAX_REQUEST_LEN],
    len: usize,
}

impl Request {
    fn of(cmd: Command) -> Self {
        let mut buf = [0u8; MAX_REQUEST_LEN];
        let len = cmd.encode(&mut buf);
        Self { buf, len }
    }

    /// The bytes to write to the lidar.
    #[must_use]
    pub fn bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

/// `SCAN` (0xA5 0x20): starts the motor and the measurement stream.
#[must_use]
pub fn scan_request() -> Request {
    Request::of(Command::Scan)
}

/// `STOP` (0xA5 0x25): stops the stream; motor spins down on the C1.
#[must_use]
pub fn stop_request() -> Request {
    Request::of(Command::Stop)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_the_documented_opcodes() {
        assert_eq!(scan_request().bytes(), &[0xA5, 0x20]);
        assert_eq!(stop_request().bytes(), &[0xA5, 0x25]);
    }
}
