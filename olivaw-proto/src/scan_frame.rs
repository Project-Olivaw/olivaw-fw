//! One full lidar rotation, as published by the car over MQTT.
//!
//! Points keep the RPLIDAR's native fixed-point units (angle in 1/64 degree,
//! distance in 1/4 millimetre) so the car never touches floating point and
//! the hub converts exactly once. Encoded with postcard: compact varints,
//! ~3 KB for a typical 500-point C1 rotation.

use heapless::Vec;
use serde::{Deserialize, Serialize};

use crate::ProtoError;

/// Maximum points kept per rotation. The C1 yields ~500 at 10 Hz; slower
/// motor speeds yield more. Anything beyond is counted in
/// [`ScanFrame::dropped`], not silently lost.
pub const MAX_NODES: usize = 720;

/// One measurement, in lidar-native fixed point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct ScanPoint {
    /// Angle, Q6: degrees × 64, clockwise from the lidar's forward mark.
    pub angle_q6: u16,
    /// Distance, Q2: millimetres × 4. `0` = no return.
    pub dist_q2: u16,
    /// Return quality `0..=63`.
    pub quality: u8,
}

impl ScanPoint {
    /// Angle in degrees.
    #[must_use]
    pub fn angle_deg(&self) -> f32 {
        f32::from(self.angle_q6) / 64.0
    }

    /// Distance in millimetres. `0.0` = no return.
    #[must_use]
    pub fn distance_mm(&self) -> f32 {
        f32::from(self.dist_q2) / 4.0
    }

    /// `true` when the point carries a real measurement.
    #[must_use]
    pub const fn is_valid(&self) -> bool {
        self.dist_q2 != 0
    }
}

/// A complete rotation.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct ScanFrame {
    /// Rotation counter since the lidar was started; wraps.
    pub seq: u32,
    /// Car uptime when the rotation completed, milliseconds; wraps.
    pub t_ms: u32,
    /// Points that did not fit in [`MAX_NODES`].
    pub dropped: u16,
    /// The measurements, in the order the lidar produced them.
    pub points: Vec<ScanPoint, MAX_NODES>,
}

impl ScanFrame {
    /// Upper bound of the postcard encoding, bytes (varint worst case).
    pub const MAX_ENCODED_LEN: usize = 5 + 5 + 3 + 2 + MAX_NODES * 7;

    /// Serialise with postcard into `out`; returns the used slice.
    ///
    /// # Errors
    ///
    /// [`ProtoError::BufferTooSmall`] if `out` cannot hold the frame.
    pub fn encode<'a>(&self, out: &'a mut [u8]) -> Result<&'a [u8], ProtoError> {
        let used = postcard::to_slice(self, out).map_err(|e| match e {
            postcard::Error::SerializeBufferFull => ProtoError::BufferTooSmall,
            other => ProtoError::Postcard(other),
        })?;
        Ok(&*used)
    }

    /// Deserialise a postcard-encoded frame.
    ///
    /// # Errors
    ///
    /// [`ProtoError::Postcard`] for malformed input.
    pub fn decode(bytes: &[u8]) -> Result<Self, ProtoError> {
        Ok(postcard::from_bytes(bytes)?)
    }

    /// Number of valid (non-zero distance) points.
    #[must_use]
    pub fn valid_points(&self) -> usize {
        self.points.iter().filter(|p| p.is_valid()).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(n: usize) -> ScanFrame {
        let mut f = ScanFrame {
            seq: 7,
            t_ms: 12_345,
            dropped: 0,
            points: Vec::new(),
        };
        for i in 0..n {
            #[allow(clippy::cast_possible_truncation)]
            let i16 = i as u16;
            f.points
                .push(ScanPoint {
                    angle_q6: i16 * 46,
                    dist_q2: 4000 + i16 * 3,
                    quality: 47,
                })
                .ok();
        }
        f
    }

    #[test]
    fn round_trips_a_typical_rotation() {
        let f = frame(500);
        let mut buf = [0u8; ScanFrame::MAX_ENCODED_LEN];
        let bytes = f.encode(&mut buf).unwrap();
        assert!(bytes.len() < 3600, "encoded {} bytes", bytes.len());
        assert_eq!(ScanFrame::decode(bytes).unwrap(), f);
    }

    #[test]
    fn full_frame_fits_the_bound() {
        let f = frame(MAX_NODES);
        let mut buf = [0u8; ScanFrame::MAX_ENCODED_LEN];
        assert!(f.encode(&mut buf).is_ok());
    }

    #[test]
    fn small_buffer_is_reported() {
        let mut tiny = [0u8; 8];
        assert_eq!(
            frame(10).encode(&mut tiny).unwrap_err(),
            ProtoError::BufferTooSmall
        );
    }

    #[test]
    fn point_unit_helpers() {
        let p = ScanPoint {
            angle_q6: 90 * 64,
            dist_q2: 1234 * 4,
            quality: 10,
        };
        assert!((p.angle_deg() - 90.0).abs() < 1e-6);
        assert!((p.distance_mm() - 1234.0).abs() < 1e-6);
        assert!(p.is_valid());
        assert!(!ScanPoint::default().is_valid());
    }
}
