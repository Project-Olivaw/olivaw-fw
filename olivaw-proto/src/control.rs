//! Control opcodes written by the phone to the car's control characteristic.
//!
//! Every command is one opcode byte followed by an optional little-endian
//! payload. Unknown opcodes are rejected, never ignored, so a typo in the app
//! surfaces as an error instead of a silent no-op.

use crate::ProtoError;

/// Longest encoded control command, bytes.
pub const CONTROL_MAX_LEN: usize = 3;

/// A control command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Control {
    /// Brake now and latch: drive commands are ignored until [`Control::Clear`].
    EStop,
    /// Release the E-stop latch.
    Clear,
    /// Start the lidar scan (motor + data stream).
    LidarOn,
    /// Stop the lidar.
    LidarOff,
    /// Bring up Wi-Fi + MQTT.
    UplinkOn,
    /// Tear down Wi-Fi + MQTT (BLE keeps working).
    UplinkOff,
    /// Change the duty cap, per-mille `0..=1000` (clamped by the firmware).
    SetMaxDuty(u16),
}

impl Control {
    const OP_ESTOP: u8 = 0x01;
    const OP_CLEAR: u8 = 0x02;
    const OP_LIDAR_ON: u8 = 0x10;
    const OP_LIDAR_OFF: u8 = 0x11;
    const OP_UPLINK_ON: u8 = 0x20;
    const OP_UPLINK_OFF: u8 = 0x21;
    const OP_SET_MAX_DUTY: u8 = 0x30;

    /// Encode into `out`; returns the number of bytes used.
    #[must_use]
    pub fn encode(self, out: &mut [u8; CONTROL_MAX_LEN]) -> usize {
        match self {
            Self::EStop => {
                out[0] = Self::OP_ESTOP;
                1
            }
            Self::Clear => {
                out[0] = Self::OP_CLEAR;
                1
            }
            Self::LidarOn => {
                out[0] = Self::OP_LIDAR_ON;
                1
            }
            Self::LidarOff => {
                out[0] = Self::OP_LIDAR_OFF;
                1
            }
            Self::UplinkOn => {
                out[0] = Self::OP_UPLINK_ON;
                1
            }
            Self::UplinkOff => {
                out[0] = Self::OP_UPLINK_OFF;
                1
            }
            Self::SetMaxDuty(permille) => {
                out[0] = Self::OP_SET_MAX_DUTY;
                out[1..3].copy_from_slice(&permille.to_le_bytes());
                3
            }
        }
    }

    /// Decode one command from `bytes`.
    ///
    /// # Errors
    ///
    /// [`ProtoError::TooShort`] for an empty or truncated frame,
    /// [`ProtoError::BadOpcode`] for an unknown opcode.
    pub fn decode(bytes: &[u8]) -> Result<Self, ProtoError> {
        let &op = bytes
            .first()
            .ok_or(ProtoError::TooShort { got: 0, needed: 1 })?;
        match op {
            Self::OP_ESTOP => Ok(Self::EStop),
            Self::OP_CLEAR => Ok(Self::Clear),
            Self::OP_LIDAR_ON => Ok(Self::LidarOn),
            Self::OP_LIDAR_OFF => Ok(Self::LidarOff),
            Self::OP_UPLINK_ON => Ok(Self::UplinkOn),
            Self::OP_UPLINK_OFF => Ok(Self::UplinkOff),
            Self::OP_SET_MAX_DUTY => match bytes {
                [_, lo, hi, ..] => Ok(Self::SetMaxDuty(u16::from_le_bytes([*lo, *hi]))),
                _ => Err(ProtoError::TooShort {
                    got: bytes.len(),
                    needed: 3,
                }),
            },
            other => Err(ProtoError::BadOpcode(other)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_every_command() {
        for cmd in [
            Control::EStop,
            Control::Clear,
            Control::LidarOn,
            Control::LidarOff,
            Control::UplinkOn,
            Control::UplinkOff,
            Control::SetMaxDuty(550),
            Control::SetMaxDuty(1000),
        ] {
            let mut buf = [0u8; CONTROL_MAX_LEN];
            let n = cmd.encode(&mut buf);
            assert_eq!(Control::decode(&buf[..n]).unwrap(), cmd);
        }
    }

    #[test]
    fn rejects_unknown_and_truncated() {
        assert_eq!(Control::decode(&[0x7F]), Err(ProtoError::BadOpcode(0x7F)));
        assert!(matches!(
            Control::decode(&[]),
            Err(ProtoError::TooShort { .. })
        ));
        assert!(matches!(
            Control::decode(&[0x30, 1]),
            Err(ProtoError::TooShort { needed: 3, .. })
        ));
    }
}
