//! The car's telemetry frame: what the phone sees over BLE notify and what
//! the hub receives as JSON over MQTT.
//!
//! The BLE encoding is a fixed 19-byte little-endian layout so it fits the
//! 20-byte payload of the default 23-byte ATT MTU (no MTU negotiation
//! required). Layout, byte offsets:
//!
//! ```text
//!  0  version            u8   = 1
//!  1  flags              u8   bitfield, see [`Flags`]
//!  2  battery_mv         u16  pack voltage, millivolts
//!  4  battery_pct        u8   0..=100
//!  5  left               i16  commanded left duty, per-mille
//!  7  right              i16  commanded right duty, per-mille
//!  9  uptime_s           u32  seconds since boot
//! 13  rssi_dbm           i8   BLE RSSI as seen by the car (0 = unknown)
//! 14  lidar              u8   LinkState
//! 15  wifi               u8   LinkState
//! 16  mqtt               u8   LinkState
//! 17  max_duty_permille  u16  the duty cap currently applied
//! ```

use serde::{Deserialize, Serialize};

use crate::ProtoError;
use crate::status::LinkState;

/// Current telemetry frame version.
pub const TELEMETRY_VERSION: u8 = 1;
/// Encoded length of a v1 telemetry frame, bytes.
pub const TELEMETRY_LEN: usize = 19;

/// Telemetry flag bits (byte 1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct Flags(pub u8);

impl Flags {
    /// Emergency stop is latched; motors are braked until `Clear`.
    pub const E_STOP: Flags = Flags(1 << 0);
    /// Pack voltage below the warning threshold.
    pub const LOW_BATTERY: Flags = Flags(1 << 1);
    /// Pack voltage below the cut-off threshold; driving is inhibited.
    pub const CRITICAL_BATTERY: Flags = Flags(1 << 2);
    /// A BLE central is connected.
    pub const BLE_CONNECTED: Flags = Flags(1 << 3);
    /// The drive watchdog has expired (no fresh command).
    pub const WATCHDOG_STALE: Flags = Flags(1 << 4);
    /// The lidar UART reported at least one FIFO overrun since boot.
    pub const UART_OVERRUN: Flags = Flags(1 << 5);

    /// `true` if every bit of `other` is set.
    #[must_use]
    pub const fn contains(self, other: Flags) -> bool {
        self.0 & other.0 == other.0
    }

    /// Set the bits of `other`.
    pub fn insert(&mut self, other: Flags) {
        self.0 |= other.0;
    }

    /// Clear the bits of `other`.
    pub fn remove(&mut self, other: Flags) {
        self.0 &= !other.0;
    }

    /// Set or clear the bits of `other`.
    pub fn set(&mut self, other: Flags, on: bool) {
        if on {
            self.insert(other);
        } else {
            self.remove(other);
        }
    }
}

/// One telemetry sample. Field units are in the field names or docs; nothing
/// here is ever converted to floats on the car.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct Telemetry {
    /// Status bits.
    pub flags: Flags,
    /// Battery pack voltage, millivolts.
    pub battery_mv: u16,
    /// Battery state of charge, percent `0..=100`.
    pub battery_pct: u8,
    /// Commanded left duty after the governor, per-mille `-1000..=1000`.
    pub left: i16,
    /// Commanded right duty after the governor, per-mille `-1000..=1000`.
    pub right: i16,
    /// Seconds since boot.
    pub uptime_s: u32,
    /// BLE RSSI measured by the car, dBm. `0` means unknown.
    pub rssi_dbm: i8,
    /// Lidar link state.
    pub lidar: LinkState,
    /// Wi-Fi link state.
    pub wifi: LinkState,
    /// MQTT link state.
    pub mqtt: LinkState,
    /// Duty cap in force, per-mille `0..=1000`.
    pub max_duty_permille: u16,
}

impl Telemetry {
    /// Encode as the fixed BLE layout.
    #[must_use]
    pub fn encode(&self) -> [u8; TELEMETRY_LEN] {
        let mut b = [0u8; TELEMETRY_LEN];
        b[0] = TELEMETRY_VERSION;
        b[1] = self.flags.0;
        b[2..4].copy_from_slice(&self.battery_mv.to_le_bytes());
        b[4] = self.battery_pct;
        b[5..7].copy_from_slice(&self.left.to_le_bytes());
        b[7..9].copy_from_slice(&self.right.to_le_bytes());
        b[9..13].copy_from_slice(&self.uptime_s.to_le_bytes());
        b[13] = self.rssi_dbm.to_le_bytes()[0];
        b[14] = self.lidar.as_u8();
        b[15] = self.wifi.as_u8();
        b[16] = self.mqtt.as_u8();
        b[17..19].copy_from_slice(&self.max_duty_permille.to_le_bytes());
        b
    }

    /// Decode the fixed BLE layout. Extra trailing bytes are ignored so a
    /// future longer frame still decodes its v1 prefix.
    ///
    /// # Errors
    ///
    /// [`ProtoError::TooShort`] or [`ProtoError::BadVersion`].
    pub fn decode(bytes: &[u8]) -> Result<Self, ProtoError> {
        if bytes.len() < TELEMETRY_LEN {
            return Err(ProtoError::TooShort {
                got: bytes.len(),
                needed: TELEMETRY_LEN,
            });
        }
        if bytes[0] != TELEMETRY_VERSION {
            return Err(ProtoError::BadVersion(bytes[0]));
        }
        let u16_at = |i: usize| u16::from_le_bytes([bytes[i], bytes[i + 1]]);
        let i16_at = |i: usize| i16::from_le_bytes([bytes[i], bytes[i + 1]]);
        Ok(Self {
            flags: Flags(bytes[1]),
            battery_mv: u16_at(2),
            battery_pct: bytes[4],
            left: i16_at(5),
            right: i16_at(7),
            uptime_s: u32::from_le_bytes([bytes[9], bytes[10], bytes[11], bytes[12]]),
            rssi_dbm: i8::from_le_bytes([bytes[13]]),
            lidar: LinkState::from_u8(bytes[14]),
            wifi: LinkState::from_u8(bytes[15]),
            mqtt: LinkState::from_u8(bytes[16]),
            max_duty_permille: u16_at(17),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Telemetry {
        let mut flags = Flags::default();
        flags.insert(Flags::BLE_CONNECTED);
        flags.insert(Flags::LOW_BATTERY);
        Telemetry {
            flags,
            battery_mv: 11_432,
            battery_pct: 63,
            left: -250,
            right: 1000,
            uptime_s: 123_456,
            rssi_dbm: -67,
            lidar: LinkState::Up,
            wifi: LinkState::Starting,
            mqtt: LinkState::Off,
            max_duty_permille: 550,
        }
    }

    #[test]
    fn round_trips() {
        let t = sample();
        let bytes = t.encode();
        assert_eq!(bytes.len(), TELEMETRY_LEN);
        assert_eq!(Telemetry::decode(&bytes).unwrap(), t);
    }

    #[test]
    fn layout_is_stable() {
        // This is the byte fixture mirrored in the TypeScript decoders.
        let bytes = sample().encode();
        assert_eq!(
            bytes,
            [
                1,
                0b0000_1010,
                0xA8,
                0x2C,
                63,
                0x06,
                0xFF,
                0xE8,
                0x03,
                0x40,
                0xE2,
                0x01,
                0x00,
                0xBD,
                2,
                1,
                0,
                0x26,
                0x02
            ]
        );
    }

    #[test]
    fn rejects_short_and_wrong_version() {
        assert!(matches!(
            Telemetry::decode(&[1, 2, 3]),
            Err(ProtoError::TooShort {
                got: 3,
                needed: TELEMETRY_LEN
            })
        ));
        let mut bytes = sample().encode();
        bytes[0] = 9;
        assert_eq!(Telemetry::decode(&bytes), Err(ProtoError::BadVersion(9)));
    }

    #[test]
    fn tolerates_trailing_bytes() {
        let mut long = [0u8; 32];
        long[..TELEMETRY_LEN].copy_from_slice(&sample().encode());
        assert_eq!(Telemetry::decode(&long).unwrap(), sample());
    }

    #[test]
    fn flags_set_and_clear() {
        let mut f = Flags::default();
        f.set(Flags::E_STOP, true);
        assert!(f.contains(Flags::E_STOP));
        f.set(Flags::E_STOP, false);
        assert!(!f.contains(Flags::E_STOP));
    }
}
