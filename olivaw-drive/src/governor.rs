//! Duty cap. On a 3S pack (12.6 V full) through an L298N (~2 V drop) the
//! yellow TT motors, rated 3–6 V, would see 10.6 V at full duty — the cap
//! keeps the *average* motor voltage sane without changing how the joystick
//! feels, because both sides scale by the same factor and the turn ratio
//! survives.

use crate::cmdvel::{DriveCommand, SPEED_MAX};

/// Scales commands so that `|speed| <= max_permille`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Governor {
    max_permille: u16,
}

impl Governor {
    /// A cap of `max_permille` (`0..=1000`, clamped).
    #[must_use]
    pub const fn new(max_permille: u16) -> Self {
        let max = if max_permille > SPEED_MAX as u16 {
            SPEED_MAX as u16
        } else {
            max_permille
        };
        Self { max_permille: max }
    }

    /// The cap in force.
    #[must_use]
    pub const fn max_permille(&self) -> u16 {
        self.max_permille
    }

    /// Replace the cap (clamped to `0..=1000`).
    pub fn set_max_permille(&mut self, max_permille: u16) {
        *self = Self::new(max_permille);
    }

    /// Scale both sides by `max/1000`. Ratio-preserving, so a command that
    /// was already inside the cap still shrinks — the cap is a *gain*, which
    /// is what a speed limiter should feel like on a joystick.
    #[must_use]
    pub fn apply(&self, cmd: DriveCommand) -> DriveCommand {
        let scale = |v: i16| -> i16 {
            let scaled = i32::from(v.clamp(-SPEED_MAX, SPEED_MAX)) * i32::from(self.max_permille)
                / i32::from(SPEED_MAX);
            i16::try_from(scaled).unwrap_or(if v < 0 { -SPEED_MAX } else { SPEED_MAX })
        };
        DriveCommand {
            left: scale(cmd.left),
            right: scale(cmd.right),
        }
    }
}

impl Default for Governor {
    /// Full duty allowed — firmware config picks the real default.
    fn default() -> Self {
        Self::new(SPEED_MAX as u16)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scales_both_sides_by_the_cap() {
        let g = Governor::new(550);
        let out = g.apply(DriveCommand {
            left: 1000,
            right: -500,
        });
        assert_eq!(
            out,
            DriveCommand {
                left: 550,
                right: -275
            }
        );
    }

    #[test]
    fn cap_is_clamped_and_zero_stops() {
        assert_eq!(Governor::new(5000).max_permille(), 1000);
        let out = Governor::new(0).apply(DriveCommand {
            left: 900,
            right: 900,
        });
        assert_eq!(out, DriveCommand::default());
    }

    #[test]
    fn out_of_range_inputs_are_clamped_first() {
        let out = Governor::new(1000).apply(DriveCommand {
            left: i16::MAX,
            right: i16::MIN,
        });
        assert_eq!(
            out,
            DriveCommand {
                left: 1000,
                right: -1000
            }
        );
    }
}
