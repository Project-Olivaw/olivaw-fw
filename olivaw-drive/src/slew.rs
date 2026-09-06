//! Slew-rate limiter. Motors that go from 0 to full duty in one tick pull a
//! stall-current spike that sags the 5 V rail and brown-outs the ESP32;
//! ramping over a few ticks costs nothing in feel and removes the resets.

use crate::cmdvel::DriveCommand;

/// Limits how much each side may change per tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Slew {
    max_step: i16,
    current: DriveCommand,
}

impl Slew {
    /// Allow at most `max_step_permille` of change per [`step`](Self::step).
    /// `0` disables ramping (every step jumps straight to the target).
    #[must_use]
    pub const fn new(max_step_permille: i16) -> Self {
        let max_step = if max_step_permille == i16::MIN {
            i16::MAX
        } else {
            max_step_permille.abs()
        };
        Self {
            max_step,
            current: DriveCommand { left: 0, right: 0 },
        }
    }

    /// Move one tick towards `target`; returns the value to apply now.
    pub fn step(&mut self, target: DriveCommand) -> DriveCommand {
        if self.max_step == 0 {
            self.current = target;
            return target;
        }
        let approach = |now: i16, want: i16| -> i16 {
            let delta = i32::from(want) - i32::from(now);
            let clamped = delta.clamp(-i32::from(self.max_step), i32::from(self.max_step));
            i16::try_from(i32::from(now) + clamped).unwrap_or(want)
        };
        self.current = DriveCommand {
            left: approach(self.current.left, target.left),
            right: approach(self.current.right, target.right),
        };
        self.current
    }

    /// Jump to `value` immediately (used after a brake so the ramp does not
    /// resume from a stale speed).
    pub fn reset_to(&mut self, value: DriveCommand) {
        self.current = value;
    }

    /// The value last applied.
    #[must_use]
    pub const fn current(&self) -> DriveCommand {
        self.current
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ramps_up_and_down_by_at_most_the_step() {
        let mut s = Slew::new(100);
        let target = DriveCommand {
            left: 350,
            right: -350,
        };
        assert_eq!(
            s.step(target),
            DriveCommand {
                left: 100,
                right: -100
            }
        );
        assert_eq!(
            s.step(target),
            DriveCommand {
                left: 200,
                right: -200
            }
        );
        assert_eq!(
            s.step(target),
            DriveCommand {
                left: 300,
                right: -300
            }
        );
        assert_eq!(s.step(target), target, "lands exactly on the target");
        assert_eq!(
            s.step(DriveCommand::default()),
            DriveCommand {
                left: 250,
                right: -250
            }
        );
    }

    #[test]
    fn zero_step_disables_ramping() {
        let mut s = Slew::new(0);
        let target = DriveCommand {
            left: 1000,
            right: 1000,
        };
        assert_eq!(s.step(target), target);
    }

    #[test]
    fn reset_restarts_from_the_given_value() {
        let mut s = Slew::new(50);
        s.step(DriveCommand {
            left: 50,
            right: 50,
        });
        s.reset_to(DriveCommand::default());
        assert_eq!(s.current(), DriveCommand::default());
    }
}
