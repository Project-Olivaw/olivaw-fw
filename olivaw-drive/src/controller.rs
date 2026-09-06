//! The drive pipeline the firmware task runs every tick.
//!
//! Composition, in order: watchdog (link liveness) → E-stop latch → battery
//! inhibit → governor (duty cap) → slew (ramp). The output says whether to
//! brake or what to apply. Everything is plain data, so the whole safety
//! story is unit-tested without a motor in sight.

use crate::cmdvel::{DriveCommand, Watchdog};
use crate::governor::Governor;
use crate::slew::Slew;

/// Tunables for [`DriveController`]. Every number the drive loop uses lives
/// here, named and documented, so the firmware's `config.rs` is the only
/// place that decides them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriveConfig {
    /// No fresh command within this window → coast. Reference firmware: 500 ms.
    pub watchdog_ms: u32,
    /// Duty cap, per-mille. 550 keeps 6 V TT motors happy on a 3S pack.
    pub max_duty_permille: u16,
    /// Maximum per-tick change, per-mille. 0 disables ramping.
    pub slew_step_permille: i16,
}

impl Default for DriveConfig {
    fn default() -> Self {
        Self {
            watchdog_ms: 500,
            max_duty_permille: 550,
            slew_step_permille: 60,
        }
    }
}

/// What the motors should do this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveOutput {
    /// Apply these speeds (already capped and ramped).
    Drive(DriveCommand),
    /// Actively brake (E-stop).
    Brake,
}

impl DriveOutput {
    /// The commanded speeds, zeros when braking — what telemetry reports.
    #[must_use]
    pub const fn command(self) -> DriveCommand {
        match self {
            Self::Drive(c) => c,
            Self::Brake => DriveCommand { left: 0, right: 0 },
        }
    }
}

/// Watchdog + E-stop + inhibit + governor + slew.
#[derive(Debug, Clone, Copy)]
pub struct DriveController {
    watchdog: Watchdog,
    governor: Governor,
    slew: Slew,
    estop: bool,
    inhibited: bool,
    stale: bool,
    watchdog_ms: u32,
    last_feed_ms: Option<u32>,
}

impl DriveController {
    /// Build from config. Starts stopped, not E-stopped.
    #[must_use]
    pub const fn new(cfg: DriveConfig) -> Self {
        Self {
            watchdog: Watchdog::new(cfg.watchdog_ms),
            governor: Governor::new(cfg.max_duty_permille),
            slew: Slew::new(cfg.slew_step_permille),
            estop: false,
            inhibited: false,
            stale: true,
            watchdog_ms: cfg.watchdog_ms,
            last_feed_ms: None,
        }
    }

    /// A fresh command arrived from the link at `now_ms`.
    pub fn feed(&mut self, cmd: DriveCommand, now_ms: u32) {
        self.watchdog.feed(cmd, now_ms);
        self.last_feed_ms = Some(now_ms);
    }

    /// The link dropped: forget the last command immediately.
    pub fn link_lost(&mut self) {
        self.watchdog.trip();
        self.last_feed_ms = None;
    }

    /// Latch the emergency stop. Motors brake until [`clear`](Self::clear).
    pub fn estop(&mut self) {
        self.estop = true;
        self.watchdog.trip();
        self.last_feed_ms = None;
        self.slew.reset_to(DriveCommand::default());
    }

    /// Release the emergency stop.
    pub fn clear(&mut self) {
        self.estop = false;
    }

    /// `true` while the E-stop is latched.
    #[must_use]
    pub const fn is_estopped(&self) -> bool {
        self.estop
    }

    /// Inhibit driving (e.g. critical battery). Unlike E-stop this coasts
    /// rather than brakes and clears itself when `inhibited` goes false.
    pub fn set_inhibited(&mut self, inhibited: bool) {
        self.inhibited = inhibited;
    }

    /// `true` if the last tick found no fresh command.
    #[must_use]
    pub const fn is_stale(&self) -> bool {
        self.stale
    }

    /// Change the duty cap, per-mille (clamped to `0..=1000`).
    pub fn set_max_duty(&mut self, permille: u16) {
        self.governor.set_max_permille(permille);
    }

    /// The duty cap in force.
    #[must_use]
    pub const fn max_duty(&self) -> u16 {
        self.governor.max_permille()
    }

    /// Run one control tick at `now_ms` and return what to apply.
    pub fn tick(&mut self, now_ms: u32) -> DriveOutput {
        self.stale = !self.link_fresh(now_ms);
        if self.estop {
            return DriveOutput::Brake;
        }
        let wanted = self.watchdog.command(now_ms);
        let target = if self.inhibited {
            DriveCommand::default()
        } else {
            self.governor.apply(wanted)
        };
        DriveOutput::Drive(self.slew.step(target))
    }

    fn link_fresh(&self, now_ms: u32) -> bool {
        matches!(self.last_feed_ms, Some(t) if now_ms.wrapping_sub(t) <= self.watchdog_ms)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctl() -> DriveController {
        DriveController::new(DriveConfig {
            watchdog_ms: 500,
            max_duty_permille: 500,
            slew_step_permille: 0,
        })
    }

    #[test]
    fn caps_and_passes_a_fresh_command() {
        let mut c = ctl();
        c.feed(
            DriveCommand {
                left: 1000,
                right: -1000,
            },
            0,
        );
        assert_eq!(
            c.tick(10),
            DriveOutput::Drive(DriveCommand {
                left: 500,
                right: -500
            })
        );
        assert!(!c.is_stale());
    }

    #[test]
    fn watchdog_coasts_after_timeout() {
        let mut c = ctl();
        c.feed(
            DriveCommand {
                left: 800,
                right: 800,
            },
            0,
        );
        assert_eq!(c.tick(600), DriveOutput::Drive(DriveCommand::default()));
        assert!(c.is_stale());
    }

    #[test]
    fn estop_brakes_until_cleared() {
        let mut c = ctl();
        c.feed(
            DriveCommand {
                left: 800,
                right: 800,
            },
            0,
        );
        c.estop();
        assert_eq!(c.tick(1), DriveOutput::Brake);
        c.feed(
            DriveCommand {
                left: 800,
                right: 800,
            },
            2,
        );
        assert_eq!(
            c.tick(3),
            DriveOutput::Brake,
            "commands are ignored while latched"
        );
        c.clear();
        c.feed(
            DriveCommand {
                left: 800,
                right: 800,
            },
            4,
        );
        assert_eq!(
            c.tick(5),
            DriveOutput::Drive(DriveCommand {
                left: 400,
                right: 400
            })
        );
    }

    #[test]
    fn inhibit_coasts_but_keeps_listening() {
        let mut c = ctl();
        c.set_inhibited(true);
        c.feed(
            DriveCommand {
                left: 800,
                right: 800,
            },
            0,
        );
        assert_eq!(c.tick(1), DriveOutput::Drive(DriveCommand::default()));
        c.set_inhibited(false);
        assert_eq!(
            c.tick(2),
            DriveOutput::Drive(DriveCommand {
                left: 400,
                right: 400
            })
        );
    }

    #[test]
    fn slew_ramps_the_capped_target() {
        let mut c = DriveController::new(DriveConfig {
            watchdog_ms: 500,
            max_duty_permille: 1000,
            slew_step_permille: 100,
        });
        c.feed(
            DriveCommand {
                left: 300,
                right: 300,
            },
            0,
        );
        assert_eq!(
            c.tick(1),
            DriveOutput::Drive(DriveCommand {
                left: 100,
                right: 100
            })
        );
        assert_eq!(
            c.tick(2),
            DriveOutput::Drive(DriveCommand {
                left: 200,
                right: 200
            })
        );
        assert_eq!(
            c.tick(3),
            DriveOutput::Drive(DriveCommand {
                left: 300,
                right: 300
            })
        );
    }

    #[test]
    fn link_lost_stops_without_waiting() {
        let mut c = ctl();
        c.feed(
            DriveCommand {
                left: 800,
                right: 800,
            },
            0,
        );
        c.link_lost();
        assert_eq!(c.tick(1), DriveOutput::Drive(DriveCommand::default()));
    }

    #[test]
    fn max_duty_can_change_at_runtime() {
        let mut c = ctl();
        c.set_max_duty(1000);
        c.feed(
            DriveCommand {
                left: 1000,
                right: 1000,
            },
            0,
        );
        assert_eq!(
            c.tick(1),
            DriveOutput::Drive(DriveCommand {
                left: 1000,
                right: 1000
            })
        );
        assert_eq!(c.max_duty(), 1000);
    }
}
