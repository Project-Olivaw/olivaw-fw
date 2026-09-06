//! Low-battery state machine with hysteresis, so the warning does not
//! flicker every time the motors sag the pack for a moment.

/// Thresholds, pack millivolts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MonitorConfig {
    /// Below this → [`BatteryState::Low`] (3S: 3.50 V/cell).
    pub warn_mv: u32,
    /// Below this → [`BatteryState::Critical`], driving inhibited (3S: 3.30 V/cell).
    pub critical_mv: u32,
    /// Must recover by this much above a threshold to step back up.
    pub hysteresis_mv: u32,
    /// Consecutive samples below a threshold before acting (debounce).
    pub debounce_samples: u8,
}

impl MonitorConfig {
    /// The car's 3S pack, sampled at 2 Hz: 2 s of confirmation.
    pub const CAR_3S: MonitorConfig = MonitorConfig {
        warn_mv: 10_500,
        critical_mv: 9_900,
        hysteresis_mv: 300,
        debounce_samples: 4,
    };
}

/// Battery health verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BatteryState {
    /// Fine.
    #[default]
    Ok,
    /// Head home.
    Low,
    /// Stop driving now.
    Critical,
}

/// Debounced, hysteretic threshold tracker.
#[derive(Debug, Clone, Copy)]
pub struct BatteryMonitor {
    cfg: MonitorConfig,
    state: BatteryState,
    below_warn: u8,
    below_crit: u8,
}

impl BatteryMonitor {
    /// Start in [`BatteryState::Ok`].
    #[must_use]
    pub const fn new(cfg: MonitorConfig) -> Self {
        Self {
            cfg,
            state: BatteryState::Ok,
            below_warn: 0,
            below_crit: 0,
        }
    }

    /// Current verdict.
    #[must_use]
    pub const fn state(&self) -> BatteryState {
        self.state
    }

    /// Feed one pack reading, millivolts; returns the (possibly new) state.
    pub fn update(&mut self, pack_mv: u32) -> BatteryState {
        let cfg = self.cfg;
        self.below_crit = if pack_mv < cfg.critical_mv {
            self.below_crit.saturating_add(1)
        } else {
            0
        };
        self.below_warn = if pack_mv < cfg.warn_mv {
            self.below_warn.saturating_add(1)
        } else {
            0
        };

        self.state = match self.state {
            BatteryState::Ok => {
                if self.below_crit >= cfg.debounce_samples {
                    BatteryState::Critical
                } else if self.below_warn >= cfg.debounce_samples {
                    BatteryState::Low
                } else {
                    BatteryState::Ok
                }
            }
            BatteryState::Low => {
                if self.below_crit >= cfg.debounce_samples {
                    BatteryState::Critical
                } else if pack_mv >= cfg.warn_mv + cfg.hysteresis_mv {
                    BatteryState::Ok
                } else {
                    BatteryState::Low
                }
            }
            BatteryState::Critical => {
                if pack_mv >= cfg.critical_mv + cfg.hysteresis_mv {
                    BatteryState::Low
                } else {
                    BatteryState::Critical
                }
            }
        };
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn monitor() -> BatteryMonitor {
        BatteryMonitor::new(MonitorConfig {
            warn_mv: 10_500,
            critical_mv: 9_900,
            hysteresis_mv: 300,
            debounce_samples: 3,
        })
    }

    #[test]
    fn a_single_sag_does_not_trip() {
        let mut m = monitor();
        assert_eq!(m.update(10_000), BatteryState::Ok);
        assert_eq!(m.update(11_000), BatteryState::Ok);
    }

    #[test]
    fn sustained_low_then_recovery_with_hysteresis() {
        let mut m = monitor();
        for _ in 0..3 {
            m.update(10_400);
        }
        assert_eq!(m.state(), BatteryState::Low);
        assert_eq!(
            m.update(10_600),
            BatteryState::Low,
            "inside the hysteresis band"
        );
        assert_eq!(m.update(10_900), BatteryState::Ok);
    }

    #[test]
    fn critical_steps_down_through_low() {
        let mut m = monitor();
        for _ in 0..3 {
            m.update(9_500);
        }
        assert_eq!(m.state(), BatteryState::Critical);
        assert_eq!(m.update(10_300), BatteryState::Low);
        assert_eq!(m.update(10_900), BatteryState::Ok);
    }
}
