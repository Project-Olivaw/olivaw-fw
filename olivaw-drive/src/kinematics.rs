//! Differential-drive kinematics: `(v, ω)` twist commands → per-side wheel
//! speeds, and back.
//!
//! Pure math, `no_std`, zero dependencies. Units are explicit everywhere:
//! metres, seconds, radians (counter-clockwise positive, x forward, y left —
//! the same convention as `slam/core-types`).
//!
//! The normalization used by [`DifferentialDrive::mix`] preserves the
//! turn/speed *ratio* when a command exceeds the wheels' capability: both
//! sides are scaled down together rather than clamped independently, so a
//! full-speed turn still turns instead of going straight. That subtlety is
//! the reason this is a component and not three lines in `main.rs`.

/// Geometry + limits of a differential-drive base.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialDrive {
    /// Distance between the left and right wheel contact points, metres.
    pub track_width_m: f32,
    /// Wheel speed that corresponds to full command (per-mille 1000), m/s.
    pub max_wheel_speed_mps: f32,
}

/// Per-side wheel linear speeds, m/s. Positive = forward.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WheelSpeeds {
    /// Left wheel ground speed, m/s.
    pub left_mps: f32,
    /// Right wheel ground speed, m/s.
    pub right_mps: f32,
}

/// A drive command in per-mille of full speed per side (`-1000..=1000`),
/// ready for `drivers/l298n` / `comms/cmdvel-protocol`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MixOutput {
    /// Left side command, `-1000..=1000`.
    pub left_permille: i16,
    /// Right side command, `-1000..=1000`.
    pub right_permille: i16,
}

impl DifferentialDrive {
    /// Construct from geometry. `track_width_m` and `max_wheel_speed_mps`
    /// must be positive and finite; violations are clamped to small positive
    /// values rather than panicking (this code runs in control loops).
    #[must_use]
    pub fn new(track_width_m: f32, max_wheel_speed_mps: f32) -> Self {
        let sanitize = |v: f32, fallback: f32| {
            if v.is_finite() && v > 0.0 {
                v
            } else {
                fallback
            }
        };
        Self {
            track_width_m: sanitize(track_width_m, 0.1),
            max_wheel_speed_mps: sanitize(max_wheel_speed_mps, 0.1),
        }
    }

    /// Inverse kinematics: body twist → wheel speeds.
    ///
    /// `v_mps` is forward speed (m/s), `omega_radps` is yaw rate (rad/s,
    /// counter-clockwise positive → positive ω turns left, so the right
    /// wheel runs faster).
    #[must_use]
    pub fn wheel_speeds(&self, v_mps: f32, omega_radps: f32) -> WheelSpeeds {
        let half_track = self.track_width_m / 2.0;
        WheelSpeeds {
            left_mps: v_mps - omega_radps * half_track,
            right_mps: v_mps + omega_radps * half_track,
        }
    }

    /// Forward kinematics: wheel speeds → body twist `(v_mps, omega_radps)`.
    /// The exact inverse of [`wheel_speeds`](Self::wheel_speeds).
    #[must_use]
    pub fn twist(&self, wheels: WheelSpeeds) -> (f32, f32) {
        let v = f32::midpoint(wheels.left_mps, wheels.right_mps);
        let omega = (wheels.right_mps - wheels.left_mps) / self.track_width_m;
        (v, omega)
    }

    /// Inverse kinematics straight to motor commands: twist → per-mille per
    /// side, ratio-preserving.
    ///
    /// If either wheel would exceed `max_wheel_speed_mps`, **both** sides are
    /// scaled down by the same factor, keeping the commanded arc. Non-finite
    /// inputs produce a stop command.
    #[must_use]
    pub fn mix(&self, v_mps: f32, omega_radps: f32) -> MixOutput {
        if !v_mps.is_finite() || !omega_radps.is_finite() {
            return MixOutput::default();
        }
        let wheels = self.wheel_speeds(v_mps, omega_radps);
        let peak = wheels.left_mps.abs().max(wheels.right_mps.abs());
        let scale = if peak > self.max_wheel_speed_mps {
            self.max_wheel_speed_mps / peak
        } else {
            1.0
        };
        let to_permille = |mps: f32| {
            let permille = mps * scale / self.max_wheel_speed_mps * 1000.0;
            // Clamp guards the boundary case peak == max (rounding overshoot).
            #[allow(clippy::cast_possible_truncation)]
            {
                permille.clamp(-1000.0, 1000.0) as i16
            }
        };
        MixOutput {
            left_permille: to_permille(wheels.left_mps),
            right_permille: to_permille(wheels.right_mps),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BASE: DifferentialDrive = DifferentialDrive {
        track_width_m: 0.2,
        max_wheel_speed_mps: 1.0,
    };

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn straight_line_drives_both_wheels_equally() {
        let w = BASE.wheel_speeds(0.5, 0.0);
        assert!(close(w.left_mps, 0.5) && close(w.right_mps, 0.5));
        let m = BASE.mix(0.5, 0.0);
        assert_eq!(
            m,
            MixOutput {
                left_permille: 500,
                right_permille: 500
            }
        );
    }

    #[test]
    fn positive_omega_turns_left() {
        // CCW-positive: turning left → right wheel faster.
        let w = BASE.wheel_speeds(0.5, 1.0);
        assert!(w.right_mps > w.left_mps);
        assert!(close(w.left_mps, 0.4) && close(w.right_mps, 0.6));
    }

    #[test]
    fn spin_in_place_is_antisymmetric() {
        let m = BASE.mix(0.0, 5.0);
        assert_eq!(m.left_permille, -m.right_permille);
        assert!(m.right_permille > 0);
    }

    #[test]
    fn forward_and_inverse_kinematics_round_trip() {
        let (v, omega) = (0.3, -1.2);
        let w = BASE.wheel_speeds(v, omega);
        let (v2, omega2) = BASE.twist(w);
        assert!(close(v, v2) && close(omega, omega2));
    }

    #[test]
    fn saturation_preserves_the_arc_ratio() {
        // 2 m/s forward + hard turn: way over the 1 m/s wheel limit.
        let m = BASE.mix(2.0, 10.0);
        assert!(m.left_permille.abs() <= 1000 && m.right_permille.abs() <= 1000);
        // The faster side pegs at full scale…
        assert_eq!(m.right_permille, 1000);
        // …and the ratio between sides survives (1.0 : 3.0 before scaling).
        let ratio = f32::from(m.left_permille) / f32::from(m.right_permille);
        assert!((ratio - (1.0 / 3.0)).abs() < 0.01, "ratio {ratio}");
    }

    #[test]
    fn non_finite_input_stops() {
        assert_eq!(BASE.mix(f32::NAN, 0.0), MixOutput::default());
        assert_eq!(BASE.mix(0.0, f32::INFINITY), MixOutput::default());
    }

    #[test]
    fn constructor_sanitizes_bad_geometry() {
        let bad = DifferentialDrive::new(-1.0, f32::NAN);
        assert!(bad.track_width_m > 0.0);
        assert!(bad.max_wheel_speed_mps > 0.0);
    }
}
