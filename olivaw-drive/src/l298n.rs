//! L298N dual H-bridge DC motor driver.
//!
//! Generic over `embedded-hal 1.0` traits: two direction pins
//! ([`OutputPin`]) and one PWM enable pin ([`SetDutyCycle`]) per motor. No
//! HAL types, no platform code — construct it from whatever your HAL gives
//! you.
//!
//! Speed is signed per-mille of full duty: `-1000..=1000`. Positive is
//! "forward" for however you wired IN1/IN2; swap the pins (or negate) if
//! your robot drives backwards. Out-of-range values are clamped, matching
//! the reference firmware's behaviour.
//!
//! Truth table per motor (from the L298N datasheet):
//!
//! | IN1 | IN2 | EN  | state                       |
//! |-----|-----|-----|-----------------------------|
//! | 1   | 0   | PWM | forward, speed ∝ duty       |
//! | 0   | 1   | PWM | reverse, speed ∝ duty       |
//! | 0   | 0   | x   | coast                       |
//! | 1   | 1   | x   | brake (both terminals tied) |
//!
//! Ported from the Hands-On-Robotics module 07 C++ firmware (`ESP32 DevKit
//! v1`, 1 kHz PWM). The 500 ms command watchdog from that firmware lives in
//! `comms/cmdvel-protocol`, not here — this driver only moves motors.

use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

/// Full-scale speed magnitude: [`Motor::drive`] accepts `-1000..=1000`.
pub const SPEED_MAX: i16 = 1000;

/// Driver failure: one of the underlying pins failed.
///
/// `PinE` is the direction pins' error type, `PwmE` the enable pin's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<PinE, PwmE> {
    /// A direction pin (IN1/IN2) could not be set.
    Pin(PinE),
    /// The PWM duty cycle could not be set on the enable pin.
    Pwm(PwmE),
}

/// One side of the L298N: IN1/IN2 direction pins + EN PWM pin.
pub struct Motor<In1, In2, En> {
    in1: In1,
    in2: In2,
    en: En,
}

impl<PinE, In1, In2, En> Motor<In1, In2, En>
where
    In1: OutputPin<Error = PinE>,
    In2: OutputPin<Error = PinE>,
    En: SetDutyCycle,
{
    /// Wrap the three control pins of one motor channel.
    ///
    /// The pins are left untouched until the first [`drive`](Self::drive) /
    /// [`coast`](Self::coast) / [`brake`](Self::brake) call.
    pub fn new(in1: In1, in2: In2, en: En) -> Self {
        Self { in1, in2, en }
    }

    /// Set speed as signed per-mille of full duty, `-1000..=1000`.
    ///
    /// `0` coasts (both direction pins low). Values outside the range are
    /// clamped. Duty resolution is whatever the PWM pin provides — the
    /// per-mille value is rescaled to its `max_duty_cycle()`.
    ///
    /// # Errors
    ///
    /// Propagates the first pin or PWM failure; the motor may be left in a
    /// mixed state if that happens — call [`coast`](Self::coast) to recover.
    pub fn drive(&mut self, speed_permille: i16) -> Result<(), Error<PinE, En::Error>> {
        let speed = speed_permille.clamp(-SPEED_MAX, SPEED_MAX);
        match speed {
            s if s > 0 => {
                self.in1.set_high().map_err(Error::Pin)?;
                self.in2.set_low().map_err(Error::Pin)?;
            }
            s if s < 0 => {
                self.in1.set_low().map_err(Error::Pin)?;
                self.in2.set_high().map_err(Error::Pin)?;
            }
            _ => {
                self.in1.set_low().map_err(Error::Pin)?;
                self.in2.set_low().map_err(Error::Pin)?;
            }
        }
        let magnitude = speed.unsigned_abs(); // 0..=1000
        let max = self.en.max_duty_cycle();
        // duty = magnitude/1000 × max, in u32 to avoid overflow.
        let duty = u16::try_from(u32::from(magnitude) * u32::from(max) / 1000).unwrap_or(max);
        self.en.set_duty_cycle(duty).map_err(Error::Pwm)
    }

    /// Let the motor spin freely (IN1=IN2=low, duty 0).
    ///
    /// # Errors
    ///
    /// Propagates the first pin or PWM failure.
    pub fn coast(&mut self) -> Result<(), Error<PinE, En::Error>> {
        self.drive(0)
    }

    /// Actively brake (IN1=IN2=high, duty full).
    ///
    /// # Errors
    ///
    /// Propagates the first pin or PWM failure.
    pub fn brake(&mut self) -> Result<(), Error<PinE, En::Error>> {
        self.in1.set_high().map_err(Error::Pin)?;
        self.in2.set_high().map_err(Error::Pin)?;
        self.en.set_duty_cycle_fully_on().map_err(Error::Pwm)
    }

    /// Release the pins, consuming the driver.
    pub fn release(self) -> (In1, In2, En) {
        (self.in1, self.in2, self.en)
    }
}

/// Both channels of an L298N as a differential pair (left + right motor).
///
/// A convenience wrapper for the common robot-car wiring; use two bare
/// [`Motor`]s instead if your channels are unrelated.
pub struct L298n<Left, Right> {
    /// Left motor channel (ENB/IN3/IN4 on the common breakout).
    pub left: Left,
    /// Right motor channel (ENA/IN1/IN2 on the common breakout).
    pub right: Right,
}

impl<PinE, LI1, LI2, LEn, RI1, RI2, REn> L298n<Motor<LI1, LI2, LEn>, Motor<RI1, RI2, REn>>
where
    LI1: OutputPin<Error = PinE>,
    LI2: OutputPin<Error = PinE>,
    LEn: SetDutyCycle,
    RI1: OutputPin<Error = PinE>,
    RI2: OutputPin<Error = PinE>,
    REn: SetDutyCycle<Error = LEn::Error>,
{
    /// Pair two motor channels.
    pub fn new(left: Motor<LI1, LI2, LEn>, right: Motor<RI1, RI2, REn>) -> Self {
        Self { left, right }
    }

    /// Drive both sides, per-mille `-1000..=1000` each.
    ///
    /// # Errors
    ///
    /// Propagates the first failure; the other side keeps its last state.
    pub fn drive(
        &mut self,
        left_permille: i16,
        right_permille: i16,
    ) -> Result<(), Error<PinE, LEn::Error>> {
        self.left.drive(left_permille)?;
        self.right.drive(right_permille)
    }

    /// Coast both sides.
    ///
    /// # Errors
    ///
    /// Propagates the first failure.
    pub fn stop(&mut self) -> Result<(), Error<PinE, LEn::Error>> {
        self.drive(0, 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;

    /// Minimal mock pin recording its level.
    #[derive(Default)]
    struct MockPin {
        high: bool,
    }
    impl embedded_hal::digital::ErrorType for MockPin {
        type Error = Infallible;
    }
    impl OutputPin for MockPin {
        fn set_low(&mut self) -> Result<(), Infallible> {
            self.high = false;
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Infallible> {
            self.high = true;
            Ok(())
        }
    }

    /// Mock PWM with an 8-bit-style max duty of 255.
    #[derive(Default)]
    struct MockPwm {
        duty: u16,
    }
    impl embedded_hal::pwm::ErrorType for MockPwm {
        type Error = Infallible;
    }
    impl SetDutyCycle for MockPwm {
        fn max_duty_cycle(&self) -> u16 {
            255
        }
        fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Infallible> {
            self.duty = duty;
            Ok(())
        }
    }

    fn motor() -> Motor<MockPin, MockPin, MockPwm> {
        Motor::new(MockPin::default(), MockPin::default(), MockPwm::default())
    }

    #[test]
    fn forward_reverse_and_coast_set_the_truth_table() {
        let mut m = motor();
        m.drive(1000).unwrap();
        assert!(m.in1.high && !m.in2.high);
        assert_eq!(m.en.duty, 255);

        m.drive(-500).unwrap();
        assert!(!m.in1.high && m.in2.high);
        assert_eq!(m.en.duty, 127); // 500/1000 × 255, truncated

        m.coast().unwrap();
        assert!(!m.in1.high && !m.in2.high);
        assert_eq!(m.en.duty, 0);
    }

    #[test]
    fn out_of_range_speeds_clamp() {
        let mut m = motor();
        m.drive(i16::MAX).unwrap();
        assert_eq!(m.en.duty, 255);
        m.drive(i16::MIN).unwrap();
        assert_eq!(m.en.duty, 255);
        assert!(!m.in1.high && m.in2.high);
    }

    #[test]
    fn brake_ties_both_terminals() {
        let mut m = motor();
        m.brake().unwrap();
        assert!(m.in1.high && m.in2.high);
        assert_eq!(m.en.duty, 255);
    }

    #[test]
    fn pair_drives_both_sides() {
        let mut pair = L298n::new(motor(), motor());
        pair.drive(250, -250).unwrap();
        assert!(pair.left.in1.high);
        assert!(pair.right.in2.high);
        pair.stop().unwrap();
        assert_eq!(pair.left.en.duty, 0);
        assert_eq!(pair.right.en.duty, 0);
    }
}
