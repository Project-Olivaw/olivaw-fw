//! The actuator boundary: anything that can turn a [`DriveCommand`] into
//! wheel motion.
//!
//! The controller only ever talks to this trait, so the L298N can be swapped
//! for a TB6612, an ESC pair, or a mock without touching the control logic.

use crate::cmdvel::DriveCommand;
use crate::l298n::{L298n, Motor};
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

/// A pair of driven wheels/tracks.
pub trait Motors {
    /// Driver failure type.
    type Error;

    /// Apply per-side speeds, per-mille `-1000..=1000`.
    ///
    /// # Errors
    ///
    /// Propagates the driver's failure.
    fn set(&mut self, cmd: DriveCommand) -> Result<(), Self::Error>;

    /// Let both sides spin freely.
    ///
    /// # Errors
    ///
    /// Propagates the driver's failure.
    fn coast(&mut self) -> Result<(), Self::Error> {
        self.set(DriveCommand::default())
    }

    /// Actively brake both sides.
    ///
    /// # Errors
    ///
    /// Propagates the driver's failure.
    fn brake(&mut self) -> Result<(), Self::Error>;
}

impl<PinE, LI1, LI2, LEn, RI1, RI2, REn> Motors
    for L298n<Motor<LI1, LI2, LEn>, Motor<RI1, RI2, REn>>
where
    LI1: OutputPin<Error = PinE>,
    LI2: OutputPin<Error = PinE>,
    LEn: SetDutyCycle,
    RI1: OutputPin<Error = PinE>,
    RI2: OutputPin<Error = PinE>,
    REn: SetDutyCycle<Error = LEn::Error>,
{
    type Error = crate::l298n::Error<PinE, LEn::Error>;

    fn set(&mut self, cmd: DriveCommand) -> Result<(), Self::Error> {
        self.drive(cmd.left, cmd.right)
    }

    fn brake(&mut self) -> Result<(), Self::Error> {
        self.left.brake()?;
        self.right.brake()
    }
}

/// A recording mock for tests and the hub's simulator.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct MockMotors {
    /// Last command applied by [`Motors::set`].
    pub last: DriveCommand,
    /// Number of [`Motors::brake`] calls.
    pub brakes: u32,
    /// `true` while braked (cleared by the next `set`).
    pub braked: bool,
}

impl Motors for MockMotors {
    type Error = core::convert::Infallible;

    fn set(&mut self, cmd: DriveCommand) -> Result<(), Self::Error> {
        self.last = cmd;
        self.braked = false;
        Ok(())
    }

    fn brake(&mut self) -> Result<(), Self::Error> {
        self.last = DriveCommand::default();
        self.braked = true;
        self.brakes += 1;
        Ok(())
    }
}
