//! Two L298N channels on LEDC low-speed timer 0, 1 kHz, 8-bit duty.

use esp_hal::gpio::{DriveMode, Level, Output, OutputConfig};
use esp_hal::ledc::channel::{self, ChannelIFace as _};
use esp_hal::ledc::timer::{self, TimerIFace as _};
use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::peripherals::{GPIO16, GPIO17, GPIO18, GPIO19, GPIO22, GPIO23, LEDC};
use esp_hal::time::Rate;
use olivaw_drive::{L298n, Motor};
use static_cell::StaticCell;

use crate::board::BoardError;
use crate::pins::MOTOR_PWM_HZ;

/// One LEDC PWM output.
pub type PwmChannel = channel::Channel<'static, LowSpeed>;
/// One motor channel: two direction pins + PWM enable.
pub type CarMotor = Motor<Output<'static>, Output<'static>, PwmChannel>;
/// Both motors as a differential pair (`left`, `right`).
pub type CarMotors = L298n<CarMotor, CarMotor>;

/// The GPIOs the motor driver is wired to.
pub struct MotorPins {
    /// L298N ENA.
    pub right_en: GPIO22<'static>,
    /// L298N IN1.
    pub right_in1: GPIO16<'static>,
    /// L298N IN2.
    pub right_in2: GPIO17<'static>,
    /// L298N ENB.
    pub left_en: GPIO23<'static>,
    /// L298N IN3.
    pub left_in3: GPIO18<'static>,
    /// L298N IN4.
    pub left_in4: GPIO19<'static>,
}

// The channel config borrows the timer for as long as the channel lives, so
// the timer must be 'static.
static LEDC_TIMER0: StaticCell<timer::Timer<'static, LowSpeed>> = StaticCell::new();

fn low<'d>(pin: impl esp_hal::gpio::OutputPin + 'd) -> Output<'d> {
    Output::new(pin, Level::Low, OutputConfig::default())
}

/// Configure LEDC and wrap the six pins into [`CarMotors`], stopped.
///
/// # Errors
///
/// [`BoardError::Pwm`] if the timer or a channel rejects the configuration
/// (only possible if `MOTOR_PWM_HZ` / duty resolution are out of range).
pub fn init(ledc: LEDC<'static>, pins: MotorPins) -> Result<CarMotors, BoardError> {
    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let timer = LEDC_TIMER0.init(ledc.timer::<LowSpeed>(timer::Number::Timer0));
    timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(MOTOR_PWM_HZ),
        })
        .map_err(|_| BoardError::Pwm)?;
    let timer: &'static timer::Timer<'static, LowSpeed> = timer;

    let mut right_en = ledc.channel::<LowSpeed>(channel::Number::Channel0, pins.right_en);
    right_en
        .configure(channel::config::Config {
            timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .map_err(|_| BoardError::Pwm)?;
    let mut left_en = ledc.channel::<LowSpeed>(channel::Number::Channel1, pins.left_en);
    left_en
        .configure(channel::config::Config {
            timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .map_err(|_| BoardError::Pwm)?;

    let right = Motor::new(low(pins.right_in1), low(pins.right_in2), right_en);
    let left = Motor::new(low(pins.left_in3), low(pins.left_in4), left_en);
    Ok(L298n::new(left, right))
}
