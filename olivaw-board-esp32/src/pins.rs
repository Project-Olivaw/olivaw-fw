//! The pin map, in one place.
//!
//! | Function | GPIO | Why this pin |
//! | --- | --- | --- |
//! | Right motor EN (PWM) | 22 | Reference wiring (Hands-On-Robotics module 07) |
//! | Right motor IN1 / IN2 | 16 / 17 | Reference wiring |
//! | Left motor EN (PWM) | 23 | Reference wiring |
//! | Left motor IN3 / IN4 | 18 / 19 | Reference wiring |
//! | Battery sense | 34 | `ADC1_CH6`, input-only, unaffected by Wi-Fi (ADC2 is unusable with Wi-Fi) |
//! | Lidar RX (← C1 TX) | 27 | Plain GPIO-matrix pin, no strapping role |
//! | Lidar TX (→ C1 RX) | 25 | Plain GPIO-matrix pin (also DAC1, harmless) |
//! | Status LED | 2 | Onboard LED on `DevKit V1` (strapping pin, fine as output) |
//!
//! Avoided on purpose: GPIO 6–11 (flash), 0/5/12/15 (strapping), 34–39
//! (input-only, so never for outputs), and every ADC2 channel.

/// Right motor PWM enable (L298N ENA).
pub const MOTOR_RIGHT_EN: u8 = 22;
/// Right motor direction (L298N IN1).
pub const MOTOR_RIGHT_IN1: u8 = 16;
/// Right motor direction (L298N IN2).
pub const MOTOR_RIGHT_IN2: u8 = 17;
/// Left motor PWM enable (L298N ENB).
pub const MOTOR_LEFT_EN: u8 = 23;
/// Left motor direction (L298N IN3).
pub const MOTOR_LEFT_IN3: u8 = 18;
/// Left motor direction (L298N IN4).
pub const MOTOR_LEFT_IN4: u8 = 19;
/// Battery divider node (ADC1 channel 6).
pub const BATTERY_SENSE: u8 = 34;
/// UART RX from the lidar's TX.
pub const LIDAR_RX: u8 = 27;
/// UART TX to the lidar's RX.
pub const LIDAR_TX: u8 = 25;
/// Onboard status LED.
pub const STATUS_LED: u8 = 2;

/// Motor PWM frequency, hertz (reference firmware: 1 kHz).
pub const MOTOR_PWM_HZ: u32 = 1_000;
/// Lidar UART baud rate (RPLIDAR C1).
pub const LIDAR_BAUD: u32 = 460_800;
