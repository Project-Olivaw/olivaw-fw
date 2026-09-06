//! `Board::init`: peripherals in, typed parts out.

use esp_hal::gpio::Output;
use esp_hal::peripherals::{BT, Peripherals, SW_INTERRUPT, TIMG0, WIFI};

use crate::battery::BatteryAdc;
use crate::motors::{CarMotors, MotorPins};

/// Board bring-up failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum BoardError {
    /// LEDC timer/channel configuration was rejected.
    Pwm,
    /// The lidar UART configuration was rejected.
    LidarUart,
}

impl core::fmt::Display for BoardError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Pwm => write!(f, "LEDC PWM configuration rejected — check MOTOR_PWM_HZ"),
            Self::LidarUart => write!(f, "lidar UART configuration rejected — check LIDAR_BAUD"),
        }
    }
}

/// Radio and scheduler peripherals handed back to the binary untouched.
pub struct Radio {
    /// Bluetooth controller peripheral (for `esp_radio::ble::controller::BleConnector`).
    pub bt: BT<'static>,
    /// Wi-Fi peripheral (for `esp_radio::wifi::WifiController`).
    pub wifi: WIFI<'static>,
    /// Timer group 0 (esp-rtos time base).
    pub timg0: TIMG0<'static>,
    /// Software interrupts (esp-rtos scheduler + interrupt executors).
    pub sw_interrupt: SW_INTERRUPT<'static>,
}

/// Everything the firmware tasks need, already configured.
pub struct Board {
    /// Left + right motors through the L298N, stopped.
    pub motors: CarMotors,
    /// Battery divider ADC.
    pub battery: BatteryAdc,
    /// Onboard status LED, off.
    pub led: Output<'static>,
    /// RPLIDAR C1 UART, async.
    #[cfg(feature = "lidar")]
    pub lidar_uart: esp_hal::uart::Uart<'static, esp_hal::Async>,
    /// Radio + scheduler peripherals.
    pub radio: Radio,
}

impl Board {
    /// Take the pins in [`crate::pins`] and configure every part.
    ///
    /// # Errors
    ///
    /// [`BoardError`] if a peripheral rejects its configuration.
    pub fn init(p: Peripherals) -> Result<Self, BoardError> {
        let motors = crate::motors::init(
            p.LEDC,
            MotorPins {
                right_en: p.GPIO22,
                right_in1: p.GPIO16,
                right_in2: p.GPIO17,
                left_en: p.GPIO23,
                left_in3: p.GPIO18,
                left_in4: p.GPIO19,
            },
        )?;
        let battery = BatteryAdc::new(p.ADC1, p.GPIO34);
        let led = crate::led::init(p.GPIO2);
        #[cfg(feature = "lidar")]
        let lidar_uart = crate::lidar_uart::init(p.UART1, p.GPIO27, p.GPIO25)
            .map_err(|_| BoardError::LidarUart)?;
        Ok(Self {
            motors,
            battery,
            led,
            #[cfg(feature = "lidar")]
            lidar_uart,
            radio: Radio {
                bt: p.BT,
                wifi: p.WIFI,
                timg0: p.TIMG0,
                sw_interrupt: p.SW_INTERRUPT,
            },
        })
    }
}
