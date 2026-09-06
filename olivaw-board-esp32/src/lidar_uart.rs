//! UART1 for the RPLIDAR C1: 460800 8N1 on GPIO27 (RX) / GPIO25 (TX).
//!
//! The ESP32 has no UART DMA and a 128-byte RX FIFO; at the C1's ~25 KB/s
//! that is ~5 ms of slack. The FIFO-full threshold is set low (64 bytes)
//! and the idle timeout short so the reader wakes with room to spare.

use esp_hal::Async;
use esp_hal::peripherals::{GPIO25, GPIO27, UART1};
use esp_hal::uart::{Config, ConfigError, RxConfig, Uart};

use crate::pins::LIDAR_BAUD;

/// RX FIFO fill level that wakes the reader, bytes (max 127).
pub const RX_FIFO_THRESHOLD: u16 = 64;
/// Idle time that also wakes the reader, in symbol periods.
pub const RX_TIMEOUT_SYMBOLS: u8 = 4;

/// Configure UART1 for the lidar and return it in async mode.
///
/// # Errors
///
/// [`ConfigError`] if the baud rate or FIFO threshold is rejected (only if
/// the constants above are changed to invalid values).
pub fn init(
    uart: UART1<'static>,
    rx: GPIO27<'static>,
    tx: GPIO25<'static>,
) -> Result<Uart<'static, Async>, ConfigError> {
    let config = Config::default().with_baudrate(LIDAR_BAUD).with_rx(
        RxConfig::default()
            .with_fifo_full_threshold(RX_FIFO_THRESHOLD)
            .with_timeout(RX_TIMEOUT_SYMBOLS),
    );
    Ok(Uart::new(uart, config)?
        .with_rx(rx)
        .with_tx(tx)
        .into_async())
}
