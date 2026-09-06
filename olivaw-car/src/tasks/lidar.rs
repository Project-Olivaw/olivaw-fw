//! RPLIDAR C1 on UART1: starts/stops on `LIDAR_CTL`, pumps bytes into
//! rotations, publishes them to `SCANS` (drop-oldest).
//!
//! The reader runs in the thread executor. If Wi-Fi traffic ever starves it
//! (FIFO overruns show up in telemetry as `UART_OVERRUN` and in the
//! `uart_overruns` counter) the documented upgrade is to move only the
//! `read_async` → `Pipe` copy into an `esp_rtos::embassy::InterruptExecutor`.

use embassy_futures::select::{Either, select};
use esp_hal::Async;
use esp_hal::uart::{RxError, Uart};
use olivaw_lidar_pump::{FrameAssembler, Pump, scan_request, stop_request};
use olivaw_proto::{Flags, LinkState, ScanFrame};

use crate::shared::{LIDAR_CTL, SCANS, now_ms, with_state};

/// Bytes read per wake-up (twice the FIFO).
const CHUNK: usize = 256;

/// Owns the lidar UART.
#[embassy_executor::task]
pub async fn run(mut uart: Uart<'static, Async>) {
    let mut pump = Pump::new();
    let mut frames = FrameAssembler::new();
    let mut buf = [0u8; CHUNK];

    loop {
        set_link(LinkState::Off);
        if !LIDAR_CTL.wait().await {
            continue;
        }

        set_link(LinkState::Starting);
        if let Err(e) = uart.write_async(scan_request().bytes()).await {
            log::error!("lidar: cannot send SCAN: {e:?}");
            set_link(LinkState::Fault);
            continue;
        }
        pump.start();
        frames.reset();
        log::info!("lidar: scanning");

        loop {
            match select(uart.read_async(&mut buf), LIDAR_CTL.wait()).await {
                Either::First(Ok(n)) => {
                    let now = now_ms();
                    pump.push(&buf[..n], |node| {
                        if let Some(frame) = frames.push(node, now) {
                            publish(frame);
                        }
                    });
                    if pump.is_streaming() {
                        set_link(LinkState::Up);
                    }
                }
                Either::First(Err(RxError::FifoOverflowed)) => {
                    with_state(|s| {
                        s.uart_overruns = s.uart_overruns.saturating_add(1);
                        s.flags.insert(Flags::UART_OVERRUN);
                    });
                }
                Either::First(Err(e)) => log::warn!("lidar: uart rx: {e:?}"),
                Either::Second(true) => {}
                Either::Second(false) => break,
            }
        }

        if let Err(e) = uart.write_async(stop_request().bytes()).await {
            log::warn!("lidar: cannot send STOP: {e:?}");
        }
        pump.stop();
        let stats = pump.stats();
        log::info!(
            "lidar: stopped after {} nodes ({} resync bytes, {} descriptor skips)",
            stats.nodes,
            stats.resync_bytes,
            stats.descriptor_skips
        );
    }
}

fn set_link(state: LinkState) {
    with_state(|s| s.lidar = state);
}

/// Drop the oldest queued rotation rather than block the reader.
fn publish(frame: ScanFrame) {
    if let Err(embassy_sync::channel::TrySendError::Full(frame)) = SCANS.try_send(frame) {
        let _ = SCANS.try_receive();
        let _ = SCANS.try_send(frame);
    }
}
