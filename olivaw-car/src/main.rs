//! Olivaw car firmware — ESP32-WROOM-32D.
//!
//! `main` only brings up the board, the scheduler and the radio, then spawns
//! one task per concern (see `tasks/`). Behaviour lives in the tasks and
//! the numbers in `config.rs`.

#![no_std]
#![no_main]

mod config;
mod control;
mod heap;
#[cfg_attr(not(feature = "uplink"), allow(dead_code))]
mod secrets;
mod shared;
mod tasks;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use olivaw_board_esp32::Board;
use trouble_host::prelude::ExternalController;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    log::info!("olivaw-car {} starting", env!("CARGO_PKG_VERSION"));

    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    heap::init();

    let board = match Board::init(peripherals) {
        Ok(board) => board,
        Err(e) => {
            log::error!("board init failed: {e}");
            halt().await
        }
    };
    let radio = board.radio;

    // The preemptive scheduler must run before any radio initialisation.
    let timg0 = TimerGroup::new(radio.timg0);
    let sw_int = SoftwareInterruptControl::new(radio.sw_interrupt);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let connector = match BleConnector::new(radio.bt, esp_radio::ble::Config::default()) {
        Ok(c) => c,
        Err(e) => {
            log::error!("BLE controller init failed: {e:?}");
            halt().await
        }
    };
    let controller: tasks::ble::Controller = ExternalController::new(connector);

    spawn(spawner, "drive", tasks::drive::run(board.motors));
    spawn(spawner, "battery", tasks::battery::run(board.battery));
    spawn(spawner, "telemetry", tasks::telemetry::run());
    spawn(spawner, "status_led", tasks::status_led::run(board.led));
    spawn(spawner, "ble", tasks::ble::run(controller));
    #[cfg(feature = "lidar")]
    spawn(spawner, "lidar", tasks::lidar::run(board.lidar_uart));
    #[cfg(feature = "uplink")]
    tasks::wifi::spawn(spawner, radio.wifi);

    loop {
        Timer::after(Duration::from_secs(30)).await;
        heap::log_stats();
    }
}

/// Spawn a task or say which one could not start (task pool exhausted).
fn spawn<S>(
    spawner: Spawner,
    name: &str,
    token: Result<embassy_executor::SpawnToken<S>, embassy_executor::SpawnError>,
) {
    match token {
        Ok(token) => spawner.spawn(token),
        Err(e) => log::error!("could not spawn task {name}: {e:?}"),
    }
}

/// Park forever after an unrecoverable init error (the log line says why).
async fn halt() -> ! {
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
