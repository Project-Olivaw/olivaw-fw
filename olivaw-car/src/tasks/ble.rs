//! BLE GATT peripheral (`TrouBLE` on esp-radio).
//!
//! Services:
//! - Olivaw Car `12345678-1234-5678-1234-56789abcdef0`
//!   - `…def1` drive, write / write-without-response: `"<left>,<right>"` per-mille
//!   - `…def2` telemetry, read / notify: `olivaw_proto::Telemetry` (19 bytes)
//!   - `…def3` control, write / write-without-response: `olivaw_proto::Control`
//! - Battery Service `0x180F` with Battery Level `0x2A19`, read / notify
//!
//! One connection at a time. On disconnect the drive task is told
//! immediately (`LinkLost`) instead of waiting for the watchdog.

// The TrouBLE derive macros borrow their field types; nothing to fix here.
#![allow(clippy::needless_borrows_for_generic_args)]

use embassy_futures::join::join;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};
use esp_radio::ble::controller::BleConnector;
use olivaw_proto::{Control, Flags, TELEMETRY_LEN, Telemetry};
use trouble_host::prelude::*;

use crate::config;
use crate::shared::{DRIVE_CMD, DRIVE_EVENTS, DriveEvent, TELEMETRY, with_state};

/// The HCI controller: esp-radio's BLE connector behind bt-hci.
pub type Controller = ExternalController<BleConnector<'static>, 1>;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2;

/// Longest drive frame accepted (`"-1000,-1000\r\n"` is 13 bytes).
const DRIVE_MAX: usize = 20;

#[gatt_server]
struct Server {
    car: CarService,
    battery: BatteryService,
}

/// Olivaw car control + telemetry.
#[gatt_service(uuid = "12345678-1234-5678-1234-56789abcdef0")]
struct CarService {
    /// ASCII `"<left>,<right>"` per-mille, optional trailing newline.
    #[characteristic(
        uuid = "12345678-1234-5678-1234-56789abcdef1",
        write,
        write_without_response
    )]
    drive: [u8; DRIVE_MAX],
    /// Packed `Telemetry` v1.
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef2", read, notify)]
    telemetry: [u8; TELEMETRY_LEN],
    /// `Control` opcode + payload.
    #[characteristic(
        uuid = "12345678-1234-5678-1234-56789abcdef3",
        write,
        write_without_response
    )]
    control: [u8; 3],
}

/// Standard Battery Service so the OS shows the pack level too.
#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    /// Percent `0..=100`.
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 0)]
    level: u8,
}

/// Runs the BLE host forever: advertise, accept one central, serve, repeat.
#[embassy_executor::task]
pub async fn run(controller: Controller) {
    let address = Address::random(config::BLE_ADDRESS);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    let server = match Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: config::BLE_NAME,
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    })) {
        Ok(server) => server,
        Err(e) => {
            log::error!("ble: cannot build GATT server: {e}");
            return;
        }
    };

    let _ = join(host_runner(runner), async {
        loop {
            match advertise(&mut peripheral).await {
                Ok(conn) => match conn.with_attribute_server(&server) {
                    Ok(gatt) => serve(&server, &gatt, &stack).await,
                    Err(e) => log::error!("ble: attribute server: {e:?}"),
                },
                Err(e) => {
                    log::warn!("ble: advertise failed: {e:?}");
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
        }
    })
    .await;
}

async fn host_runner<C: trouble_host::Controller>(mut runner: Runner<'_, C, DefaultPacketPool>) {
    if let Err(e) = runner.run().await {
        log::error!("ble: host runner exited: {e:?}");
    }
}

async fn advertise<'a, C: trouble_host::Controller>(
    peripheral: &mut Peripheral<'a, C, DefaultPacketPool>,
) -> Result<Connection<'a, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0u8; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(config::BLE_NAME.as_bytes()),
        ],
        &mut adv_data,
    )?;
    let mut scan_data = [0u8; 31];
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::ServiceUuids128(&[CAR_SERVICE_UUID_BYTES])],
        &mut scan_data,
    )?;
    log::info!("ble: advertising as {}", config::BLE_NAME);
    let advertiser = peripheral
        .advertise(
            &AdvertisementParameters::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &scan_data[..scan_len],
            },
        )
        .await?;
    let conn = advertiser.accept().await?;
    log::info!("ble: connected");
    Ok(conn)
}

/// The car service UUID as little-endian bytes for the scan response.
const CAR_SERVICE_UUID_BYTES: [u8; 16] = [
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12,
];

async fn serve<C: trouble_host::Controller>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    stack: &Stack<'_, C, DefaultPacketPool>,
) {
    with_state(|s| s.flags.insert(Flags::BLE_CONNECTED));
    match select(gatt_loop(server, conn), notify_loop(server, conn, stack)).await {
        Either::First(()) => log::info!("ble: disconnected"),
        Either::Second(()) => log::warn!("ble: notify path failed, dropping connection"),
    }
    with_state(|s| {
        s.flags.remove(Flags::BLE_CONNECTED);
        s.rssi_dbm = 0;
    });
    if DRIVE_EVENTS.try_send(DriveEvent::LinkLost).is_err() {
        log::error!("drive event queue full; LinkLost not delivered (watchdog will stop)");
    }
}

/// Handles writes until the central disconnects.
async fn gatt_loop(server: &Server<'_>, conn: &GattConnection<'_, '_, DefaultPacketPool>) {
    let drive_handle = server.car.drive.handle;
    let control_handle = server.car.control.handle;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                log::info!("ble: disconnect reason {reason:?}");
                return;
            }
            GattConnectionEvent::Gatt { event } => {
                if let GattEvent::Write(w) = &event {
                    let handle = w.handle();
                    if handle == drive_handle {
                        match olivaw_drive::parse_frame(w.data()) {
                            Ok(cmd) => DRIVE_CMD.signal(cmd),
                            Err(e) => log::warn!("ble: bad drive frame: {e}"),
                        }
                    } else if handle == control_handle {
                        match Control::decode(w.data()) {
                            Ok(cmd) => crate::control::dispatch(cmd),
                            Err(e) => log::warn!("ble: bad control frame: {e}"),
                        }
                    }
                }
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => log::warn!("ble: event accept: {e:?}"),
                }
            }
            _ => {}
        }
    }
}

/// Pushes telemetry notifications; returns when a notify fails.
async fn notify_loop<C: trouble_host::Controller>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    stack: &Stack<'_, C, DefaultPacketPool>,
) {
    let Some(mut rx) = TELEMETRY.receiver() else {
        log::error!("ble: no telemetry receiver slot");
        return;
    };
    let mut tick: u32 = 0;
    loop {
        let t: Telemetry = rx.changed().await;
        tick = tick.wrapping_add(1);

        if tick.is_multiple_of(config::RSSI_EVERY_TICKS)
            && let Ok(rssi) = conn.raw().rssi(stack).await
        {
            with_state(|s| s.rssi_dbm = rssi);
        }
        if server
            .car
            .telemetry
            .notify(conn, &t.encode())
            .await
            .is_err()
        {
            return;
        }
        if tick.is_multiple_of(config::BATTERY_LEVEL_EVERY_TICKS)
            && server
                .battery
                .level
                .notify(conn, &t.battery_pct)
                .await
                .is_err()
        {
            return;
        }
    }
}
