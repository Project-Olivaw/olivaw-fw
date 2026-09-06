//! Wi-Fi station + embassy-net stack, brought up at boot when the `uplink`
//! feature is on and an SSID was compiled in. Reconnects forever.

use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, Runner, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::WIFI;
use esp_hal::rng::Rng;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{Config, ControllerConfig, Interface, WifiController};
use olivaw_proto::LinkState;
use static_cell::StaticCell;

use crate::shared::with_state;
use crate::{config, secrets};

static STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();

/// Build the controller and the network stack, then spawn the three uplink
/// tasks. Skipped with a log line when no SSID was compiled in.
pub fn spawn(spawner: Spawner, wifi: WIFI<'static>) {
    if secrets::WIFI_SSID.is_empty() {
        log::warn!("uplink: no WIFI_SSID in secrets.toml — Wi-Fi stays off");
        return;
    }
    let station = StationConfig::default()
        .with_ssid(secrets::WIFI_SSID)
        .with_password(secrets::WIFI_PASS.into());
    let controller = match WifiController::new(
        wifi,
        ControllerConfig::default().with_initial_config(Config::Station(station)),
    ) {
        Ok(c) => c,
        Err(e) => {
            log::error!("uplink: Wi-Fi controller init failed: {e:?}");
            with_state(|s| s.wifi = LinkState::Fault);
            return;
        }
    };
    let interface = Interface::station();

    let rng = Rng::new();
    let seed = (u64::from(rng.random()) << 32) | u64::from(rng.random());
    let (stack, runner) = embassy_net::new(
        interface,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        STACK_RESOURCES.init(StackResources::new()),
        seed,
    );

    spawn_or_log(spawner, "net", net_task(runner));
    spawn_or_log(spawner, "wifi", connection_task(controller));
    spawn_or_log(spawner, "mqtt", crate::tasks::mqtt::run(stack));
}

fn spawn_or_log<S>(
    spawner: Spawner,
    name: &str,
    token: Result<embassy_executor::SpawnToken<S>, embassy_executor::SpawnError>,
) {
    match token {
        Ok(token) => spawner.spawn(token),
        Err(e) => log::error!("uplink: could not spawn {name}: {e:?}"),
    }
}

/// Drives the smoltcp stack.
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await;
}

/// Connect, wait for the drop, back off, repeat.
#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>) {
    loop {
        with_state(|s| s.wifi = LinkState::Starting);
        log::info!("uplink: connecting to {}", secrets::WIFI_SSID);
        match controller.connect_async().await {
            Ok(info) => {
                log::info!("uplink: Wi-Fi connected: {info:?}");
                with_state(|s| s.wifi = LinkState::Up);
                let dropped = controller.wait_for_disconnect_async().await;
                log::warn!("uplink: Wi-Fi disconnected: {dropped:?}");
            }
            Err(e) => {
                log::warn!("uplink: Wi-Fi connect failed: {e:?}");
                with_state(|s| s.wifi = LinkState::Fault);
                Timer::after(Duration::from_secs(config::NET_RETRY_S)).await;
            }
        }
    }
}
