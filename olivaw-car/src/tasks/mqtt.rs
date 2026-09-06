//! MQTT publisher over the embassy-net stack.
//!
//! Publishes `olivaw/<car>/status` (retained, with an `offline` last will),
//! `…/telemetry` as JSON at 1 Hz, and `…/scan` as postcard per rotation;
//! subscribes to `…/cmd` for control opcodes. Reconnects with back-off.

use core::num::NonZero;
use core::str::FromStr;

use embassy_futures::select::{Either4, select4};
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_net::{IpAddress, Ipv4Address, Stack};
use embassy_time::{Duration, Timer};
use olivaw_proto::topics::{self, Channel as Topic};
use olivaw_proto::{Control, LinkState, ScanFrame, Telemetry};
use rust_mqtt::Bytes;
use rust_mqtt::buffer::AllocBuffer;
use rust_mqtt::client::Client;
use rust_mqtt::client::event::Event;
use rust_mqtt::client::options::{
    ConnectOptions, PublicationOptions, SubscriptionOptions, TopicReference, WillOptions,
};
use rust_mqtt::config::KeepAlive;
use rust_mqtt::types::{MqttBinary, MqttString, TopicFilter, TopicName};
use static_cell::StaticCell;

use crate::shared::{DRIVE_CMD, TELEMETRY, UPLINK_CTL, with_state};
use crate::{config, secrets};

static RX_BUF: StaticCell<[u8; 4096]> = StaticCell::new();
static TX_BUF: StaticCell<[u8; 4096]> = StaticCell::new();
static SCAN_BUF: StaticCell<[u8; ScanFrame::MAX_ENCODED_LEN]> = StaticCell::new();

type MqttClient<'c, N> = Client<'c, N, AllocBuffer, 1, 4, 4, 0>;

/// Keeps a session open for as long as the network allows.
#[embassy_executor::task]
pub async fn run(stack: Stack<'static>) {
    let rx_buf = RX_BUF.init([0; 4096]);
    let tx_buf = TX_BUF.init([0; 4096]);
    let scan_buf = SCAN_BUF.init([0; ScanFrame::MAX_ENCODED_LEN]);
    let port = u16::from_str(secrets::MQTT_PORT).unwrap_or(1883);
    let mut publishing = true;

    loop {
        with_state(|s| s.mqtt = LinkState::Off);
        stack.wait_config_up().await;
        with_state(|s| s.mqtt = LinkState::Starting);

        let Some(ip) = resolve(stack, secrets::MQTT_HOST).await else {
            log::warn!("mqtt: cannot resolve {:?}", secrets::MQTT_HOST);
            with_state(|s| s.mqtt = LinkState::Fault);
            Timer::after(Duration::from_secs(config::NET_RETRY_S)).await;
            continue;
        };

        let mut socket = TcpSocket::new(stack, &mut rx_buf[..], &mut tx_buf[..]);
        socket.set_timeout(Some(Duration::from_secs(
            u64::from(config::MQTT_KEEP_ALIVE_S) * 2,
        )));
        if let Err(e) = socket.connect((ip, port)).await {
            log::warn!("mqtt: tcp connect to {ip}:{port} failed: {e:?}");
            with_state(|s| s.mqtt = LinkState::Fault);
            Timer::after(Duration::from_secs(config::NET_RETRY_S)).await;
            continue;
        }

        let mut buffer = AllocBuffer;
        let mut client: MqttClient<'_, _> = Client::new(&mut buffer);
        match session(&mut client, socket, scan_buf, &mut publishing).await {
            Ok(()) => log::info!("mqtt: session closed"),
            Err(msg) => log::warn!("mqtt: {msg}"),
        }
        with_state(|s| s.mqtt = LinkState::Fault);
        Timer::after(Duration::from_secs(config::NET_RETRY_S)).await;
    }
}

async fn resolve(stack: Stack<'static>, host: &str) -> Option<IpAddress> {
    if let Ok(v4) = Ipv4Address::from_str(host) {
        return Some(IpAddress::Ipv4(v4));
    }
    stack
        .dns_query(host, DnsQueryType::A)
        .await
        .ok()?
        .first()
        .copied()
}

async fn session<N: rust_mqtt::io::Transport>(
    client: &mut MqttClient<'_, N>,
    socket: N,
    scan_buf: &mut [u8],
    publishing: &mut bool,
) -> Result<(), &'static str> {
    let status_topic = topic(Topic::Status)?;
    let telemetry_topic = topic(Topic::Telemetry)?;
    let scan_topic = topic(Topic::Scan)?;
    let cmd_topic = topic(Topic::Cmd)?;

    let mut will = WillOptions::new(name(&status_topic)?, MqttBinary::from(offline_msg()));
    will.will_retain = true;
    let keep_alive =
        NonZero::new(config::MQTT_KEEP_ALIVE_S).map_or(KeepAlive::Infinite, KeepAlive::Seconds);
    let mut options = ConnectOptions::new().clean_start().keep_alive(keep_alive);
    options.will = Some(will);
    if !secrets::MQTT_USER.is_empty() {
        options.user_name =
            Some(MqttString::try_from(secrets::MQTT_USER).map_err(|_| "bad MQTT_USER")?);
        options.password = Some(
            MqttString::try_from(secrets::MQTT_PASS)
                .map_err(|_| "bad MQTT_PASS")?
                .into(),
        );
    }
    let client_id = MqttString::try_from(config::CAR_ID).map_err(|_| "bad CAR_ID")?;
    client
        .connect(socket, &options, Some(client_id))
        .await
        .map_err(|_| "CONNECT failed")?;
    with_state(|s| s.mqtt = LinkState::Up);
    log::info!("mqtt: connected as {}", config::CAR_ID);

    let mut online = PublicationOptions::new(TopicReference::Name(name(&status_topic)?));
    online.retain = true;
    client
        .publish(&online, Bytes::from(topics::STATUS_ONLINE.as_bytes()))
        .await
        .map_err(|_| "publish status failed")?;
    let filter =
        TopicFilter::new(MqttString::try_from(cmd_topic.as_str()).map_err(|_| "bad cmd topic")?)
            .ok_or("bad cmd filter")?;
    client
        .subscribe(filter, SubscriptionOptions::default())
        .await
        .map_err(|_| "SUBSCRIBE failed")?;

    let mut telemetry_rx = TELEMETRY.receiver().ok_or("no telemetry receiver slot")?;
    let mut ticks: u32 = 0;
    let ping_every = Duration::from_secs(u64::from(config::MQTT_KEEP_ALIVE_S / 2).max(5));

    loop {
        match select4(
            client.poll_header(),
            telemetry_rx.changed(),
            crate::tasks::mqtt::next_scan(),
            Timer::after(ping_every),
        )
        .await
        {
            Either4::First(header) => {
                let header = header.map_err(|_| "poll header failed")?;
                if let Event::Publish(publish) = client
                    .poll_body(header)
                    .await
                    .map_err(|_| "poll body failed")?
                {
                    handle_command(publish.message.as_ref());
                }
            }
            Either4::Second(t) => {
                if let Some(on) = UPLINK_CTL.try_take() {
                    *publishing = on;
                    log::info!("mqtt: publishing {}", if on { "on" } else { "paused" });
                }
                ticks = ticks.wrapping_add(1);
                if *publishing && ticks.is_multiple_of(config::MQTT_TELEMETRY_EVERY_TICKS) {
                    publish_telemetry(client, &telemetry_topic, &t).await?;
                }
            }
            Either4::Third(frame) => {
                if *publishing {
                    let bytes = frame.encode(scan_buf).map_err(|_| "scan encode failed")?;
                    let opts = PublicationOptions::new(TopicReference::Name(name(&scan_topic)?));
                    client
                        .publish(&opts, Bytes::from(bytes))
                        .await
                        .map_err(|_| "publish scan failed")?;
                }
            }
            Either4::Fourth(()) => client.ping().await.map_err(|_| "PING failed")?,
        }
    }
}

async fn publish_telemetry<N: rust_mqtt::io::Transport>(
    client: &mut MqttClient<'_, N>,
    topic: &heapless::String<{ topics::TOPIC_MAX_LEN }>,
    t: &Telemetry,
) -> Result<(), &'static str> {
    let mut json = [0u8; 256];
    let len = serde_json_core::to_slice(t, &mut json).map_err(|_| "telemetry json too long")?;
    let opts = PublicationOptions::new(TopicReference::Name(name(topic)?));
    client
        .publish(&opts, Bytes::from(&json[..len]))
        .await
        .map_err(|_| "publish telemetry failed")?;
    Ok(())
}

fn offline_msg() -> MqttString<'static> {
    // The literal is valid UTF-8 and short; the fallback keeps this infallible.
    MqttString::try_from(topics::STATUS_OFFLINE)
        .unwrap_or_else(|_| MqttString::try_from("offline").unwrap_or_default())
}

/// Next rotation from the lidar, or pend forever without the feature.
async fn next_scan() -> ScanFrame {
    #[cfg(feature = "lidar")]
    {
        crate::shared::SCANS.receive().await
    }
    #[cfg(not(feature = "lidar"))]
    {
        core::future::pending().await
    }
}

/// `cmd` payloads are either a control opcode frame or a drive frame.
fn handle_command(payload: &[u8]) {
    if let Ok(cmd) = olivaw_drive::parse_frame(payload) {
        DRIVE_CMD.signal(cmd);
    } else if let Ok(control) = Control::decode(payload) {
        crate::control::dispatch(control);
    } else {
        log::warn!("mqtt: unrecognised cmd payload ({} bytes)", payload.len());
    }
}

fn topic(channel: Topic) -> Result<heapless::String<{ topics::TOPIC_MAX_LEN }>, &'static str> {
    topics::topic(config::CAR_ID, channel).ok_or("topic too long")
}

fn name(
    topic: &heapless::String<{ topics::TOPIC_MAX_LEN }>,
) -> Result<TopicName<'_>, &'static str> {
    TopicName::new(MqttString::try_from(topic.as_str()).map_err(|_| "bad topic string")?)
        .ok_or("bad topic name")
}
