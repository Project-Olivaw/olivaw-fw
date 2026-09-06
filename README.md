# olivaw-fw

Firmware for the Olivaw car: an ESP32-WROOM-32D driving two DC motors through an L298N,
controlled over Bluetooth LE from the [olivaw-pilot](https://github.com/Project-Olivaw/olivaw-pilot)
app, with battery telemetry and — behind cargo features — an RPLIDAR C1 stream and a Wi-Fi/MQTT
uplink to [olivaw-hub](https://github.com/Project-Olivaw/olivaw-hub).

Pure Rust, `no_std`, [esp-hal](https://github.com/esp-rs/esp-hal) + Embassy + [TrouBLE](https://github.com/embassy-rs/trouble).
Part of [Project Olivaw](https://github.com/Project-Olivaw).

## Layout

| Crate | Runs on | What |
| --- | --- | --- |
| `olivaw-proto` | host + car | Wire types: BLE telemetry/control frames, MQTT scan frames and topics |
| `olivaw-drive` | host + car | Drive pipeline: `"<left>,<right>"` frames, 500 ms watchdog, E-stop, duty cap, slew, L298N |
| `olivaw-battery` | host + car | Divider maths, ADC model, LiPo state of charge, low-battery hysteresis |
| `olivaw-lidar-pump` | host + car | Streaming RPLIDAR parser over `olivaw-lidar`'s protocol core |
| `olivaw-board-esp32` | car | The pin map and `Board::init()`: motors, battery ADC, LED, lidar UART |
| `olivaw-car` | car | The binary: one Embassy task per concern (`src/tasks/`) |

The four host crates are the ones with logic; the two ESP32 crates only wire hardware to them.

## One-time setup

```sh
cargo install espup espflash --locked
espup install                       # Xtensa Rust toolchain, channel "esp"
. ~/export-esp.sh                   # every new shell (or add to ~/.zshrc)
```

## Build, flash, monitor

```sh
cd olivaw-car
cargo run --release                             # Phase 1: BLE car + battery telemetry
cargo run --release --features lidar            # + RPLIDAR C1 on UART1
cp secrets.example.toml secrets.toml            # Wi-Fi + broker credentials (gitignored)
cargo run --release --features lidar,uplink     # + Wi-Fi/MQTT uplink to olivaw-hub
```

`cargo run` flashes over USB and opens the serial monitor (`ESP_LOG=info`).

## Verify (no hardware)

```sh
cargo test                                      # host crates
cargo clippy --all-targets -- -D warnings
cd olivaw-car && cargo clippy --features lidar,uplink -- -D warnings && cargo build --release --features lidar,uplink
```

## Protocol

BLE peripheral `OLIVAW-CAR`, service `12345678-1234-5678-1234-56789abcdef0`:

| Char | UUID suffix | Props | Payload |
| --- | --- | --- | --- |
| drive | `…def1` | write, write w/o response | ASCII `"<left>,<right>"` per-mille, `-1000..=1000` |
| telemetry | `…def2` | read, notify (5 Hz) | 19-byte packed `Telemetry` (`olivaw-proto`) |
| control | `…def3` | write | `Control` opcode: E-stop, clear, lidar on/off, uplink on/off, set max duty |

Plus the standard Battery Service (`0x180F` / `0x2A19`). MQTT topics: `olivaw/<car>/{status,telemetry,scan,cmd}`.

## Wiring and the rest

Pin map: `olivaw-board-esp32/src/pins.rs`. Power, wiring diagram, bring-up checklist and the
lessons learned live in the project vault (`P-Olivaw-Second-Brain`).
