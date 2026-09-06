# CLAUDE.md — olivaw-fw

> ESP32 firmware for the Olivaw car. `no_std`, esp-hal 1.1 line + Embassy + TrouBLE + esp-radio.
> Part of Project Olivaw. House standards and lessons: `../P-Olivaw-Second-Brain/P-Olivaw-Second-Brain/Home.md`.

## Rules

- **Logic lives in the host crates** (`olivaw-proto`, `olivaw-drive`, `olivaw-battery`, `olivaw-lidar-pump`)
  and is unit-tested on the host. The ESP32 crates only own hardware. Never put a number or a
  decision in a task that could live in `olivaw-car/src/config.rs` or a host crate.
- One task per file under `olivaw-car/src/tasks/`; tasks talk only through `shared.rs`.
- No `unwrap`/`expect`/`panic` outside tests. Errors are logged with what to do.
- Units on the wire are fixed: mV, per-mille, seconds, dBm, lidar Q6/Q2. Convert on the hub.
- Never gate CI on hardware. `cargo test` at the root and `cargo clippy` in `olivaw-car` must be green.
- Version line is **esp-hal 1.1.2 / esp-rtos 0.3 / esp-radio 1.0.0-beta.0 / trouble-host 0.6** — they
  are pinned together; bump them together (see the vault decision note before touching them).

## Commands

```sh
cargo test                                             # host
cd olivaw-car && cargo clippy --features lidar,uplink -- -D warnings
cd olivaw-car && cargo run --release [--features lidar,uplink]   # flash + monitor
```
