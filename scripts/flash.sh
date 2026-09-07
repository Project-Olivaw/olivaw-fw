#!/usr/bin/env bash
# Build the car firmware, flash it over USB and open the serial monitor.
#   ./scripts/flash.sh                    # Phase 1: BLE car + battery telemetry
#   ./scripts/flash.sh lidar              # + RPLIDAR C1
#   ./scripts/flash.sh lidar,uplink       # + Wi-Fi / MQTT (needs olivaw-car/secrets.toml)
#   PORT=/dev/cu.usbserial-0001 ./scripts/flash.sh   # pick the port explicitly
#   NO_MONITOR=1 ./scripts/flash.sh       # flash and return (CI, scripts)
#
# Safety: never have the L298N 5V -> ESP32 VIN wire and the USB cable connected at the same time.
set -euo pipefail
cd "$(dirname "$0")/../olivaw-car"

if [ -f "$HOME/export-esp.sh" ]; then
  # shellcheck disable=SC1090
  . "$HOME/export-esp.sh"
else
  echo "~/export-esp.sh not found. Run ./scripts/setup.sh first." >&2
  exit 1
fi

FEATURES="${1:-}"

# Pick the port: $PORT, else the first USB serial adapter (CP210x = usbserial, CH340 = wchusbserial).
PORT="${PORT:-$(ls /dev/cu.usbserial* /dev/cu.wchusbserial* /dev/ttyUSB* 2>/dev/null | head -1 || true)}"
if [ -z "$PORT" ]; then
  echo "No USB serial port found. Is the ESP32 plugged in? (ls /dev/cu.*)" >&2
  exit 1
fi

echo "==> building olivaw-car (features: ${FEATURES:-none})"
if [ -n "$FEATURES" ]; then
  cargo build --release --features "$FEATURES"
else
  cargo build --release
fi

ELF=../target/xtensa-esp32-none-elf/release/olivaw-car
echo "==> flashing $ELF to $PORT"
if [ -n "${NO_MONITOR:-}" ]; then
  espflash flash --chip esp32 --port "$PORT" --baud 921600 "$ELF"
else
  espflash flash --chip esp32 --port "$PORT" --baud 921600 --monitor "$ELF"
fi
