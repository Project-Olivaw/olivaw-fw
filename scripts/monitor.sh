#!/usr/bin/env bash
# Open the serial monitor without reflashing (115200, resets the chip first).
#   ./scripts/monitor.sh                 # interactive (Ctrl+C to exit, Ctrl+R to reset)
#   LOG=boot.log ./scripts/monitor.sh    # also record the session to a file (needs a TTY; uses `script`)
set -euo pipefail
# shellcheck disable=SC1090
. "$HOME/export-esp.sh"
PORT="${PORT:-$(ls /dev/cu.usbserial* /dev/cu.wchusbserial* /dev/ttyUSB* 2>/dev/null | head -1 || true)}"
[ -n "$PORT" ] || { echo "No USB serial port found." >&2; exit 1; }
if [ -n "${LOG:-}" ]; then
  # espflash refuses to run its input reader without a TTY; `script` gives it one and keeps a transcript.
  exec script -q "$LOG" espflash monitor -p "$PORT" -B 115200 -c esp32
fi
exec espflash monitor -p "$PORT" -B 115200 -c esp32
