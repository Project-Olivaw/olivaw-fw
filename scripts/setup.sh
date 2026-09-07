#!/usr/bin/env bash
# One-time toolchain setup for the ESP32 firmware (macOS / Linux).
#   ./scripts/setup.sh
# Installs espup (Xtensa Rust toolchain manager), the "esp" toolchain, and espflash.
set -euo pipefail

if ! command -v cargo >/dev/null; then
  echo "Rust is missing. Install it first: https://rustup.rs" >&2
  exit 1
fi

cargo install espup --locked
cargo install espflash --locked
espup install

echo
echo "Done. The Xtensa toolchain is the 'esp' channel; every shell that builds firmware must run:"
echo "    . ~/export-esp.sh"
echo "Add that line to ~/.zshrc if you want it permanent. Then: ./scripts/flash.sh"
