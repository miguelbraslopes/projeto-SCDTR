#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

if [[ $# -ne 1 ]]; then
  echo "Uso: $0 rpi1|rpi2"
  exit 1
fi

TARGET="$1"
case "$TARGET" in
  rpi1)
    PIO_ENV="pico_rpi1"
    ;;
  rpi2)
    PIO_ENV="pico_rpi2"
    ;;
  *)
    echo "Alvo invalido: $TARGET"
    echo "Use: rpi1 ou rpi2"
    exit 1
    ;;
esac

if [[ -x "$HOME/.platformio/penv/bin/pio" ]]; then
  PIO_CMD="$HOME/.platformio/penv/bin/pio"
elif command -v pio >/dev/null 2>&1; then
  PIO_CMD="pio"
else
  echo "PlatformIO nao encontrado."
  exit 1
fi

echo "[1/2] Upload para $TARGET ($PIO_ENV)..."
"$PIO_CMD" run -e "$PIO_ENV" -t upload

echo "[2/2] A abrir interface serial de $TARGET..."
exec python3 interface_serial.py --rpi "$TARGET"
