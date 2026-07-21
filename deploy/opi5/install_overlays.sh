#!/usr/bin/env bash
# Enable OPi5 overlays for interceptor (Orangepi OS / image ~1.3.2).
set -euo pipefail

ENV="${ORANGEPI_ENV:-/boot/orangepiEnv.txt}"
NEED="uart1-m1 uart3-m0 pwm3-m0"

if [[ ! -f "$ENV" ]]; then
  echo "ERR: $ENV not found (run on Orange Pi with Orangepi boot)"
  exit 1
fi

if grep -qE '^overlays=' "$ENV"; then
  cur=$(grep -E '^overlays=' "$ENV" | tail -n1 | cut -d= -f2-)
  merged="$cur"
  for o in $NEED; do
    if ! echo " $merged " | grep -q " $o "; then
      merged="$merged $o"
    fi
  done
  sudo sed -i -E "s|^overlays=.*|overlays=${merged}|" "$ENV"
else
  echo "overlays=$NEED" | sudo tee -a "$ENV" >/dev/null
fi

echo "OK: updated $ENV"
grep -E '^overlays=' "$ENV" || true
echo "Reboot required. After boot: ls /dev/ttyS1 /dev/ttyS3"
echo "Optional custom dtbo from interceptor-uart-pwm.dts:"
echo "  dtc -@ -I dts -O dtb -o interceptor-uart-pwm.dtbo interceptor-uart-pwm.dts"
