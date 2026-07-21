#!/usr/bin/env bash
# Print pwmchip path for pwm3 (servo) after overlays=...pwm3-m0
set -euo pipefail
echo "pwm chips:"
ls -l /sys/class/pwm/ || true
echo ""
echo "Looking for pwm3 (register name contains pwm3)..."
for c in /sys/class/pwm/pwmchip*; do
  [[ -e "$c" ]] || continue
  t=$(readlink -f "$c" || true)
  echo "  $c -> $t"
  if echo "$t" | grep -qi 'pwm3'; then
    echo "SUGGEST servo_pwmchip=\"$c\" in deploy/chase.json"
  fi
done
echo "Test 50Hz / 1.5ms:"
echo "  echo 0 > \$CHIP/export"
echo "  echo 20000000 > \$CHIP/pwm0/period"
echo "  echo 1500000 > \$CHIP/pwm0/duty_cycle"
echo "  echo 1 > \$CHIP/pwm0/enable"
