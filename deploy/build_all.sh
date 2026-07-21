#!/usr/bin/env bash
# Build orchestrator + trackers + chase/osd for Orange Pi / RK3588.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLEAN=0
for arg in "$@"; do
  [[ "$arg" == "--clean" ]] && CLEAN=1
done

echo "== building orchestrator =="
if [[ "$CLEAN" -eq 1 ]]; then
  (cd "$ROOT/orchestrator" && ./build.sh --clean)
else
  (cd "$ROOT/orchestrator" && ./build.sh)
fi

echo "== building trackers =="
if [[ "$CLEAN" -eq 1 ]]; then
  (cd "$ROOT/tracking" && ./build.sh --clean)
else
  (cd "$ROOT/tracking" && ./build.sh)
fi

echo "== building control (chase_fc + osd_overlay) =="
if [[ "$CLEAN" -eq 1 ]]; then
  rm -rf "$ROOT/control/build"
fi
bash "$ROOT/control/build.sh"

echo ""
echo "Binaries:"
echo "  $ROOT/orchestrator/build/orch_daemon"
echo "  $ROOT/tracking/build/nanotrack_fc"
echo "  $ROOT/tracking/build/lighttrack_fc"
echo "  $ROOT/control/build/chase_fc"
echo "  $ROOT/control/build/osd_overlay"
echo ""
echo "Next: overlays → deploy/opi5/install_overlays.sh"
echo "      cd $ROOT/deploy && DRON_ENABLE_CHASE=1 ./start_tracking.sh restart"
