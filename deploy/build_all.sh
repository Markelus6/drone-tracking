#!/usr/bin/env bash
# Build orchestrator + trackers for onboard Orange Pi / RK3588 deploy.
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

echo ""
echo "Binaries:"
echo "  $ROOT/orchestrator/build/orch_daemon"
echo "  $ROOT/tracking/build/nanotrack_fc"
echo "  $ROOT/tracking/build/lighttrack_fc"
echo ""
echo "Next: cd $ROOT/deploy && ./start_tracking.sh restart"
