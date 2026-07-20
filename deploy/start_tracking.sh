#!/usr/bin/env bash
# Start / stop / status for drone-tracking stack on Orange Pi / RK3588.
set -euo pipefail

ACTION="${1:-restart}"
if [[ "${ACTION}" == "-h" || "${ACTION}" == "--help" ]]; then
  cat <<'EOF'
Usage: ./start_tracking.sh [start|stop|restart|status]

Environment:
  DRON_PROJECT_DIR   project root (default: parent of deploy/)
  TRACKER            nano | light  (default: light)
  TRACK_CAMERA       V4L2 device   (default: /dev/cam_usb2)
  TRACK_CAM_W/H/FPS  capture size  (default: 640 480 30)
  VISION_TELEMETRY_PORT  UDP telem (default: 12345)
  STATS_WEB_PORT         HTTP stats (default: 8090)
  DRON_PYTHON_BIN        python for stats_web.py
  DRON_LOG_DIR           log directory
  DRON_SKIP_STATS=1      do not start :8090
EOF
  exit 0
fi

_ts() { date '+%H:%M:%S'; }
_say() { echo "[$(_ts)] $*"; }
_ok() { echo "[$(_ts)] OK $*"; }
_warn() { echo "[$(_ts)] WARN $*" >&2; }
_err() { echo "[$(_ts)] ERR $*" >&2; }

_is_running() { pgrep -f "$1" >/dev/null 2>&1; }
_kill_pat() {
  local pat="$1"
  if _is_running "$pat"; then
    _say "KILL $pat (pids: $(pgrep -f "$pat" | tr '\n' ' '))"
    pkill -TERM -f "$pat" 2>/dev/null || true
    sleep 0.4
    pkill -9 -f "$pat" 2>/dev/null || true
  fi
}
_free_port() {
  local spec="$1"
  if command -v fuser >/dev/null 2>&1; then
    fuser -k -9 "$spec" 2>/dev/null || true
  fi
}

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
PROJECT_DIR="${DRON_PROJECT_DIR:-$(cd "${_SCRIPT_DIR}/.." && pwd)}"
ORCH_BIN="${ORCH_BIN:-$PROJECT_DIR/orchestrator/build/orch_daemon}"
NANO_BIN="${NANO_BIN:-$PROJECT_DIR/tracking/build/nanotrack_fc}"
LIGHT_BIN="${LIGHT_BIN:-$PROJECT_DIR/tracking/build/lighttrack_fc}"
STATS_PY="${STATS_PY:-$_SCRIPT_DIR/stats_web.py}"
PYTHON_BIN="${DRON_PYTHON_BIN:-python3}"
LOG_DIR="${DRON_LOG_DIR:-$_SCRIPT_DIR/logs}"
TRACKER="${TRACKER:-light}"
TRACK_CAMERA="${TRACK_CAMERA:-/dev/cam_usb2}"
TRACK_CAM_W="${TRACK_CAM_W:-640}"
TRACK_CAM_H="${TRACK_CAM_H:-480}"
TRACK_CAM_FPS="${TRACK_CAM_FPS:-30}"
CAM_NAME="${TRACK_CAM_NAME:-front}"

export VISION_TELEMETRY_PORT="${VISION_TELEMETRY_PORT:-12345}"
export STATS_WEB_PORT="${STATS_WEB_PORT:-8090}"
export STATS_WEB_POLL_MS="${STATS_WEB_POLL_MS:-100}"
export STATS_WEB_JSON_CACHE_MS="${STATS_WEB_JSON_CACHE_MS:-250}"
export DRON_LOG_DIR="$LOG_DIR"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/usr/lib:/usr/local/lib:/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu"

mkdir -p "$LOG_DIR"

_tracker_bin() {
  case "${TRACKER}" in
    nano|nanotrack) echo "$NANO_BIN" ;;
    light|lighttrack|*) echo "$LIGHT_BIN" ;;
  esac
}

_tracker_models() {
  case "${TRACKER}" in
    nano|nanotrack) echo "$PROJECT_DIR/tracking/models" ;;
    light|lighttrack|*) echo "$PROJECT_DIR/tracking/models/lighttrack" ;;
  esac
}

_require_bins() {
  local missing=0
  if [[ ! -x "$ORCH_BIN" ]]; then
    _err "Missing $ORCH_BIN — run deploy/build_all.sh"
    missing=1
  fi
  local tb
  tb="$(_tracker_bin)"
  if [[ ! -x "$tb" ]]; then
    _err "Missing $tb — run deploy/build_all.sh"
    missing=1
  fi
  if [[ "${DRON_SKIP_STATS:-0}" != "1" && ! -f "$STATS_PY" ]]; then
    _err "Missing $STATS_PY"
    missing=1
  fi
  [[ "$missing" -eq 0 ]] || exit 1
}

do_stop() {
  _say "Stopping drone-tracking stack"
  _kill_pat "stats_web.py"
  _kill_pat "nanotrack_fc"
  _kill_pat "lighttrack_fc"
  _kill_pat "orch_daemon"
  _free_port "${STATS_WEB_PORT}/tcp"
  _free_port 5003/tcp
  _free_port 5004/tcp
  _free_port 5005/tcp
  _free_port 5006/tcp
  sleep 0.3
  _ok "stopped"
}

do_start() {
  _require_bins
  if [[ ! -e "$TRACK_CAMERA" ]]; then
    _warn "Camera device $TRACK_CAMERA not found — orch may fail"
  fi

  local orch_log="$LOG_DIR/orch_daemon.log"
  local track_log="$LOG_DIR/tracker.log"
  local stats_log="$LOG_DIR/stats_web.log"
  local tb models
  tb="$(_tracker_bin)"
  models="$(_tracker_models)"

  _say "Start orch_daemon → $TRACK_CAMERA ${TRACK_CAM_W}x${TRACK_CAM_H}@${TRACK_CAM_FPS}"
  nohup "$ORCH_BIN" --add "${TRACK_CAMERA},${TRACK_CAM_W},${TRACK_CAM_H},${TRACK_CAM_FPS}" \
    >>"$orch_log" 2>&1 &
  sleep 0.8
  if ! _is_running "orch_daemon"; then
    _err "orch_daemon failed — see $orch_log"
    exit 1
  fi
  _ok "orch_daemon pid $(pgrep -f orch_daemon | head -n1)"

  _say "Start tracker=$TRACKER models=$models"
  nohup "$tb" \
    --camera "$TRACK_CAMERA" \
    --models "$models" \
    --cam-name "$CAM_NAME" \
    --telem-port "$VISION_TELEMETRY_PORT" \
    >>"$track_log" 2>&1 &
  sleep 0.6
  if ! _is_running "nanotrack_fc" && ! _is_running "lighttrack_fc"; then
    _err "tracker failed — see $track_log"
    exit 1
  fi
  _ok "tracker running"

  if [[ "${DRON_SKIP_STATS:-0}" != "1" ]]; then
    _say "Start stats_web :${STATS_WEB_PORT}"
    nohup "$PYTHON_BIN" "$STATS_PY" >>"$stats_log" 2>&1 &
    sleep 0.4
    if command -v curl >/dev/null 2>&1; then
      if curl -fsS --max-time 1 "http://127.0.0.1:${STATS_WEB_PORT}/health" >/dev/null 2>&1; then
        _ok "stats http://0.0.0.0:${STATS_WEB_PORT}/"
      else
        _warn "stats not responding yet — see $stats_log"
      fi
    else
      _ok "stats launched (curl unavailable for health check)"
    fi
  fi

  _say "logs → $LOG_DIR"
  _say "MJPEG / UI / stats:"
  case "${TRACKER}" in
    nano|nanotrack)
      _say "  MJPEG  http://0.0.0.0:5003/"
      _say "  Capture UI http://0.0.0.0:5004/"
      ;;
    *)
      _say "  MJPEG  http://0.0.0.0:5005/"
      _say "  Capture UI http://0.0.0.0:5006/"
      ;;
  esac
  _say "  Stats  http://0.0.0.0:${STATS_WEB_PORT}/"
}

do_status() {
  echo "=== drone-tracking status ==="
  for pat in orch_daemon nanotrack_fc lighttrack_fc stats_web.py; do
    if _is_running "$pat"; then
      echo "  [OK] $pat  pids: $(pgrep -f "$pat" | tr '\n' ' ')"
    else
      echo "  [X]  $pat"
    fi
  done
  echo "--- ports ---"
  if command -v ss >/dev/null 2>&1; then
    ss -lntup 2>/dev/null | grep -E ':(5003|5004|5005|5006|8090|12345|12347|12349)\b' || true
  fi
  if command -v curl >/dev/null 2>&1; then
    curl -fsS --max-time 1 "http://127.0.0.1:${STATS_WEB_PORT}/stats.json" >/dev/null 2>&1 \
      && echo "  [OK] stats :${STATS_WEB_PORT}" \
      || echo "  [X]  stats :${STATS_WEB_PORT}"
  fi
  echo "PROJECT_DIR=$PROJECT_DIR"
  echo "TRACKER=$TRACKER CAMERA=$TRACK_CAMERA"
  echo "LOG_DIR=$LOG_DIR"
}

case "$ACTION" in
  start) do_start ;;
  stop) do_stop ;;
  restart) do_stop; do_start ;;
  status) do_status ;;
  *)
    _err "Unknown action: $ACTION (use start|stop|restart|status)"
    exit 1
    ;;
esac
