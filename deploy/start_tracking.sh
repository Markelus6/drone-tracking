#!/usr/bin/env bash
# Start / stop / status for drone-tracking + Betaflight MSP + optional chase/OSD.
set -euo pipefail

ACTION="${1:-restart}"
if [[ "${ACTION}" == "-h" || "${ACTION}" == "--help" ]]; then
  cat <<'EOF'
Usage: ./start_tracking.sh [start|stop|restart|status]

Environment:
  DRON_PROJECT_DIR   project root (default: parent of deploy/)
  TRACKER            nano | nanov3 | light | csrt | kcf | mosse | mil | medianflow | tld
                     | dasiamrpn | goturn | cftrack | tctrack
                     | vit | ostrack | mixformer  (need OpenCV>=4.8)
                     | tctrack | mixformer | ostrack | cftrack
                     default: light
  TRACK_CAMERA       V4L2 device   (default: /dev/cam_usb2)
  TRACK_CAM_W/H/FPS  capture size  (default: 640 480 30)
  VISION_TELEMETRY_PORT  UDP telem (default: 12345) — owned by msp_betaflight / chase
  STATS_WEB_PORT         HTTP stats (default: 8090)
  BF_MSP_ENABLE=1        start Betaflight MSP companion (default 1)
  BF_MSP_PORT            UART to BF (default /dev/ttyS4)
  BF_GUIDANCE_ENABLE=1   send tracking sticks (default 0 = mid / safe)
  BF_ARM_ENABLE=1        raise AUX arm when guiding (default 0)
  BF_DRY_RUN=1           no UART writes
  DRON_ENABLE_CHASE=1    start chase_fc (CRSF→MSP + engage)
  DRON_ENABLE_OSD=1      start osd_overlay → OSD_SINK
  OSD_SINK               V4L2 out e.g. /dev/video10 (USB CVBS)
  CHASE_CONFIG           default: deploy/chase.json
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
MULTI_BIN="${MULTI_BIN:-$PROJECT_DIR/tracking/build/multitrack_fc}"
CHASE_BIN="${CHASE_BIN:-$PROJECT_DIR/control/build/chase_fc}"
OSD_BIN="${OSD_BIN:-$PROJECT_DIR/control/build/osd_overlay}"
STATS_PY="${STATS_PY:-$_SCRIPT_DIR/stats_web.py}"
MSP_PY="${MSP_PY:-$_SCRIPT_DIR/msp_betaflight.py}"
PYTHON_BIN="${DRON_PYTHON_BIN:-python3}"
LOG_DIR="${DRON_LOG_DIR:-$_SCRIPT_DIR/logs}"
TRACKER="${TRACKER:-light}"
TRACK_CAMERA="${TRACK_CAMERA:-/dev/cam_usb2}"
TRACK_CAM_W="${TRACK_CAM_W:-640}"
TRACK_CAM_H="${TRACK_CAM_H:-480}"
TRACK_CAM_FPS="${TRACK_CAM_FPS:-30}"
CAM_NAME="${TRACK_CAM_NAME:-front}"
CHASE_CONFIG="${CHASE_CONFIG:-$_SCRIPT_DIR/chase.json}"
OSD_SINK="${OSD_SINK:-}"
DRON_ENABLE_CHASE="${DRON_ENABLE_CHASE:-0}"
DRON_ENABLE_OSD="${DRON_ENABLE_OSD:-0}"

export VISION_TELEMETRY_PORT="${VISION_TELEMETRY_PORT:-12345}"
# When chase is on, it binds :12345 and forwards to :12346 for stats/osd
TELEM_LISTEN_FOR_STATS="$VISION_TELEMETRY_PORT"
if [[ "$DRON_ENABLE_CHASE" == "1" ]]; then
  TELEM_LISTEN_FOR_STATS=$((VISION_TELEMETRY_PORT + 1))
fi
export STATS_WEB_PORT="${STATS_WEB_PORT:-8090}"
export STATS_WEB_POLL_MS="${STATS_WEB_POLL_MS:-100}"
export STATS_WEB_JSON_CACHE_MS="${STATS_WEB_JSON_CACHE_MS:-250}"
export VISION_MJPEG_PERIOD_MS="${VISION_MJPEG_PERIOD_MS:-33}"
export VISION_MJPEG_QUALITY="${VISION_MJPEG_QUALITY:-40}"
export VISION_STREAM_MAX_W="${VISION_STREAM_MAX_W:-480}"
# Betaflight MSP (not MAVLink)
export BF_MSP_ENABLE="${BF_MSP_ENABLE:-1}"
export BF_MSP_PORT="${BF_MSP_PORT:-/dev/ttyS4}"
export BF_MSP_BAUD="${BF_MSP_BAUD:-115200}"
export BF_MSP_HZ="${BF_MSP_HZ:-50}"
export BF_GUIDANCE_ENABLE="${BF_GUIDANCE_ENABLE:-0}"
export BF_ARM_ENABLE="${BF_ARM_ENABLE:-0}"
export BF_MAP="${BF_MAP:-aetr}"
export BF_TELEM_FILE="${BF_TELEM_FILE:-/dev/shm/drone_telem.json}"
# When MSP owns UDP telem, stats reads the shared JSON file only
if [[ "${BF_MSP_ENABLE}" == "1" ]]; then
  export STATS_SKIP_UDP="${STATS_SKIP_UDP:-1}"
else
  export STATS_SKIP_UDP="${STATS_SKIP_UDP:-0}"
fi
export DRON_LOG_DIR="$LOG_DIR"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/usr/lib:/usr/local/lib:/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu"

mkdir -p "$LOG_DIR"

_tracker_bin() {
  case "${TRACKER}" in
    nano|nanotrack) echo "$NANO_BIN" ;;
    light|lighttrack) echo "$LIGHT_BIN" ;;
    *) echo "$MULTI_BIN" ;;
  esac
}

_tracker_models() {
  case "${TRACKER}" in
    nano|nanotrack) echo "$PROJECT_DIR/tracking/models" ;;
    light|lighttrack) echo "$PROJECT_DIR/tracking/models/lighttrack" ;;
    nanov3|nano_v3|v3) echo "$PROJECT_DIR/tracking/models/nanotrackv3" ;;
    tctrack|tctrack++|tctrackpp|tctracklite) echo "$PROJECT_DIR/tracking/models/nanotrackv3" ;;
    dasiamrpn|dasiam|siamrpn) echo "$PROJECT_DIR/tracking/models/dasiamrpn" ;;
    goturn) echo "$PROJECT_DIR/tracking/models/goturn" ;;
    vit|vittrack|ostrack|ostrack256|mixformer|mixformerv2|mixformerv2s)
      echo "$PROJECT_DIR/tracking/models/vittrack" ;;
    csrt|kcf|mosse|mil|medianflow|tld|mf|cftrack|cf|cfensemble) echo "$PROJECT_DIR/tracking/models" ;;
    *) echo "$PROJECT_DIR/tracking/models" ;;
  esac
}

_tracker_extra_args() {
  case "${TRACKER}" in
    nano|nanotrack|light|lighttrack) echo "" ;;
    *) echo "--backend ${TRACKER}" ;;
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
  if [[ "${BF_MSP_ENABLE}" == "1" && ! -f "$MSP_PY" ]]; then
    _err "Missing $MSP_PY"
    missing=1
  fi
  if [[ "${DRON_SKIP_STATS:-0}" != "1" && ! -f "$STATS_PY" ]]; then
    _err "Missing $STATS_PY"
    missing=1
  fi
  if [[ "$DRON_ENABLE_CHASE" == "1" && ! -x "$CHASE_BIN" ]]; then
    _err "Missing $CHASE_BIN — build control/"
    missing=1
  fi
  if [[ "$DRON_ENABLE_OSD" == "1" && ! -x "$OSD_BIN" ]]; then
    _err "Missing $OSD_BIN — build control/"
    missing=1
  fi
  [[ "$missing" -eq 0 ]] || exit 1
}

do_stop() {
  _say "Stopping drone-tracking stack"
  _kill_pat "osd_overlay"
  _kill_pat "chase_fc"
  _kill_pat "stats_web.py"
  _kill_pat "msp_betaflight.py"
  _kill_pat "nanotrack_fc"
  _kill_pat "lighttrack_fc"
  _kill_pat "multitrack_fc"
  _kill_pat "orch_daemon"
  _free_port "${STATS_WEB_PORT}/tcp"
  for p in 5003 5004 5005 5006 5007 5008 \
           5010 5011 5012 5013 5014 5015 5016 5017 5018 5019 5020 5021 \
           5022 5023 5024 5025 5026 5027 5028 5029 5030 5031 5032 5033; do
    _free_port "${p}/tcp"
  done
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
  local msp_log="$LOG_DIR/msp_betaflight.log"
  local stats_log="$LOG_DIR/stats_web.log"
  local chase_log="$LOG_DIR/chase.log"
  local osd_log="$LOG_DIR/osd_overlay.log"
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
  # shellcheck disable=SC2086
  nohup "$tb" \
    --camera "$TRACK_CAMERA" \
    --models "$models" \
    --cam-name "$CAM_NAME" \
    --telem-port "$VISION_TELEMETRY_PORT" \
    $(_tracker_extra_args) \
    >>"$track_log" 2>&1 &
  sleep 0.8
  if ! _is_running "nanotrack_fc" && ! _is_running "lighttrack_fc" && ! _is_running "multitrack_fc"; then
    _err "tracker failed — see $track_log"
    exit 1
  fi
  _ok "tracker running"

  if [[ "${BF_MSP_ENABLE}" == "1" ]]; then
    _say "Start msp_betaflight → ${BF_MSP_PORT} @ ${BF_MSP_BAUD} (guidance=${BF_GUIDANCE_ENABLE})"
    if ! "$PYTHON_BIN" -c "import serial" >/dev/null 2>&1; then
      _warn "pyserial missing — MSP will dry-run (pip3 install pyserial)"
    fi
    nohup "$PYTHON_BIN" "$MSP_PY" >>"$msp_log" 2>&1 &
    sleep 0.4
    if _is_running "msp_betaflight.py"; then
      _ok "msp_betaflight pid $(pgrep -f msp_betaflight.py | head -n1)"
    else
      _warn "msp_betaflight failed — see $msp_log"
    fi
  else
    _say "BF_MSP_ENABLE=0 — skip Betaflight MSP"
  fi

  if [[ "$DRON_ENABLE_CHASE" == "1" ]]; then
    _say "Start chase_fc config=$CHASE_CONFIG"
    nohup "$CHASE_BIN" --config "$CHASE_CONFIG" >>"$chase_log" 2>&1 &
    sleep 0.5
    if _is_running "chase_fc"; then
      _ok "chase_fc pid $(pgrep -f chase_fc | head -n1)"
    else
      _warn "chase_fc failed — see $chase_log"
    fi
  fi

  if [[ "$DRON_ENABLE_OSD" == "1" ]]; then
    local osd_telem="$VISION_TELEMETRY_PORT"
    if [[ "$DRON_ENABLE_CHASE" == "1" ]]; then
      osd_telem=$((VISION_TELEMETRY_PORT + 5))
    fi
    local osd_args=(--camera "$TRACK_CAMERA" --telem-port "$osd_telem" --fps "$TRACK_CAM_FPS")
    if [[ -n "$OSD_SINK" ]]; then
      osd_args+=(--sink "$OSD_SINK")
    else
      _warn "DRON_ENABLE_OSD=1 but OSD_SINK empty — overlay runs without V4L2 out"
    fi
    _say "Start osd_overlay ${osd_args[*]}"
    nohup "$OSD_BIN" "${osd_args[@]}" >>"$osd_log" 2>&1 &
    sleep 0.4
    if _is_running "osd_overlay"; then
      _ok "osd_overlay running"
    else
      _warn "osd_overlay failed — see $osd_log"
    fi
  fi

  if [[ "${DRON_SKIP_STATS:-0}" != "1" ]]; then
    _say "Start stats_web :${STATS_WEB_PORT} (UDP telem listen :${TELEM_LISTEN_FOR_STATS})"
    VISION_TELEMETRY_PORT="$TELEM_LISTEN_FOR_STATS" \
      nohup "$PYTHON_BIN" "$STATS_PY" >>"$stats_log" 2>&1 &
    sleep 0.4
    if command -v curl >/dev/null 2>&1; then
      if curl -fsS --max-time 1 "http://127.0.0.1:${STATS_WEB_PORT}/health" >/dev/null 2>&1; then
        _ok "stats http://0.0.0.0:${STATS_WEB_PORT}/"
      else
        _warn "stats not responding yet — see $stats_log"
      fi
    else
      _ok "stats launched"
    fi
  fi

  _say "logs → $LOG_DIR"
  _say "MJPEG / UI / stats / BF:"
  case "${TRACKER}" in
    nano|nanotrack)
      _say "  MJPEG  http://0.0.0.0:5003/"
      _say "  Capture UI http://0.0.0.0:5004/"
      ;;
    light|lighttrack)
      _say "  MJPEG  http://0.0.0.0:5005/"
      _say "  Capture UI http://0.0.0.0:5006/"
      ;;
    nanov3|nano_v3|v3)
      _say "  MJPEG  http://0.0.0.0:5007/"
      _say "  Capture UI http://0.0.0.0:5008/"
      ;;
    csrt)
      _say "  MJPEG  http://0.0.0.0:5010/"
      _say "  Capture UI http://0.0.0.0:5011/"
      ;;
    kcf)
      _say "  MJPEG  http://0.0.0.0:5012/"
      _say "  Capture UI http://0.0.0.0:5013/"
      ;;
    mosse)
      _say "  MJPEG  http://0.0.0.0:5014/"
      _say "  Capture UI http://0.0.0.0:5015/"
      ;;
    mil)
      _say "  MJPEG  http://0.0.0.0:5016/"
      _say "  Capture UI http://0.0.0.0:5017/"
      ;;
    medianflow|mf)
      _say "  MJPEG  http://0.0.0.0:5018/"
      _say "  Capture UI http://0.0.0.0:5019/"
      ;;
    tld)
      _say "  MJPEG  http://0.0.0.0:5020/"
      _say "  Capture UI http://0.0.0.0:5021/"
      ;;
    dasiamrpn|dasiam|siamrpn)
      _say "  MJPEG  http://0.0.0.0:5022/"
      _say "  Capture UI http://0.0.0.0:5023/"
      ;;
    goturn)
      _say "  MJPEG  http://0.0.0.0:5024/"
      _say "  Capture UI http://0.0.0.0:5025/"
      ;;
    cftrack|cf|cfensemble)
      _say "  MJPEG  http://0.0.0.0:5026/"
      _say "  Capture UI http://0.0.0.0:5027/"
      ;;
    tctrack|tctrack++|tctrackpp|tctracklite)
      _say "  MJPEG  http://0.0.0.0:5028/"
      _say "  Capture UI http://0.0.0.0:5029/"
      ;;
    vit|vittrack|ostrack|ostrack256|mixformer|mixformerv2|mixformerv2s)
      _say "  MJPEG  http://0.0.0.0:5030/"
      _say "  Capture UI http://0.0.0.0:5031/"
      ;;
    *)
      _say "  MJPEG/UI ports depend on backend (see tracker.log)"
      ;;
  esac
  _say "  Stats  http://0.0.0.0:${STATS_WEB_PORT}/"
  _say "  MSP    ${BF_MSP_PORT} (guidance=${BF_GUIDANCE_ENABLE} arm=${BF_ARM_ENABLE})"
  [[ "$DRON_ENABLE_CHASE" == "1" ]] && _say "  chase_fc ENGAGE on CH8 (see chase.json)"
  [[ "$DRON_ENABLE_OSD" == "1" ]] && _say "  osd_overlay sink=${OSD_SINK:-none}"
}

do_status() {
  echo "=== drone-tracking status (Betaflight MSP) ==="
  for pat in orch_daemon nanotrack_fc lighttrack_fc multitrack_fc msp_betaflight.py chase_fc osd_overlay stats_web.py; do
    if _is_running "$pat"; then
      echo "  [OK] $pat  pids: $(pgrep -f "$pat" | tr '\n' ' ')"
    else
      echo "  [X]  $pat"
    fi
  done
  echo "--- ports ---"
  if command -v ss >/dev/null 2>&1; then
    ss -lntup 2>/dev/null | grep -E ':(500[3-9]|501[0-9]|502[0-9]|503[0-3]|8090|12345|12346|1234[79]|12350)\b' || true
  fi
  if command -v curl >/dev/null 2>&1; then
    curl -fsS --max-time 1 "http://127.0.0.1:${STATS_WEB_PORT}/stats.json" >/dev/null 2>&1 \
      && echo "  [OK] stats :${STATS_WEB_PORT}" \
      || echo "  [X]  stats :${STATS_WEB_PORT}"
  fi
  echo "PROJECT_DIR=$PROJECT_DIR TRACKER=$TRACKER CAMERA=$TRACK_CAMERA"
  echo "BF_MSP_PORT=$BF_MSP_PORT guidance=$BF_GUIDANCE_ENABLE arm=$BF_ARM_ENABLE"
  echo "CHASE=$DRON_ENABLE_CHASE OSD=$DRON_ENABLE_OSD SINK=${OSD_SINK:-}"
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
