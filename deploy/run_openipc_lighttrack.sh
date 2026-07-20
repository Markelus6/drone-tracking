#!/usr/bin/env bash
# Start OpenIPC LAN → SHM → LightTrack on Orange Pi
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export LD_LIBRARY_PATH="/home/orangepi/ncnn-install/lib:${LD_LIBRARY_PATH:-}"

URL="${OPENIPC_URL:-rtsp://root:12345@192.168.1.10/stream=0}"
CAM="${TRACK_CAMERA:-/dev/cam_usb2}"
W="${TRACK_CAM_W:-640}"
H="${TRACK_CAM_H:-480}"
FPS="${TRACK_CAM_FPS:-30}"

mkdir -p "$ROOT/deploy/logs"
pkill -f rtsp_shm_writer.py 2>/dev/null || true
pkill -f lighttrack_fc 2>/dev/null || true
sleep 1

echo "[run] ping camera..."
if ! ping -c 1 -W 2 192.168.1.10 >/dev/null; then
  echo "ERROR: 192.168.1.10 unreachable. Plug camera Ethernet + route via PC or same LAN."
  exit 1
fi

echo "[run] RTSP writer $URL -> SHM $CAM ${W}x${H}@$FPS"
python3 "$ROOT/deploy/rtsp_shm_writer.py" --url "$URL" --device "$CAM" --width "$W" --height "$H" --fps "$FPS" \
  >"$ROOT/deploy/logs/rtsp_shm.log" 2>&1 &
echo $! >"$ROOT/deploy/logs/rtsp_shm.pid"
sleep 2

echo "[run] lighttrack_fc"
"$ROOT/tracking/build/lighttrack_fc" \
  --camera "$CAM" \
  --models "$ROOT/tracking/models/lighttrack" \
  >"$ROOT/deploy/logs/lighttrack.log" 2>&1 &
echo $! >"$ROOT/deploy/logs/lighttrack.pid"
sleep 1

IP=$(hostname -I | awk '{print $1}')
echo "OK"
echo "  MJPEG+box:  http://${IP}:5005/"
echo "  Capture UI: http://${IP}:5006/"
echo "  Center lock: python3 $ROOT/deploy/ch8_engage_track.py --simulate"
echo "  CH8 engage:  python3 $ROOT/deploy/ch8_engage_track.py --uart /dev/ttyS0"
