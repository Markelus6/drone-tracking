# drone-tracking

Standalone visual object tracking for Orange Pi / RK3588.

**Only tracking.** No YOLO. No optical flow. No flight controller.

## Contents

| Path | Role |
|------|------|
| `tracking/` | NanoTrack + LightTrack (NCNN) |
| `orchestrator/` | V4L2 → shared-memory camera frames |
| `control/` | `chase_fc` (CRSF→MSP) + `osd_overlay` |
| `tracking/models/` | NCNN `.param` / `.bin` weights |
| `deploy/` | On-drone start scripts, `chase.json`, stats `:8090` |
| `reference/rpi_interceptor/` | Recovered old RPi system (app, config, unlock) |

## Dependencies

- OpenCV
- [ncnn](https://github.com/Tencent/ncnn) (default install path: `/root/ncnn-install`)
- pthread, rt
- Python 3 (only for `deploy/stats_web.py`)

Override ncnn path in `tracking/CMakeLists.txt` (`NCNN_DIR`) if needed.

## Build (on board)

```bash
cd /root/drone-tracking
bash deploy/build_all.sh
```

Or separately:

```bash
cd orchestrator && ./build.sh
cd ../tracking && ./build.sh
```

Binaries: `tracking/build/nanotrack_fc`, `tracking/build/lighttrack_fc`, `orchestrator/build/orch_daemon`.

## Deploy / run on drone

```bash
cd /root/drone-tracking/deploy
TRACKER=light ./start_tracking.sh restart   # or TRACKER=nano
./start_tracking.sh status
```

systemd:

```bash
cp deploy/drone-tracking.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now drone-tracking
```

Details: [deploy/README.md](deploy/README.md).

### Stats `:8090`

Dashboard + JSON (same idea as perehvatchik FC stats):

- `http://<board-ip>:8090/` — live cards (tracker, telem, orch, CPU/RAM, temps)
- `http://<board-ip>:8090/stats.json` — machine-readable snapshot
- `http://<board-ip>:8090/health` — liveness

Env: `STATS_WEB_PORT`, `STATS_WEB_POLL_MS`, `STATS_WEB_JSON_CACHE_MS`, `VISION_TELEMETRY_PORT`.

## Manual run (debug)

```bash
./orchestrator/build/orch_daemon --add /dev/cam_usb2,640,480,30

./tracking/build/lighttrack_fc \
  --camera /dev/cam_usb2 \
  --models ./tracking/models/lighttrack

./tracking/build/nanotrack_fc \
  --camera /dev/cam_usb2 \
  --models ./tracking/models

python3 deploy/stats_web.py
```

## Protocol

Init / reset over UDP (no detector required):

```bash
# LightTrack commands :12349
echo '{"cmd":"init","bbox_norm":[0.5,0.5,0.2,0.2]}' > /dev/udp/127.0.0.1/12349
echo '{"cmd":"reset"}' > /dev/udp/127.0.0.1/12349

# NanoTrack commands :12347
echo '{"cmd":"init","bbox_norm":[0.5,0.5,0.2,0.2]}' > /dev/udp/127.0.0.1/12347
```

Telemetry → `:12345` (bbox + 1 Hz heartbeat with FPS / track_ms):

```json
{"cam":"front","tracker":"lighttrack","bbox_norm":[0.51,0.48,0.12,0.10],"class_id":0,"conf":1.0}
{"tracker":"lighttrack","cam":"front","alive":true,"tracking":true,"fps":28.5,"track_ms_avg":12.1,...}
```

| Binary | Cmd port | MJPEG | Capture UI |
|--------|----------|-------|------------|
| `nanotrack_fc` | 12347 | 5003 | 5004 |
| `lighttrack_fc` | 12349 | 5005 | 5006 |
| `stats_web.py` | — | — | **8090** |

## Contract

1. Frames from orchestrator SHM (`/dev/shm/drone_cam_*`)
2. Target seed via UDP `init` / `reset` only
3. Output: UDP bbox JSON with `"tracker":"nanotrack"|"lighttrack"`
4. Ops visibility: HTTP stats on `:8090`
