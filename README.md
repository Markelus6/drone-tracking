# drone-tracking

Visual object tracking for Orange Pi / RK3588 + **Betaflight** (MSP), not ArduPilot/MAVLink.

**No YOLO. No optical flow. No MAVLink.** Tracker → UDP bbox → `msp_betaflight.py` → UART MSP `SET_RAW_RC`.

## Contents

| Path | Role |
|------|------|
| `tracking/` | NanoTrack + LightTrack (NCNN) |
| `orchestrator/` | V4L2 → shared-memory camera frames |
| `tracking/models/` | NCNN `.param` / `.bin` weights |
| `deploy/` | Start scripts, systemd, stats `:8090`, **MSP Betaflight** |

## Dependencies

- OpenCV
- [ncnn](https://github.com/Tencent/ncnn) (default install path: `/root/ncnn-install`)
- pthread, rt
- Python 3 + `pyserial` (`pip3 install pyserial`) for MSP UART

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

Binaries: `tracking/build/nanotrack_fc`, `tracking/build/lighttrack_fc`, `tracking/build/multitrack_fc`, `orchestrator/build/orch_daemon`.

## Trackers (`TRACKER=…`)

| TRACKER | Notes | UI |
|---------|-------|-----|
| `nano` / `light` | NCNN production | :5004 / :5006 |
| `nanov3` | NanoTrack V3 ONNX | :5008 |
| `csrt` `kcf` `mosse` `mil` `medianflow` `tld` | OpenCV classical | :5011+ |
| `dasiamrpn` `goturn` | OpenCV DNN (fetch models) | :5023 / :5025 |
| `cftrack` | KCF+CSRT ensemble | :5027 |
| `tctrack` | NanoV3 + template refresh | :5029 |
| `vit`/`ostrack`/`mixformer` | needs OpenCV ≥4.8 | :5031 |

See `tracking/models/COMPARE.md`. Stats: http://\<IP\>:8090/

## Deploy / run on drone

```bash
cd /root/drone-tracking/deploy
TRACKER=light ./start_tracking.sh restart   # or TRACKER=nano
./start_tracking.sh status

# Enable stick output (after BF Ports/Modes configured):
BF_GUIDANCE_ENABLE=1 ./start_tracking.sh restart
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

### MJPEG preview (default ~30 FPS, low CPU)

Tracker runs at full camera resolution; the browser preview is downscaled JPEG:

| Env | Default | Meaning |
|-----|---------|---------|
| `VISION_MJPEG_PERIOD_MS` | `33` | ~30 FPS encode cadence |
| `VISION_MJPEG_QUALITY` | `40` | JPEG quality |
| `VISION_STREAM_MAX_W` | `480` | max preview width (`0` = full) |

Full-res preview: `VISION_STREAM_MAX_W=0`. Lower CPU further: `VISION_STREAM_MAX_W=320`.

## Betaflight (MSP) — replaces MAVLink

```text
orch → tracker → UDP :12345 → msp_betaflight.py → /dev/ttyS4 MSP → Betaflight
                              ↘ /dev/shm/drone_telem.json → stats :8090
```

| Env | Default | Meaning |
|-----|---------|---------|
| `BF_MSP_ENABLE` | `1` | start MSP companion |
| `BF_MSP_PORT` | `/dev/ttyS4` | UART to BF (Ports tab → MSP) |
| `BF_MSP_BAUD` | `115200` | |
| `BF_MSP_HZ` | `50` | `SET_RAW_RC` rate |
| `BF_MAP` | `aetr` | channel order |
| `BF_GUIDANCE_ENABLE` | `0` | `1` = yaw/pitch from bbox (safe off by default) |
| `BF_ARM_ENABLE` | `0` | `1` = raise AUX arm when guiding |
| `BF_AUX_ARM_CH` | `5` | AUX channel index (1-based) |

**BF Configurator:** enable MSP on the UART wired to the Orange Pi; set Mode for MSP Override / Angle as needed; keep ELRS on a **different** UART. Failsafe must return control to RX if companion dies.

Guidance stays mid-sticks until `BF_GUIDANCE_ENABLE=1` and a fresh bbox arrives.

## Manual run (debug)

```bash
./orchestrator/build/orch_daemon --add /dev/cam_usb2,640,480,30

./tracking/build/lighttrack_fc \
  --camera /dev/cam_usb2 \
  --models ./tracking/models/lighttrack

./tracking/build/nanotrack_fc \
  --camera /dev/cam_usb2 \
  --models ./tracking/models

python3 deploy/msp_betaflight.py   # owns :12345 → MSP UART
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

Telemetry → `:12345` → **msp_betaflight** (bbox + heartbeat):

```json
{"cam":"front","tracker":"lighttrack","bbox_norm":[0.51,0.48,0.12,0.10],"class_id":0,"conf":1.0}
```

| Binary | Cmd / role |
|--------|------------|
| `nanotrack_fc` | cmd :12347 · MJPEG :5003 · UI :5004 |
| `lighttrack_fc` | cmd :12349 · MJPEG :5005 · UI :5006 |
| `msp_betaflight.py` | UDP :12345 → MSP UART (Betaflight) |
| `stats_web.py` | HTTP **:8090** |

## Contract

1. Frames from orchestrator SHM (`/dev/shm/drone_cam_*`)
2. Target seed via UDP `init` / `reset` only
3. Output: UDP bbox JSON → Betaflight MSP `SET_RAW_RC` (not MAVLink)
4. Ops visibility: HTTP stats on `:8090`
