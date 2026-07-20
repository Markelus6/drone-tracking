# drone-tracking

Standalone visual object tracking for Orange Pi / RK3588.

**Only tracking.** No YOLO. No optical flow. No flight controller.

## Contents

| Path | Role |
|------|------|
| `tracking/` | NanoTrack + LightTrack (NCNN) |
| `orchestrator/` | V4L2 → shared-memory camera frames |
| `tracking/models/` | NCNN `.param` / `.bin` weights |

## Dependencies

- OpenCV
- [ncnn](https://github.com/Tencent/ncnn) (default install path: `/root/ncnn-install`)
- pthread, rt

Override ncnn path in `tracking/CMakeLists.txt` (`NCNN_DIR`) if needed.

## Build

```bash
# camera SHM daemon
cd orchestrator && ./build.sh

# trackers
cd ../tracking && ./build.sh
```

Binaries: `tracking/build/nanotrack_fc`, `tracking/build/lighttrack_fc`, `orchestrator/build/orch_daemon`.

## Run

```bash
# 1) camera → SHM
./orchestrator/build/orch_daemon --add /dev/cam_usb2,640,480,30

# 2) tracker (pick one)
./tracking/build/lighttrack_fc \
  --camera /dev/cam_usb2 \
  --models ./tracking/models/lighttrack

./tracking/build/nanotrack_fc \
  --camera /dev/cam_usb2 \
  --models ./tracking/models
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

Telemetry → `:12345`:

```json
{"cam":"front","tracker":"lighttrack","bbox_norm":[0.51,0.48,0.12,0.10],"class_id":0,"conf":1.0}
```

| Binary | Cmd port | MJPEG |
|--------|----------|-------|
| `nanotrack_fc` | 12347 | 5003 |
| `lighttrack_fc` | 12349 | 5005 (+ UI :5006) |

## Contract

1. Frames from orchestrator SHM (`/dev/shm/drone_cam_*`)
2. Target seed via UDP `init` / `reset` only
3. Output: UDP bbox JSON with `"tracker":"nanotrack"|"lighttrack"`