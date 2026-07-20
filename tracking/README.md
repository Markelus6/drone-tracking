# Tracking — NanoTrack + LightTrack (NCNN)

| Binary | Models | Cmd | MJPEG |
|--------|--------|-----|-------|
| `nanotrack_fc` | `models/nanotrack_*.{param,bin}` | `:12347` | `:5003` |
| `lighttrack_fc` | `models/lighttrack/*` | `:12349` | `:5005` |

## Build

```bash
./build.sh
# ./build.sh --clean
```

Requires OpenCV + ncnn (`NCNN_DIR` in `CMakeLists.txt`, default `/root/ncnn-install`).

## Run

```bash
./build/lighttrack_fc --camera /dev/cam_usb2 --models ./models/lighttrack
./build/nanotrack_fc --camera /dev/cam_usb2 --models ./models
```

Camera frames come from `orch_daemon` SHM. Init bbox manually over UDP — no detector process.