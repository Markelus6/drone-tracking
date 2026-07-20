# Camera orchestrator

Owns V4L2 cameras and publishes frames to POSIX shared memory for trackers.

```bash
./build.sh
./build/orch_daemon --add /dev/cam_usb2,640,480,30
```

SHM name: `/drone_cam` + device path with `/` → `_` (e.g. `/drone_cam_dev_cam_usb2`).