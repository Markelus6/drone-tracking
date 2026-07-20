#!/usr/bin/env python3
"""RTSP/MJPEG → POSIX SHM compatible with drone-tracking orch/trackers.

Matches orchestrator/camera_orchestrator.hpp::ShmHeader (64-byte header).
Default: /dev/cam_usb2 → /dev/shm/drone_cam_dev_cam_usb2
"""
from __future__ import annotations

import argparse
import mmap
import os
import struct
import sys
import time

import cv2
import numpy as np

# uint64 frame_id, timestamp_us
# uint32 width, height, channels, data_size, ready
# uint8 padding[28]
HEADER_FMT = "<QQIIIII"
HEADER_SIZE = struct.calcsize(HEADER_FMT) + 28  # 36 + 28 = 64
assert HEADER_SIZE == 64


def shm_path_from_dev(device: str) -> str:
    return "/dev/shm/drone_cam" + device.replace("/", "_")


def pack_header(frame_id: int, ts: int, w: int, h: int, ch: int, data_size: int, ready: int) -> bytes:
    return struct.pack(HEADER_FMT, frame_id, ts, w, h, ch, data_size, ready) + bytes(28)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="rtsp://root:12345@192.168.1.10/stream=0")
    ap.add_argument("--device", default="/dev/cam_usb2")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    args = ap.parse_args()

    w, h, ch = args.width, args.height, 3
    data_size = w * h * ch
    total = HEADER_SIZE + data_size
    path = shm_path_from_dev(args.device)

    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    os.ftruncate(fd, total)
    buf = mmap.mmap(fd, total)
    os.close(fd)

    print(f"[rtsp_shm] {args.url} -> {path} {w}x{h}@{args.fps}", flush=True)

    cap = cv2.VideoCapture(args.url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print("[rtsp_shm] open failed", file=sys.stderr)
        return 1

    frame_id = 0
    period = 1.0 / max(args.fps, 1)
    t_next = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            if frame.shape[1] != w or frame.shape[0] != h:
                frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            flat = np.ascontiguousarray(frame, dtype=np.uint8).reshape(-1)
            if flat.size != data_size:
                continue

            ts = int(time.time() * 1e6)
            buf[0:HEADER_SIZE] = pack_header(frame_id, ts, w, h, ch, data_size, 0)
            buf[HEADER_SIZE : HEADER_SIZE + data_size] = flat.tobytes()
            buf[0:HEADER_SIZE] = pack_header(frame_id, ts, w, h, ch, data_size, 1)
            frame_id += 1

            now = time.time()
            if now < t_next:
                time.sleep(t_next - now)
            t_next = max(t_next + period, time.time())
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        buf.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
