#!/usr/bin/env python3
"""RTSP/MJPEG → POSIX SHM compatible with drone-tracking orch/trackers.

Matches orchestrator/camera_orchestrator.hpp::ShmHeader (64-byte header).
Default: /dev/cam_usb2 → /dev/shm/drone_cam_dev_cam_usb2

Publish protocol mirrors camera_orchestrator.cpp:
  ready=0 → barrier → write payload/meta → barrier → ready=1
so ARM readers cannot observe ready=1 before frame bytes are visible.
"""
from __future__ import annotations

import argparse
import ctypes
import mmap
import os
import platform
import struct
import sys
import time
from typing import Callable

import cv2
import numpy as np

# uint64 frame_id, timestamp_us
# uint32 width, height, channels, data_size, ready
# uint8 padding[28]
HEADER_FMT = "<QQIIIII"
HEADER_SIZE = struct.calcsize(HEADER_FMT) + 28  # 36 + 28 = 64
assert HEADER_SIZE == 64

# Offsets within ShmHeader (little-endian layout)
_OFF_FRAME_ID = 0
_OFF_READY = struct.calcsize("<QQIIII")  # 32
assert _OFF_READY == 32

_PROT_READ = 0x1
_PROT_WRITE = 0x2
_PROT_EXEC = 0x4
_MAP_PRIVATE = 0x02
_MAP_ANONYMOUS = 0x20


def shm_path_from_dev(device: str) -> str:
    return "/dev/shm/drone_cam" + device.replace("/", "_")


def pack_header(frame_id: int, ts: int, w: int, h: int, ch: int, data_size: int, ready: int) -> bytes:
    return struct.pack(HEADER_FMT, frame_id, ts, w, h, ch, data_size, ready) + bytes(28)


def _build_sync_synchronize() -> Callable[[], None]:
    """Return a full memory fence (GCC __sync_synchronize equivalent)."""
    machine = platform.machine().lower()
    # Machine code: full system fence + return
    if machine in ("aarch64", "arm64"):
        code = b"\xbf\x3f\x03\xd5\xc0\x03\x5f\xd6"  # dmb sy; ret
    elif machine.startswith("arm"):
        code = b"\xbf\xf3\x5f\xf5\x1e\xff\x2f\xe1"  # dmb sy; bx lr
    elif machine in ("x86_64", "amd64", "i386", "i686", "x86"):
        code = b"\x0f\xae\xf0\xc3"  # mfence; ret
    else:
        return lambda: None

    try:
        libc = ctypes.CDLL(None, use_errno=True)
        libc.mmap.restype = ctypes.c_void_p
        libc.mmap.argtypes = [
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_long,
        ]
        libc.mprotect.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
        libc.mprotect.restype = ctypes.c_int

        size = max(len(code), 16)
        addr = libc.mmap(
            None,
            size,
            _PROT_READ | _PROT_WRITE,
            _MAP_PRIVATE | _MAP_ANONYMOUS,
            -1,
            0,
        )
        if addr is None or addr == ctypes.c_void_p(-1).value:
            return lambda: None
        ctypes.memmove(addr, code, len(code))
        if libc.mprotect(ctypes.c_void_p(addr), size, _PROT_READ | _PROT_EXEC) != 0:
            return lambda: None
        fn = ctypes.CFUNCTYPE(None)(addr)
        # Keep mapping alive for process lifetime (never munmap).
        _build_sync_synchronize._keep = (addr, size, fn)  # type: ignore[attr-defined]
        return fn
    except (AttributeError, OSError, TypeError, ValueError):
        return lambda: None


sync_synchronize: Callable[[], None] = _build_sync_synchronize()


def set_ready(buf: mmap.mmap, ready: int) -> None:
    struct.pack_into("<I", buf, _OFF_READY, ready)


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

    # Static geometry once; per-frame we only flip ready + meta (like orch).
    buf[0:HEADER_SIZE] = pack_header(0, 0, w, h, ch, data_size, 0)

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

            # Match orch: ready=0 → barrier → payload+meta → barrier → ready=1
            set_ready(buf, 0)
            sync_synchronize()

            buf[HEADER_SIZE : HEADER_SIZE + data_size] = flat.tobytes()
            struct.pack_into("<QQ", buf, _OFF_FRAME_ID, frame_id, ts)

            sync_synchronize()
            set_ready(buf, 1)

            frame_id += 1

            now = time.time()
            if now < t_next:
                time.sleep(t_next - now)
            t_next = max(t_next + period, time.time())
    except KeyboardInterrupt:
        pass
    finally:
        set_ready(buf, 0)
        sync_synchronize()
        cap.release()
        buf.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
