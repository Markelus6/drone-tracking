#!/usr/bin/env python3
"""
Betaflight MSP companion for drone-tracking.

Replaces ArduPilot/MAVLink RC_CHANNELS_OVERRIDE with MSP_SET_RAW_RC.

  tracker UDP :12345 (bbox) → this process → UART MSP → Betaflight

Safety defaults:
  - sticks stay mid unless BF_GUIDANCE_ENABLE=1 and fresh bbox
  - arm AUX only if BF_ARM_ENABLE=1
  - lost/stale target → mid sticks

Env:
  BF_MSP_PORT          UART device (default /dev/ttyS4)
  BF_MSP_BAUD          default 115200
  BF_MSP_HZ            SET_RAW_RC rate (default 50)
  BF_GUIDANCE_ENABLE   1 = send tracking sticks (default 0 = mid only / dry-run OK)
  BF_ARM_ENABLE        1 = raise AUX arm channel when guiding
  BF_RC_CHANNELS       channel count (default 8)
  BF_MAP               AETR channel order (default aetr)
  BF_YAW_GAIN          err→PWM gain (default 350)
  BF_PITCH_GAIN        (default 350)
  BF_PITCH_INVERT      1 = invert pitch (default 1, camera look)
  BF_DEADZONE          |err| below this → center (default 0.12)
  BF_BBOX_TIMEOUT_S    stale bbox → mid (default 0.4)
  BF_THROTTLE_PWM      hold throttle (default 1500)
  BF_AUX_ARM_CH        1-based channel for arm (default 5)
  BF_AUX_ARM_HIGH      PWM when armed (default 2000)
  BF_AUX_ARM_LOW       PWM when safe (default 1000)
  VISION_TELEMETRY_PORT  UDP listen (default 12345)
  BF_TELEM_FILE        shared snapshot for stats_web (default /dev/shm/drone_telem.json)
"""

from __future__ import annotations

import json
import os
import socket
import struct
import threading
import time
from typing import Any

try:
    import serial  # type: ignore
except ImportError:
    serial = None  # type: ignore

MSP_SET_RAW_RC = 200
MSP_STATUS = 101
MSP_ATTITUDE = 108
MSP_ANALOG = 110

BF_MSP_PORT = os.environ.get("BF_MSP_PORT", "/dev/ttyS4")
BF_MSP_BAUD = int(os.environ.get("BF_MSP_BAUD", "115200"))
BF_MSP_HZ = max(20, int(os.environ.get("BF_MSP_HZ", "50")))
BF_GUIDANCE_ENABLE = os.environ.get("BF_GUIDANCE_ENABLE", "0") == "1"
BF_ARM_ENABLE = os.environ.get("BF_ARM_ENABLE", "0") == "1"
BF_RC_CHANNELS = max(4, min(18, int(os.environ.get("BF_RC_CHANNELS", "8"))))
BF_MAP = (os.environ.get("BF_MAP", "aetr") or "aetr").strip().lower()
BF_YAW_GAIN = float(os.environ.get("BF_YAW_GAIN", "350"))
BF_PITCH_GAIN = float(os.environ.get("BF_PITCH_GAIN", "350"))
BF_PITCH_INVERT = os.environ.get("BF_PITCH_INVERT", "1") != "0"
BF_DEADZONE = float(os.environ.get("BF_DEADZONE", "0.12"))
BF_BBOX_TIMEOUT_S = float(os.environ.get("BF_BBOX_TIMEOUT_S", "0.4"))
BF_THROTTLE_PWM = int(os.environ.get("BF_THROTTLE_PWM", "1500"))
BF_AUX_ARM_CH = int(os.environ.get("BF_AUX_ARM_CH", "5"))
BF_AUX_ARM_HIGH = int(os.environ.get("BF_AUX_ARM_HIGH", "2000"))
BF_AUX_ARM_LOW = int(os.environ.get("BF_AUX_ARM_LOW", "1000"))
VISION_TELEMETRY_PORT = int(os.environ.get("VISION_TELEMETRY_PORT", "12345"))
BF_TELEM_FILE = os.environ.get("BF_TELEM_FILE", "/dev/shm/drone_telem.json")
BF_DRY_RUN = os.environ.get("BF_DRY_RUN", "0") == "1"

_lock = threading.Lock()
_state: dict[str, Any] = {
    "last_ts": 0.0,
    "bbox_ts": 0.0,
    "lost_ts": 0.0,
    "alive_ts": 0.0,
    "tracker": None,
    "cam": None,
    "bbox_norm": None,
    "conf": None,
    "tracking": False,
    "packets": 0,
    "cx": None,
    "cy": None,
    "h": None,
    "score": None,
    "fps": None,
}


def _clamp_pwm(v: float, lo: int = 1000, hi: int = 2000) -> int:
    return max(lo, min(hi, int(round(v))))


def _msp_checksum(size: int, cmd: int, payload: bytes) -> int:
    c = size ^ cmd
    for b in payload:
        c ^= b
    return c & 0xFF


def msp_pack(cmd: int, payload: bytes = b"") -> bytes:
    """MSP v1 request/command frame: $M< size cmd payload checksum."""
    size = len(payload)
    if size > 254:
        raise ValueError("MSP v1 payload too large")
    return (
        b"$M<"
        + bytes([size, cmd & 0xFF])
        + payload
        + bytes([_msp_checksum(size, cmd, payload)])
    )


def msp_set_raw_rc(channels: list[int]) -> bytes:
    payload = b"".join(struct.pack("<H", _clamp_pwm(c)) for c in channels)
    return msp_pack(MSP_SET_RAW_RC, payload)


def _axis_pwms(cx: float, cy: float) -> tuple[int, int, int, int]:
    """Return roll, pitch, throttle, yaw PWM from normalized bbox center."""
    err_x = (cx - 0.5) * 2.0
    err_y = (cy - 0.5) * 2.0
    if abs(err_x) < BF_DEADZONE:
        err_x = 0.0
    if abs(err_y) < BF_DEADZONE:
        err_y = 0.0
    yaw = 1500.0 + err_x * BF_YAW_GAIN
    pitch_err = -err_y if BF_PITCH_INVERT else err_y
    pitch = 1500.0 + pitch_err * BF_PITCH_GAIN
    roll = 1500  # hold mid — yaw/pitch visual servoing first
    thr = BF_THROTTLE_PWM
    return (
        roll,
        _clamp_pwm(pitch, 1300, 1700),
        _clamp_pwm(thr),
        _clamp_pwm(yaw, 1300, 1700),
    )


def _mid_channels(n: int) -> list[int]:
    ch = [1500] * n
    if n >= 3:
        ch[2] = BF_THROTTLE_PWM  # may be overridden by map
    return ch


def _apply_map(roll: int, pitch: int, thr: int, yaw: int, n: int) -> list[int]:
    """Build channel list. Default AETR = roll,pitch,throttle,yaw."""
    ch = _mid_channels(n)
    axes = {
        "a": roll,
        "e": pitch,
        "t": thr,
        "r": yaw,
    }
    order = BF_MAP if len(BF_MAP) >= 4 else "aetr"
    for i, letter in enumerate(order[:4]):
        if i < n and letter in axes:
            ch[i] = axes[letter]
    # AUX arm
    aux_i = BF_AUX_ARM_CH - 1
    if 0 <= aux_i < n:
        ch[aux_i] = BF_AUX_ARM_LOW
    return ch


def _compute_rc(now: float) -> tuple[list[int], dict[str, Any]]:
    with _lock:
        s = dict(_state)
    n = BF_RC_CHANNELS
    meta: dict[str, Any] = {
        "guidance": False,
        "arm_aux": False,
        "reason": "idle",
        "bbox_age_s": -1.0,
    }
    bbox = s.get("bbox_norm")
    bbox_ts = float(s.get("bbox_ts") or 0.0)
    age = (now - bbox_ts) if bbox_ts > 0 else 1e9
    meta["bbox_age_s"] = age if bbox_ts > 0 else -1.0

    if not BF_GUIDANCE_ENABLE:
        ch = _apply_map(1500, 1500, BF_THROTTLE_PWM, 1500, n)
        meta["reason"] = "guidance_disabled"
        return ch, meta

    if not bbox or age > BF_BBOX_TIMEOUT_S:
        ch = _apply_map(1500, 1500, BF_THROTTLE_PWM, 1500, n)
        meta["reason"] = "no_target" if not bbox else "stale_bbox"
        return ch, meta

    cx, cy = float(bbox[0]), float(bbox[1])
    roll, pitch, thr, yaw = _axis_pwms(cx, cy)
    ch = _apply_map(roll, pitch, thr, yaw, n)
    meta.update(
        {
            "guidance": True,
            "reason": "tracking",
            "cx": cx,
            "cy": cy,
            "roll": ch[0] if n > 0 else None,
            "pitch": ch[1] if n > 1 else None,
            "throttle": ch[2] if n > 2 else None,
            "yaw": ch[3] if n > 3 else None,
        }
    )
    if BF_ARM_ENABLE:
        aux_i = BF_AUX_ARM_CH - 1
        if 0 <= aux_i < n:
            ch[aux_i] = BF_AUX_ARM_HIGH
            meta["arm_aux"] = True
    return ch, meta


def _ingest(raw: str) -> None:
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return
    if not isinstance(data, dict):
        return
    now = time.time()
    with _lock:
        _state["last_ts"] = now
        _state["packets"] = int(_state.get("packets") or 0) + 1
        if data.get("tracker"):
            _state["tracker"] = str(data["tracker"])
        if data.get("cam"):
            _state["cam"] = str(data["cam"])
        if data.get("alive"):
            _state["alive_ts"] = now
        if data.get("lost"):
            _state["lost_ts"] = now
            _state["tracking"] = False
            _state["bbox_norm"] = None
        bbox = data.get("bbox_norm")
        if isinstance(bbox, list) and len(bbox) >= 4:
            _state["bbox_ts"] = now
            _state["bbox_norm"] = [float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])]
            _state["tracking"] = True
        if "tracking" in data:
            _state["tracking"] = bool(data["tracking"])
        for key in ("conf", "fps", "cx", "cy", "h", "score"):
            if key in data and data[key] is not None:
                try:
                    _state[key] = float(data[key])
                except (TypeError, ValueError):
                    pass


def _write_telem_file(rc: list[int], meta: dict[str, Any], msp_ok: bool, err: str | None) -> None:
    now = time.time()
    with _lock:
        s = dict(_state)
    snap = {
        "now_ts": now,
        "project": "drone-tracking",
        "fc": "betaflight",
        "protocol": "msp",
        "msp": {
            "port": BF_MSP_PORT,
            "baud": BF_MSP_BAUD,
            "hz": BF_MSP_HZ,
            "ok": msp_ok,
            "error": err,
            "dry_run": BF_DRY_RUN or serial is None,
            "guidance_enable": BF_GUIDANCE_ENABLE,
            "arm_enable": BF_ARM_ENABLE,
            "map": BF_MAP,
        },
        "rc": {
            "channels": rc,
            **meta,
        },
        "tracker": {
            "name": s.get("tracker"),
            "cam": s.get("cam"),
            "tracking": bool(s.get("tracking")),
            "bbox_norm": s.get("bbox_norm"),
            "conf": s.get("conf") if s.get("conf") is not None else s.get("score"),
            "fps": s.get("fps"),
            "cx": s.get("cx"),
            "cy": s.get("cy"),
            "h": s.get("h"),
            "score": s.get("score"),
            "packets": s.get("packets"),
            "bbox_age_s": meta.get("bbox_age_s"),
            "alive_age_s": (now - float(s["alive_ts"])) if s.get("alive_ts") else -1.0,
            "telem_age_s": (now - float(s["last_ts"])) if s.get("last_ts") else -1.0,
        },
        # Compatible keys for stats_web file ingest
        "bbox_norm": s.get("bbox_norm"),
        "tracker_name": s.get("tracker"),
        "cam": s.get("cam"),
        "fps": s.get("fps"),
        "score": s.get("score"),
        "conf": s.get("conf"),
        "tracking": bool(s.get("tracking")),
        "last_ts": s.get("last_ts"),
        "bbox_ts": s.get("bbox_ts"),
        "alive_ts": s.get("alive_ts"),
        "lost_ts": s.get("lost_ts"),
        "packets": s.get("packets"),
    }
    path = BF_TELEM_FILE
    tmp = path + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(snap, f, separators=(",", ":"))
        os.replace(tmp, path)
    except OSError:
        pass


def _udp_thread() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", VISION_TELEMETRY_PORT))
    sock.settimeout(1.0)
    print(f"[msp_bf] UDP telem listen :{VISION_TELEMETRY_PORT}", flush=True)
    while True:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            _ingest(data.decode("utf-8", errors="replace").strip())
        except Exception:
            continue


def run() -> None:
    print(
        f"[msp_bf] Betaflight MSP  port={BF_MSP_PORT} baud={BF_MSP_BAUD} "
        f"hz={BF_MSP_HZ} map={BF_MAP} guidance={int(BF_GUIDANCE_ENABLE)} "
        f"arm={int(BF_ARM_ENABLE)} dry_run={int(BF_DRY_RUN or serial is None)}",
        flush=True,
    )
    if serial is None:
        print("[msp_bf] WARN: pyserial not installed — dry-run (no UART)", flush=True)

    ser = None
    msp_err: str | None = None
    if serial is not None and not BF_DRY_RUN:
        try:
            ser = serial.Serial(BF_MSP_PORT, BF_MSP_BAUD, timeout=0.02, write_timeout=0.05)
            print(f"[msp_bf] UART open OK {BF_MSP_PORT}", flush=True)
        except Exception as e:
            msp_err = str(e)
            print(f"[msp_bf] UART open failed: {e} — continuing dry-run", flush=True)
            ser = None

    threading.Thread(target=_udp_thread, daemon=True, name="BfTelemUDP").start()

    period = 1.0 / BF_MSP_HZ
    frames = 0
    last_log = time.time()
    while True:
        t0 = time.time()
        rc, meta = _compute_rc(t0)
        ok = False
        err = msp_err
        if ser is not None:
            try:
                ser.write(msp_set_raw_rc(rc))
                ok = True
                err = None
            except Exception as e:
                err = str(e)
                ok = False
        else:
            ok = False
            if err is None:
                err = "dry_run" if (BF_DRY_RUN or serial is None) else "no_serial"

        _write_telem_file(rc, meta, ok, err)
        frames += 1
        now = time.time()
        if now - last_log >= 2.0:
            print(
                f"[msp_bf] hz~{frames / (now - last_log):.0f} guidance={meta.get('guidance')} "
                f"reason={meta.get('reason')} yaw={rc[3] if len(rc) > 3 else '-'} "
                f"msp_ok={ok} err={err}",
                flush=True,
            )
            frames = 0
            last_log = now

        dt = time.time() - t0
        sleep = period - dt
        if sleep > 0:
            time.sleep(sleep)


if __name__ == "__main__":
    run()
