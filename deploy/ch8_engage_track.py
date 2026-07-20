#!/usr/bin/env python3
"""When RC CH8 ~= 2000us, send LightTrack center-lock init; when low, reset.

Reads CRSF 0x16 frames from a UART (or a test --simulate flag).
Sends UDP JSON to lighttrack_fc cmd port :12349.
"""
from __future__ import annotations

import argparse
import socket
import struct
import time

# CRSF
SYNC = 0xC8
TYPE_RC = 0x16
CRC_TAB = [
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
]


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = CRC_TAB[c ^ b]
    return c


def ticks_to_us(t: int) -> int:
    return (t - 992) * 5 // 8 + 1500


def unpack_channels(payload22: bytes) -> list[int]:
    """Decode 16x11-bit channels from 22-byte CRSF 0x16 payload → microseconds."""
    if len(payload22) < 22:
        return []
    bits = int.from_bytes(payload22[:22], "little")
    ch = []
    for i in range(16):
        raw = (bits >> (11 * i)) & 0x7FF
        ch.append(ticks_to_us(raw))
    return ch


def send_udp(host: str, port: int, msg: str) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(msg.encode("ascii"), (host, port))
    sock.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="/dev/ttyS0")
    ap.add_argument("--baud", type=int, default=420000)
    ap.add_argument("--cmd-host", default="127.0.0.1")
    ap.add_argument("--cmd-port", type=int, default=12349)
    ap.add_argument("--ch", type=int, default=8, help="1-based RC channel index")
    ap.add_argument("--high-us", type=int, default=1800)
    ap.add_argument("--low-us", type=int, default=1200)
    ap.add_argument("--box", default="0.5,0.5,0.15,0.15")
    ap.add_argument("--simulate", action="store_true")
    args = ap.parse_args()
    cx, cy, bw, bh = [float(x) for x in args.box.split(",")]
    idx = args.ch - 1
    engaged = False

    def engage():
        nonlocal engaged
        if engaged:
            return
        msg = f'{{"cmd":"init","bbox_norm":[{cx},{cy},{bw},{bh}]}}'
        send_udp(args.cmd_host, args.cmd_port, msg)
        engaged = True
        print("[ch8] ENGAGE", msg, flush=True)

    def disengage():
        nonlocal engaged
        if not engaged:
            return
        send_udp(args.cmd_host, args.cmd_port, '{"cmd":"reset"}')
        engaged = False
        print("[ch8] RESET", flush=True)

    if args.simulate:
        print("[ch8] simulate: high after 2s, low after 10s")
        time.sleep(2)
        engage()
        time.sleep(8)
        disengage()
        return 0

    import serial  # type: ignore

    ser = serial.Serial(args.uart, args.baud, timeout=0.05)
    buf = bytearray()
    print(f"[ch8] listening {args.uart} @ {args.baud}, CH{args.ch}", flush=True)
    while True:
        chunk = ser.read(64)
        if chunk:
            buf.extend(chunk)
        while True:
            if SYNC not in buf:
                buf.clear()
                break
            start = buf.index(SYNC)
            if start:
                del buf[:start]
            if len(buf) < 3:
                break
            length = buf[1]
            frame_len = length + 2
            if length < 2 or length > 62:
                del buf[0]
                continue
            if len(buf) < frame_len:
                break
            frame = bytes(buf[:frame_len])
            del buf[:frame_len]
            ftype = frame[2]
            payload = frame[3:-1]
            if crc8(frame[2:-1]) != frame[-1]:
                continue
            if ftype != TYPE_RC or len(payload) < 22:
                continue
            ch = unpack_channels(payload)
            if len(ch) <= idx:
                continue
            us = ch[idx]
            if us >= args.high_us:
                engage()
            elif us <= args.low_us:
                disengage()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
