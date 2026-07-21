# control — chase + OSD overlay (Orange Pi 5 interceptor)

| Binary | Role |
|--------|------|
| `chase_fc` | ELRS CRSF `0x16` → MSP `SET_RAW_RC` → Betaflight; AUX engage → LightTrack + servo + PID |
| `osd_overlay` | SHM frame + bbox → V4L2 sink (USB CVBS / loopback → analog VTX) |

## Build

```bash
cd control && ./build.sh
# or: bash deploy/build_all.sh
```

## Config

[`deploy/chase.json`](../deploy/chase.json) — UART paths, PID, servo µs, engage channel.

Wiring / overlays: [`deploy/opi5/WIRING.md`](../deploy/opi5/WIRING.md).

## Run

```bash
# after orch + lighttrack
./control/build/chase_fc --config deploy/chase.json
./control/build/osd_overlay --camera /dev/cam_usb2 --sink /dev/video10 --telem-port 12346
```

Stack:

```bash
DRON_ENABLE_CHASE=1 DRON_ENABLE_OSD=1 OSD_SINK=/dev/video10 \
  ./deploy/start_tracking.sh restart
```

Telemetry: LightTrack → `:12345` (chase). Chase forwards → `:12346` (stats), `:12350` (osd).

## Modes

1. **Manual** — CRSF channels copied to MSP.
2. **Engage** (CH8 high) — servo nose, `init` center box, yaw/pitch PID on `bbox_norm`.
3. **Disengage** — `reset`, servo idle (~30°), passthrough.
4. **Failsafe** — no CRSF > `failsafe_ms` → stop MSP updates (BF failsafe).
