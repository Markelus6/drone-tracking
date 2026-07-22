# Deploy notes — Orange Pi 5 / RK3588 + Betaflight

## Layout on board

```text
drone-tracking/
  orchestrator/build/orch_daemon
  tracking/build/{nanotrack_fc,lighttrack_fc,multitrack_fc}
  control/build/{chase_fc,osd_overlay}
  deploy/
    start_tracking.sh
    chase.json
    opi5/WIRING.md
    msp_betaflight.py     # MSP SET_RAW_RC (not MAVLink)
    stats_web.py          # HTTP :8090
    drone-tracking.service
    logs/
```

## First install

```bash
scp -r drone-tracking root@<IP>:/root/
ssh root@<IP>
pip3 install pyserial
cd /root/drone-tracking
bash deploy/build_all.sh
chmod +x deploy/*.sh deploy/opi5/*.sh control/build.sh
sudo bash deploy/opi5/install_overlays.sh
cp deploy/drone-tracking.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now drone-tracking
# after reboot: wire ELRS/FC/servo per deploy/opi5/WIRING.md
# Betaflight: MSP on UART to Pi, Serial RX off, Receiver = MSP
```

## Betaflight wiring

1. Orange Pi UART (default `/dev/ttyS4`) ↔ BF UART with **MSP** enabled (115200)
2. ELRS/CRSF on a **separate** BF UART
3. Modes: configure arm AUX (default CH5) and optionally MSP Override / Angle
4. Leave `BF_GUIDANCE_ENABLE=0` until sticks look correct in BF Receiver tab (dry mid), then set `1`

```bash
cd deploy
TRACKER=light ./start_tracking.sh restart
BF_GUIDANCE_ENABLE=1 BF_ARM_ENABLE=0 ./start_tracking.sh restart
DRON_ENABLE_CHASE=1 DRON_ENABLE_OSD=1 OSD_SINK=/dev/video10 ./start_tracking.sh restart
./start_tracking.sh status
./start_tracking.sh stop
```

Set `servo_pwmchip` in `chase.json` after `ls -l /sys/class/pwm/` (pwm3).

## Ports

| Port | Role |
|------|------|
| 8090 | Stats (+ Betaflight card) |
| 5005 / 5006 | LightTrack MJPEG / capture UI |
| 12345/udp | Tracker telem → **msp_betaflight** (or chase when `DRON_ENABLE_CHASE=1`) |
| 12346/udp | Chase → stats |
| 12350/udp | Chase → osd_overlay |
| 12347 / 12349 | Nano / Light cmd |
| `/dev/ttyS4` | MSP to Betaflight |

Health: `curl http://127.0.0.1:8090/stats.json` → `betaflight` object

## SD / old RPi

Recovered system snapshot: [reference/rpi_interceptor/](../reference/rpi_interceptor/).
Short notes: [opi5/FORENSICS.md](opi5/FORENSICS.md).
