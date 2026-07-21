# Deploy notes — Orange Pi 5 / RK3588 interceptor

## Layout on board

```text
drone-tracking/
  orchestrator/build/orch_daemon
  tracking/build/{nanotrack_fc,lighttrack_fc}
  control/build/{chase_fc,osd_overlay}
  deploy/
    start_tracking.sh
    chase.json
    opi5/WIRING.md
    stats_web.py
    logs/
```

## First install

```bash
cd ~/drone-tracking
bash deploy/build_all.sh
chmod +x deploy/*.sh deploy/opi5/*.sh control/build.sh
sudo bash deploy/opi5/install_overlays.sh
sudo reboot
# after reboot: wire ELRS/FC/servo per deploy/opi5/WIRING.md
# Betaflight: MSP on UART to Pi, Serial RX off, Receiver = MSP
```

## Manual control

```bash
cd deploy
TRACKER=light ./start_tracking.sh restart
DRON_ENABLE_CHASE=1 DRON_ENABLE_OSD=1 OSD_SINK=/dev/video10 ./start_tracking.sh restart
./start_tracking.sh status
./start_tracking.sh stop
```

Set `servo_pwmchip` in `chase.json` after `ls -l /sys/class/pwm/` (pwm3).

## Ports

| Port | Role |
|------|------|
| 8090 | Stats dashboard |
| 5005 / 5006 | LightTrack MJPEG / capture UI |
| 12345/udp | Tracker → chase (when chase on) |
| 12346/udp | Chase → stats |
| 12350/udp | Chase → osd_overlay |
| 12349 | LightTrack cmd |

## SD / old RPi

Recovered system snapshot: [reference/rpi_interceptor/](../reference/rpi_interceptor/).
Short notes: [opi5/FORENSICS.md](opi5/FORENSICS.md).
