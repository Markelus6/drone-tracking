# Deploy notes — Orange Pi 5 / RK3588 + Betaflight

## Layout on board

```text
/root/drone-tracking/
  orchestrator/build/orch_daemon
  tracking/build/{nanotrack_fc,lighttrack_fc}
  tracking/models/...
  deploy/
    start_tracking.sh
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
chmod +x deploy/start_tracking.sh deploy/build_all.sh
cp deploy/drone-tracking.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now drone-tracking
```

## Betaflight wiring

1. Orange Pi UART (default `/dev/ttyS4`) ↔ BF UART with **MSP** enabled (115200)
2. ELRS/CRSF on a **separate** BF UART
3. Modes: configure arm AUX (default CH5) and optionally MSP Override / Angle
4. Leave `BF_GUIDANCE_ENABLE=0` until sticks look correct in BF Receiver tab (dry mid), then set `1`

```bash
BF_GUIDANCE_ENABLE=1 BF_ARM_ENABLE=0 ./start_tracking.sh restart
```

## Ports

| Port | Role |
|------|------|
| 8090 | Stats (+ Betaflight card) |
| 5005 / 5006 | LightTrack MJPEG / capture UI |
| 12345/udp | Tracker telem → **msp_betaflight** |
| 12347 / 12349 | Nano / Light cmd |
| `/dev/ttyS4` | MSP to Betaflight |

Health: `curl http://127.0.0.1:8090/stats.json` → `betaflight` object
