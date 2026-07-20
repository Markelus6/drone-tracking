# Deploy notes — Orange Pi 5 / RK3588

## Layout on board

```text
/root/drone-tracking/
  orchestrator/build/orch_daemon
  tracking/build/{nanotrack_fc,lighttrack_fc}
  tracking/models/...
  deploy/
    start_tracking.sh
    stats_web.py          # HTTP :8090
    drone-tracking.service
    logs/
```

## First install

```bash
# on build host or on the board
scp -r drone-tracking root@<IP>:/root/

ssh root@<IP>
cd /root/drone-tracking
bash deploy/build_all.sh
chmod +x deploy/start_tracking.sh deploy/build_all.sh

# optional systemd
cp deploy/drone-tracking.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now drone-tracking
```

## Manual control

```bash
cd /root/drone-tracking/deploy
TRACKER=light ./start_tracking.sh restart   # or TRACKER=nano
./start_tracking.sh status
./start_tracking.sh stop
```

## Ports

| Port | Role |
|------|------|
| 8090 | Stats dashboard + `/stats.json` |
| 5003 / 5004 | NanoTrack MJPEG / capture UI |
| 5005 / 5006 | LightTrack MJPEG / capture UI |
| 12345/udp | Tracker telemetry (stats listens) |
| 12347 / 12349 | Nano / Light cmd |

Health: `curl http://127.0.0.1:8090/health` and `curl http://127.0.0.1:8090/stats.json`
