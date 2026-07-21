# Reference: original RPi interceptor system

Recovered plaintext + key files from the old Raspberry Pi SD (Гипертех-style stack).
Use this folder to understand how the previous system worked before porting to Orange Pi 5.

## Layout

| Path | What |
|------|------|
| `flight/start.sh` | Boot entry after fscrypt unlock — loops `./app` |
| `flight/config.ini` | Flight/camera gains (throttle, angular_*, cameras, tact) |
| `flight/app` | Main aarch64 binary (MSP + CRSF/crossfire + tracking) |
| `boot/config.txt` | RPi overlays: `uart2` (ELRS), `uart4` (FC), TV out |
| `boot/cmdline.txt` | Kernel cmdline |
| `crypto/` | SoC serial, sk, derived keyfile, `unlock` binary |
| `systemd/unlock.service` | Oneshot unlock of `/opt/data` at boot |
| `network/swc.nmconnection` | Onboard Wi‑Fi AP config |
| `docs/FORENSICS_REPORT.md` | Long forensics notes |

## How it ran (short)

1. Boot → `unlock.service` → `/usr/local/bin/unlock`
2. `unlock` reads SoC serial + `/etc/fscrypt.key`, writes interleaved key → `fscrypt unlock /opt/data`
3. Runs `/opt/data/start.sh` → `copters_onboard/build/app` with `config.ini`
4. `app` talks **CRSF** on `/dev/ttyAMA2`, **MSP** on `/dev/ttyAMA4` (also ACM*), cameras via V4L2

## Map to this repo (OPi5)

| Old RPi | This project |
|---------|----------------|
| `app` + `config.ini` | `control/chase_fc` + `deploy/chase.json` + trackers |
| `/dev/ttyAMA2` CRSF | `/dev/ttyS1` (uart1) |
| `/dev/ttyAMA4` MSP | `/dev/ttyS3` (uart3) |
| servo / OSD | `chase.json` PWM + MSP DisplayPort / `osd_overlay` |

## Serial note

Live SoC serial is `100000005334d949` (`crypto/rpi_serial.txt`).
The PCB Data Matrix value was **not** the SoC serial.
