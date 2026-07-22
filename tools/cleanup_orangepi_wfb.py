#!/usr/bin/env python3
"""Remove wfb-ng / rtl8812au FPV stack we added on Orange Pi."""
from __future__ import annotations

import socket
import sys
import time

import paramiko

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

HOST = "192.168.4.25"
USER = "orangepi"
PASSWORD = "orangepi"

CLEANUP = r"""
set -x
export DEBIAN_FRONTEND=noninteractive

# stop / disable services
systemctl stop 'wifibroadcast@gs' 2>/dev/null || true
systemctl disable 'wifibroadcast@gs' 2>/dev/null || true
systemctl stop wifibroadcast 2>/dev/null || true
systemctl disable wifibroadcast 2>/dev/null || true
systemctl stop wfb-ng 2>/dev/null || true
systemctl disable wfb-ng 2>/dev/null || true

# kill leftover processes
pkill -9 wfb_rx 2>/dev/null || true
pkill -9 wfb_tx 2>/dev/null || true
pkill -9 wfb_tun 2>/dev/null || true
pkill -9 wfb-cli 2>/dev/null || true

# udev / helper scripts we added
rm -f /etc/udev/rules.d/*wfb* /etc/udev/rules.d/*8812* /etc/udev/rules.d/*rtl88* 2>/dev/null || true
rm -f /usr/local/bin/*wfb* /usr/local/sbin/*wfb* 2>/dev/null || true
rm -f /usr/local/bin/start*wfb* /usr/local/bin/*wifibroadcast* 2>/dev/null || true
udevadm control --reload-rules 2>/dev/null || true

# packages
if command -v apt-get >/dev/null; then
  apt-get remove -y wfb-ng wifibroadcast 2>/dev/null || true
  apt-get purge -y wfb-ng wifibroadcast 2>/dev/null || true
fi

# dkms driver rtl8812au / 88XXau_wfb
if command -v dkms >/dev/null; then
  dkms status || true
  for name in rtl8812au 88XXau_wfb 8812au; do
    for ver in $(dkms status 2>/dev/null | awk -F'[,/ ]+' -v n="$name" '$1==n {print $2}' | sort -u); do
      dkms remove -m "$name" -v "$ver" --all 2>/dev/null || true
    done
  done
fi
rmmod 88XXau_wfb 2>/dev/null || true
rmmod 88XXau 2>/dev/null || true
rmmod 8812au 2>/dev/null || true

# source trees / modules
rm -rf /usr/src/rtl8812au* /usr/src/88XXau* 2>/dev/null || true
rm -rf /var/lib/dkms/rtl8812au /var/lib/dkms/88XXau_wfb 2>/dev/null || true
find /lib/modules -type f \( -name '*88XXau*' -o -name '*8812au*' -o -name '*rtl8812*' \) -delete 2>/dev/null || true
depmod -a 2>/dev/null || true

# configs and keys we installed
rm -f /etc/wifibroadcast.cfg /etc/wifibroadcast.cfg.bak 2>/dev/null || true
rm -f /etc/default/wifibroadcast 2>/dev/null || true
rm -f /etc/gs.key /etc/drone.key 2>/dev/null || true
rm -rf /etc/wifibroadcast.d 2>/dev/null || true
rm -f /etc/systemd/system/wifibroadcast*.service /etc/systemd/system/wfb*.service 2>/dev/null || true
rm -f /lib/systemd/system/wifibroadcast*.service 2>/dev/null || true
systemctl daemon-reload 2>/dev/null || true

# home leftovers
rm -rf /home/orangepi/rtl8812au /home/orangepi/wfb-ng /home/orangepi/wfb* 2>/dev/null || true
rm -f /home/orangepi/gs.key /home/orangepi/drone.key 2>/dev/null || true
rm -rf /tmp/rtl8812au /tmp/wfb-ng /tmp/gs.key 2>/dev/null || true

echo '==== AFTER CLEANUP ===='
systemctl list-unit-files 2>/dev/null | grep -iE 'wfb|wifibroadcast' || echo '(no wfb units)'
dkms status 2>/dev/null || echo '(no dkms)'
command -v wfb_rx || echo '(no wfb_rx)'
ls /etc/gs.key /etc/wifibroadcast.cfg 2>/dev/null || echo '(no gs.key / wifibroadcast.cfg)'
lsmod | grep -iE '88|8812' || echo '(no 88xx modules loaded)'
echo DONE
"""


def try_connect(host: str) -> paramiko.SSHClient:
    sock = socket.create_connection((host, 22), timeout=8)
    t = paramiko.Transport(sock)
    t.banner_timeout = 60
    t.auth_timeout = 30
    t.start_client(timeout=30)
    t.auth_password(USER, PASSWORD)
    c = paramiko.SSHClient()
    c._transport = t
    return c


def main() -> int:
    # quick TCP check
    print(f"probing {HOST}:22 ...")
    try:
        s = socket.create_connection((HOST, 22), timeout=5)
        s.close()
        print("TCP 22 open")
    except OSError as e:
        print(f"SSH port closed/unreachable: {e}")
        print("Включи Orange Pi в той же Wi‑Fi сети (раньше IP был 192.168.4.25) и напиши.")
        return 2

    last_err = None
    for attempt in range(1, 4):
        try:
            print(f"SSH attempt {attempt}...")
            c = try_connect(HOST)
            break
        except Exception as e:
            last_err = e
            print("fail:", e)
            time.sleep(2)
    else:
        print("SSH failed:", last_err)
        return 1

    cmd = f"echo {PASSWORD} | sudo -S bash -lc {CLEANUP!r}"
    print("running cleanup...")
    _i, o, e = c.exec_command(cmd, timeout=300, get_pty=True)
    print(o.read().decode(errors="replace"))
    err = e.read().decode(errors="replace")
    if err.strip():
        print("stderr:", err)
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
