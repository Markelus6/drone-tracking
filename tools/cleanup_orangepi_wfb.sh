#!/bin/bash
# Run ON Orange Pi as root (or: sudo bash cleanup_orangepi_wfb.sh)
# Removes wfb-ng / rtl8812au FPV stack added for digital camera GS.
set -e
export DEBIAN_FRONTEND=noninteractive

echo "[1/6] stop services"
systemctl stop 'wifibroadcast@gs' 2>/dev/null || true
systemctl disable 'wifibroadcast@gs' 2>/dev/null || true
systemctl stop wifibroadcast 2>/dev/null || true
systemctl disable wifibroadcast 2>/dev/null || true
pkill -9 wfb_rx 2>/dev/null || true
pkill -9 wfb_tx 2>/dev/null || true
pkill -9 wfb_tun 2>/dev/null || true

echo "[2/6] udev / helpers"
rm -f /etc/udev/rules.d/*wfb* /etc/udev/rules.d/*8812* /etc/udev/rules.d/*rtl88* || true
rm -f /usr/local/bin/*wfb* /usr/local/sbin/*wfb* || true
udevadm control --reload-rules 2>/dev/null || true

echo "[3/6] packages"
apt-get remove -y wfb-ng wifibroadcast 2>/dev/null || true
apt-get purge -y wfb-ng wifibroadcast 2>/dev/null || true

echo "[4/6] dkms driver"
if command -v dkms >/dev/null; then
  dkms status || true
  dkms remove rtl8812au/5.2.20.2 --all 2>/dev/null || true
  dkms remove rtl8812au --all 2>/dev/null || true
fi
rmmod 88XXau_wfb 2>/dev/null || true
rmmod 88XXau 2>/dev/null || true
rmmod 8812au 2>/dev/null || true
rm -rf /usr/src/rtl8812au* /var/lib/dkms/rtl8812au /var/lib/dkms/88XXau_wfb || true
find /lib/modules -type f \( -name '*88XXau*' -o -name '*8812au*' \) -delete 2>/dev/null || true
depmod -a 2>/dev/null || true

echo "[5/6] configs / keys"
rm -f /etc/wifibroadcast.cfg /etc/wifibroadcast.cfg.bak /etc/default/wifibroadcast || true
rm -f /etc/gs.key /etc/drone.key || true
rm -rf /etc/wifibroadcast.d || true
rm -f /etc/systemd/system/wifibroadcast*.service || true
systemctl daemon-reload || true

echo "[6/6] home leftovers"
rm -rf /home/orangepi/rtl8812au /home/orangepi/wfb-ng || true
rm -f /home/orangepi/gs.key /home/orangepi/drone.key || true

echo "==== VERIFY ===="
systemctl list-unit-files 2>/dev/null | grep -iE 'wfb|wifibroadcast' || echo OK: no wfb units
command -v wfb_rx >/dev/null && echo WARN: wfb_rx still present || echo OK: no wfb_rx
ls /etc/gs.key /etc/wifibroadcast.cfg 2>/dev/null && echo WARN: files left || echo OK: no gs.key/cfg
lsmod | grep -iE '88|8812' || echo OK: no 88xx modules
echo DONE — Orange Pi cleaned for analog camera work
