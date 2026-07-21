#!/usr/bin/env python3
"""Sync interceptor sources to Orange Pi and prepare build/overlays."""
from __future__ import annotations

import sys
from pathlib import Path

import paramiko

HOST = "192.168.4.25"
USER = "orangepi"
PASS = "orangepi"
ROOT = Path(r"c:\Users\User\Desktop\перехватчик\drone-tracking")
REMOTE = "/home/orangepi/drone-tracking"


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=20)
    sftp = c.open_sftp()

    def ensure_dir(remote: str) -> None:
        parts = remote.strip("/").split("/")
        cur = ""
        for p in parts:
            cur += "/" + p
            try:
                sftp.stat(cur)
            except FileNotFoundError:
                try:
                    sftp.mkdir(cur)
                except OSError:
                    pass

    def put_file(local: Path, remote: str) -> None:
        ensure_dir(str(Path(remote).as_posix().rsplit("/", 1)[0]))
        sftp.put(str(local), remote)
        print("PUT", remote)

    for name in ["README.md", ".gitignore"]:
        p = ROOT / name
        if p.exists():
            put_file(p, f"{REMOTE}/{name}")

    for dname in ["control", "deploy"]:
        local_dir = ROOT / dname
        if not local_dir.exists():
            continue
        for path in local_dir.rglob("*"):
            if path.is_dir():
                continue
            if "build" in path.parts or "__pycache__" in path.parts:
                continue
            if path.suffix in {".o", ".a", ".pyc"}:
                continue
            rel = path.relative_to(ROOT).as_posix()
            put_file(path, f"{REMOTE}/{rel}")

    # Only text boot artifacts from legacy SD (not hundreds of RPi dtbo)
    legacy = ROOT / "legacy_rpi_sd"
    for name in ["README.md", "boot/config.txt", "boot/cmdline.txt", "boot/issue.txt"]:
        p = legacy / name
        if p.exists():
            put_file(p, f"{REMOTE}/legacy_rpi_sd/{name}")

    for remote in [
        f"{REMOTE}/deploy/build_all.sh",
        f"{REMOTE}/deploy/start_tracking.sh",
        f"{REMOTE}/deploy/opi5/install_overlays.sh",
        f"{REMOTE}/deploy/opi5/find_pwmchip.sh",
        f"{REMOTE}/control/build.sh",
    ]:
        try:
            sftp.chmod(remote, 0o755)
        except OSError as e:
            print("chmod skip", remote, e)

    sftp.close()
    print("SFTP done")

    def run(cmd: str, timeout: int = 600) -> tuple[int, str]:
        print(">>", cmd)
        _, stdout, stderr = c.exec_command(cmd, timeout=timeout)
        out = stdout.read().decode(errors="replace")
        err = stderr.read().decode(errors="replace")
        code = stdout.channel.recv_exit_status()
        sys.stdout.write(out)
        if err:
            sys.stdout.write("STDERR:\n" + err[:4000] + "\n")
        print("exit", code)
        return code, out

    run("ls -la ~/drone-tracking/control/src ~/drone-tracking/deploy/opi5 | head -50")
    run(
        "if [ -d /home/orangepi/ncnn-install ]; then echo NCNN=/home/orangepi/ncnn-install; "
        "elif [ -d /root/ncnn-install ]; then echo NCNN=/root/ncnn-install; "
        "else echo NCNN_MISSING; fi"
    )
    run("cat /boot/orangepiEnv.txt 2>/dev/null | head -20 || echo NO_ENV")
    run("ls /dev/ttyS* 2>/dev/null; ls /sys/class/pwm 2>/dev/null")

    # Prefer vendor overlays without reboot yet — still write env with sudo
    run(
        "echo orangepi | sudo -S bash -lc '"
        "ENV=/boot/orangepiEnv.txt; "
        "NEED=\"uart1-m1 uart3-m0 pwm3-m0\"; "
        "if [ -f \"$ENV\" ]; then "
        "  if grep -qE \"^overlays=\" \"$ENV\"; then "
        "    cur=$(grep -E \"^overlays=\" \"$ENV\" | tail -n1 | cut -d= -f2-); "
        "    merged=\"$cur\"; "
        "    for o in $NEED; do echo \" $merged \" | grep -q \" $o \" || merged=\"$merged $o\"; done; "
        "    sed -i -E \"s|^overlays=.*|overlays=${merged}|\" \"$ENV\"; "
        "  else echo \"overlays=$NEED\" >> \"$ENV\"; fi; "
        "  grep -E \"^overlays=\" \"$ENV\"; "
        "else echo NO_ORANGEPI_ENV; fi'"
    )

    # Patch NCNN_DIR in tracking CMake if needed, then build control (+ trackers if missing)
    run(
        "cd ~/drone-tracking && "
        "if [ -d /home/orangepi/ncnn-install ]; then "
        "  sed -i 's|set(NCNN_DIR \"/root/ncnn-install\")|set(NCNN_DIR \"/home/orangepi/ncnn-install\")|' tracking/CMakeLists.txt; "
        "fi; "
        "bash control/build.sh"
    )

    # Build orch/trackers if binaries missing
    run(
        "cd ~/drone-tracking; "
        "if [ ! -x tracking/build/lighttrack_fc ]; then bash deploy/build_all.sh; "
        "else echo trackers_ok; ls -la control/build/chase_fc control/build/osd_overlay tracking/build/lighttrack_fc 2>/dev/null; fi"
    )

    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
