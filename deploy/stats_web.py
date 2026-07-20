#!/usr/bin/env python3
"""
Stats web for drone-tracking — HTTP :8090 (dashboard + /stats.json).

Adapted from drone-perehvatchik flight_controller stats (STATS_WEB_*),
trimmed to the tracking-only stack: orch_daemon + nanotrack_fc / lighttrack_fc.

Listens UDP telemetry (VISION_TELEMETRY_PORT, default 12345) for live bbox/FPS.
If the port is already taken (e.g. flight controller), system/process stats still work.
"""

from __future__ import annotations

import http.server
import json
import os
import socket
import socketserver
import subprocess
import threading
import time
import urllib.parse
from typing import Any

STATS_WEB_PORT = int(os.environ.get("STATS_WEB_PORT", "8090"))
STATS_WEB_POLL_MS = max(20, int(os.environ.get("STATS_WEB_POLL_MS", "100")))
STATS_WEB_JSON_CACHE_MS = max(50, int(os.environ.get("STATS_WEB_JSON_CACHE_MS", "250")))
STATS_WEB_PREC_M = int(os.environ.get("STATS_WEB_PREC_M", "2"))
STATS_WEB_PREC_AGE = int(os.environ.get("STATS_WEB_PREC_AGE", "2"))
VISION_TELEMETRY_PORT = int(os.environ.get("VISION_TELEMETRY_PORT", "12345"))
NANOTRACK_CMD_PORT = int(os.environ.get("NANOTRACK_CMD_PORT", "12347"))
LIGHTTRACK_CMD_PORT = int(os.environ.get("LIGHTTRACK_CMD_PORT", "12349"))
NANOTRACK_VIZ_PORT = int(os.environ.get("NANOTRACK_VIZ_PORT", "5003"))
LIGHTTRACK_VIZ_PORT = int(os.environ.get("LIGHTTRACK_VIZ_PORT", "5005"))
NANOTRACK_UI_PORT = int(os.environ.get("NANOTRACK_UI_PORT", "5004"))
LIGHTTRACK_UI_PORT = int(os.environ.get("LIGHTTRACK_UI_PORT", "5006"))
LOGS_DIR = os.environ.get(
    "DRON_LOG_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"),
)

_MONITOR_PROC_KEYS = (
    ("orch", "orch_daemon"),
    ("nanotrack", "nanotrack_fc"),
    ("lighttrack", "lighttrack_fc"),
    ("stats", "stats_web.py"),
)

_telem_lock = threading.Lock()
_telem: dict[str, Any] = {
    "last_ts": 0.0,
    "packets": 0,
    "alive_ts": 0.0,
    "bbox_ts": 0.0,
    "lost_ts": 0.0,
    "tracker": None,
    "cam": None,
    "tracking": False,
    "bbox_norm": None,
    "conf": None,
    "fps": None,
    "track_ms_avg": None,
    "track_ms_max": None,
    "cx": None,
    "cy": None,
    "h": None,
    "score": None,
    "lost_streak": None,
    "last_raw": None,
}
_telem_bind_ok = False
_telem_bind_error: str | None = None

_stats_json_cache_lock = threading.Lock()
_stats_json_cache: tuple[float, bytes] | None = None
_cpu_stat_sample: dict[str, Any] = {}
_sys_mon_cache_ts = 0.0
_sys_mon_cache: dict = {}
_SYS_MON_CACHE_TTL_S = 1.0
_thermal_cache_ts = 0.0
_thermal_cache: list = []
_THERMAL_CACHE_TTL_S = 2.0
_http_cli_cache_ts = 0.0
_http_cli_cache: dict = {}
_HTTP_CLI_CACHE_TTL_S = 1.0
_logs_dir_cache_ts = 0.0
_logs_dir_cache_mb: float | None = None
_LOGS_DIR_CACHE_TTL_S = 30.0


def _proc_cmdline(pid: int) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return f.read().replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
    except OSError:
        return ""


def _find_pids_by_cmd_keyword(keyword: str) -> list[int]:
    hits: list[int] = []
    try:
        entries = os.listdir("/proc")
    except OSError:
        return hits
    for name in entries:
        if not name.isdigit():
            continue
        pid = int(name)
        if keyword in _proc_cmdline(pid):
            hits.append(pid)
    hits.sort()
    return hits


def _ps_metrics_for_pids(pids: list[int]) -> dict[int, dict]:
    if not pids:
        return {}
    try:
        proc = subprocess.run(
            ["ps", "-o", "pid=,pcpu=,rss=,nlwp=,etime=", "-p", ",".join(str(p) for p in pids)],
            capture_output=True,
            text=True,
            timeout=1.5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return {}
    if proc.returncode != 0:
        return {}
    out: dict[int, dict] = {}
    for line in (proc.stdout or "").splitlines():
        parts = line.split(None, 4)
        if len(parts) < 5:
            continue
        try:
            pid = int(parts[0])
            out[pid] = {
                "cpu_pct": float(parts[1]),
                "rss_mb": round(int(parts[2]) / 1024.0, 1),
                "threads": int(parts[3]),
                "etime": parts[4].strip(),
            }
        except (ValueError, IndexError):
            continue
    return out


def _read_meminfo_mb() -> dict:
    out = {
        "total_mb": None,
        "avail_mb": None,
        "used_mb": None,
        "used_pct": None,
        "swap_free_mb": None,
    }
    try:
        kv: dict[str, int] = {}
        with open("/proc/meminfo", "r", encoding="ascii") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 2 and parts[0].endswith(":"):
                    try:
                        kv[parts[0][:-1]] = int(parts[1])
                    except ValueError:
                        pass
        total_kb = kv.get("MemTotal")
        avail_kb = kv.get("MemAvailable") or kv.get("MemFree")
        if total_kb and total_kb > 0 and avail_kb is not None:
            used_kb = max(0, total_kb - avail_kb)
            out["total_mb"] = round(total_kb / 1024.0, 1)
            out["avail_mb"] = round(avail_kb / 1024.0, 1)
            out["used_mb"] = round(used_kb / 1024.0, 1)
            out["used_pct"] = round(100.0 * used_kb / total_kb, 1)
        swap_kb = kv.get("SwapFree")
        if swap_kb is not None:
            out["swap_free_mb"] = round(swap_kb / 1024.0, 1)
    except OSError:
        pass
    return out


def _read_cpu_usage_pct() -> float | None:
    global _cpu_stat_sample
    try:
        with open("/proc/stat", "r", encoding="ascii") as f:
            parts = f.readline().split()
        if len(parts) < 5 or parts[0] != "cpu":
            return _cpu_stat_sample.get("pct")
        nums = [int(x) for x in parts[1:]]
        idle = nums[3] + (nums[4] if len(nums) > 4 else 0)
        total = sum(nums)
        now = time.time()
        prev = _cpu_stat_sample.get("prev")
        if prev is not None:
            dt_total = total - int(prev["total"])
            dt_idle = idle - int(prev["idle"])
            if dt_total > 0 and (now - float(prev.get("ts", 0.0))) >= 0.2:
                pct = round(100.0 * (1.0 - dt_idle / dt_total), 1)
                _cpu_stat_sample["pct"] = max(0.0, min(100.0, pct))
        _cpu_stat_sample["prev"] = {"total": total, "idle": idle, "ts": now}
    except (OSError, ValueError):
        pass
    return _cpu_stat_sample.get("pct")


def _read_loadavg() -> dict:
    try:
        with open("/proc/loadavg", "r", encoding="ascii") as f:
            parts = f.read().split()
        if len(parts) >= 3:
            return {
                "load_1": float(parts[0]),
                "load_5": float(parts[1]),
                "load_15": float(parts[2]),
            }
    except (OSError, ValueError):
        pass
    return {"load_1": None, "load_5": None, "load_15": None}


def _read_system_uptime_s() -> float | None:
    try:
        with open("/proc/uptime", "r", encoding="ascii") as f:
            return round(float(f.read().split()[0]), 1)
    except (OSError, ValueError, IndexError):
        return None


def _read_cpu_freq_mhz() -> float | None:
    path = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
    try:
        with open(path, "r", encoding="ascii") as f:
            return round(int(f.read().strip()) / 1000.0, 0)
    except (OSError, ValueError):
        return None


def _disk_usage_mb(path: str) -> dict | None:
    try:
        du = os.statvfs(path)
        total = du.f_frsize * du.f_blocks
        avail = du.f_frsize * du.f_bavail
        used_pct = round(100.0 * (1.0 - avail / total), 1) if total > 0 else None
        return {
            "path": path,
            "total_mb": round(total / (1024 * 1024), 1),
            "avail_mb": round(avail / (1024 * 1024), 1),
            "used_pct": used_pct,
        }
    except OSError:
        return None


def _dir_size_mb(path: str) -> float | None:
    total = 0
    try:
        if not os.path.isdir(path):
            return None
        for root, _dirs, files in os.walk(path):
            for fn in files:
                fp = os.path.join(root, fn)
                try:
                    total += os.path.getsize(fp)
                except OSError:
                    pass
    except OSError:
        return None
    return round(total / (1024 * 1024), 2)


def _logs_dir_size_mb_cached(logs_dir: str) -> float | None:
    global _logs_dir_cache_ts, _logs_dir_cache_mb
    now = time.time()
    if (now - _logs_dir_cache_ts) < _LOGS_DIR_CACHE_TTL_S:
        return _logs_dir_cache_mb
    _logs_dir_cache_mb = _dir_size_mb(logs_dir)
    _logs_dir_cache_ts = now
    return _logs_dir_cache_mb


def _read_linux_thermal_zones_c() -> list:
    out: list = []
    base = "/sys/class/thermal"
    if not os.path.isdir(base):
        return out
    try:
        names = sorted(n for n in os.listdir(base) if n.startswith("thermal_zone"))
    except OSError:
        return out
    for zname in names:
        zdir = os.path.join(base, zname)
        try:
            with open(os.path.join(zdir, "temp"), "r", encoding="ascii") as f:
                mc = int(f.read().strip())
        except (OSError, ValueError):
            continue
        label = zname
        try:
            with open(os.path.join(zdir, "type"), "r", encoding="ascii") as f:
                label = f.read().strip() or zname
        except OSError:
            pass
        out.append({"zone": label, "id": zname, "c": round(mc / 1000.0, 1)})
    return out


def _read_linux_thermal_zones_c_cached() -> list:
    global _thermal_cache_ts, _thermal_cache
    now = time.time()
    if (now - _thermal_cache_ts) < _THERMAL_CACHE_TTL_S:
        return list(_thermal_cache)
    _thermal_cache = _read_linux_thermal_zones_c()
    _thermal_cache_ts = now
    return list(_thermal_cache)


def _count_established_tcp_by_local_port() -> tuple[dict[int, int], bool]:
    counts: dict[int, int] = {}
    try:
        proc = subprocess.run(
            ["ss", "-H", "-tn", "state", "established"],
            capture_output=True,
            text=True,
            timeout=1.5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return counts, False
    if proc.returncode != 0:
        return counts, False
    for line in (proc.stdout or "").splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        local = parts[2]
        if ":" not in local:
            continue
        try:
            port = int(local.rsplit(":", 1)[-1])
        except ValueError:
            continue
        counts[port] = counts.get(port, 0) + 1
    return counts, True


def _snapshot_http_clients_stats() -> dict:
    by_port, ss_ok = _count_established_tcp_by_local_port()
    c_nano = by_port.get(NANOTRACK_VIZ_PORT, 0)
    c_light = by_port.get(LIGHTTRACK_VIZ_PORT, 0)
    c_nano_ui = by_port.get(NANOTRACK_UI_PORT, 0)
    c_light_ui = by_port.get(LIGHTTRACK_UI_PORT, 0)
    c_stats = by_port.get(STATS_WEB_PORT, 0)
    return {
        "mjpeg_nano": c_nano,
        "mjpeg_nano_port": NANOTRACK_VIZ_PORT,
        "mjpeg_light": c_light,
        "mjpeg_light_port": LIGHTTRACK_VIZ_PORT,
        "ui_nano": c_nano_ui,
        "ui_nano_port": NANOTRACK_UI_PORT,
        "ui_light": c_light_ui,
        "ui_light_port": LIGHTTRACK_UI_PORT,
        "stats": c_stats,
        "stats_port": STATS_WEB_PORT,
        "video_total": c_nano + c_light,
        "web_total": c_nano + c_light + c_nano_ui + c_light_ui + c_stats,
        "ss_available": ss_ok,
    }


def _snapshot_http_clients_stats_cached() -> dict:
    global _http_cli_cache_ts, _http_cli_cache
    now = time.time()
    if _http_cli_cache and (now - _http_cli_cache_ts) < _HTTP_CLI_CACHE_TTL_S:
        return dict(_http_cli_cache)
    snap = _snapshot_http_clients_stats()
    _http_cli_cache = snap
    _http_cli_cache_ts = now
    return dict(snap)


def _snapshot_process_monitor() -> list:
    pids_by_key: dict[str, int] = {}
    for key, kw in _MONITOR_PROC_KEYS:
        found = _find_pids_by_cmd_keyword(kw)
        if found:
            pids_by_key[key] = found[0]
    my_pid = os.getpid()
    if "stats" not in pids_by_key:
        pids_by_key["stats"] = my_pid
    ps_map = _ps_metrics_for_pids(list(pids_by_key.values()))
    rows = []
    for key, kw in _MONITOR_PROC_KEYS:
        pid = pids_by_key.get(key)
        row: dict[str, Any] = {"key": key, "label": kw, "running": pid is not None, "pid": pid}
        if pid is not None:
            m = ps_map.get(pid, {})
            row.update(
                {
                    "cpu_pct": m.get("cpu_pct"),
                    "rss_mb": m.get("rss_mb"),
                    "threads": m.get("threads"),
                    "etime": m.get("etime"),
                }
            )
        rows.append(row)
    return rows


def _snapshot_system_monitoring() -> dict:
    global _sys_mon_cache_ts, _sys_mon_cache
    now = time.time()
    if _sys_mon_cache and (now - _sys_mon_cache_ts) < _SYS_MON_CACHE_TTL_S:
        return dict(_sys_mon_cache)
    mem = _read_meminfo_mb()
    load = _read_loadavg()
    cpu_n = os.cpu_count() or 1
    shm_path = "/dev/shm"
    snap = {
        "uptime_s": _read_system_uptime_s(),
        "cpu_count": cpu_n,
        "cpu_freq_mhz": _read_cpu_freq_mhz(),
        "load_1": load.get("load_1"),
        "load_5": load.get("load_5"),
        "load_15": load.get("load_15"),
        "load_per_cpu_1": (
            round(load["load_1"] / cpu_n, 2)
            if load.get("load_1") is not None and cpu_n > 0
            else None
        ),
        "mem_total_mb": mem.get("total_mb"),
        "mem_avail_mb": mem.get("avail_mb"),
        "mem_used_mb": mem.get("used_mb"),
        "mem_used_pct": mem.get("used_pct"),
        "cpu_usage_pct": _read_cpu_usage_pct(),
        "swap_free_mb": mem.get("swap_free_mb"),
        "stream_profile": (os.environ.get("DRON_STREAM_PROFILE") or "").strip() or None,
        "disk_root": _disk_usage_mb("/"),
        "disk_shm": _disk_usage_mb(shm_path) if os.path.exists(shm_path) else None,
        "logs_dir_mb": _logs_dir_size_mb_cached(LOGS_DIR),
        "logs_dir": LOGS_DIR,
        "processes": _snapshot_process_monitor(),
    }
    _sys_mon_cache = snap
    _sys_mon_cache_ts = now
    return dict(snap)


def _ingest_telem_packet(raw: str) -> None:
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return
    if not isinstance(data, dict):
        return
    now = time.time()
    with _telem_lock:
        _telem["last_ts"] = now
        _telem["packets"] = int(_telem.get("packets") or 0) + 1
        _telem["last_raw"] = data
        if data.get("tracker"):
            _telem["tracker"] = str(data["tracker"])
        if data.get("cam"):
            _telem["cam"] = str(data["cam"])
        if data.get("alive"):
            _telem["alive_ts"] = now
        if data.get("lost"):
            _telem["lost_ts"] = now
            _telem["tracking"] = False
            _telem["bbox_norm"] = None
        bbox = data.get("bbox_norm")
        if isinstance(bbox, list) and len(bbox) >= 4:
            _telem["bbox_ts"] = now
            _telem["bbox_norm"] = [float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])]
            _telem["tracking"] = True
        if "tracking" in data:
            _telem["tracking"] = bool(data["tracking"])
        for key in (
            "conf",
            "fps",
            "track_ms_avg",
            "track_ms_max",
            "cx",
            "cy",
            "h",
            "score",
            "lost_streak",
        ):
            if key in data and data[key] is not None:
                try:
                    _telem[key] = float(data[key])
                except (TypeError, ValueError):
                    pass


def _telem_listener_thread() -> None:
    global _telem_bind_ok, _telem_bind_error
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", VISION_TELEMETRY_PORT))
        _telem_bind_ok = True
        _telem_bind_error = None
        print(
            f"[stats_web] UDP telemetry listen :{VISION_TELEMETRY_PORT}",
            flush=True,
        )
    except OSError as e:
        _telem_bind_ok = False
        _telem_bind_error = str(e)
        print(
            f"[stats_web] UDP :{VISION_TELEMETRY_PORT} busy ({e}) — process stats only",
            flush=True,
        )
        sock.close()
        return
    sock.settimeout(1.0)
    while True:
        try:
            data, _addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            _ingest_telem_packet(data.decode("utf-8", errors="replace").strip())
        except Exception:
            continue


def _shm_cameras_present() -> list[str]:
    found: list[str] = []
    shm = "/dev/shm"
    if not os.path.isdir(shm):
        return found
    try:
        for name in sorted(os.listdir(shm)):
            if name.startswith("drone_cam"):
                found.append("/" + name if not name.startswith("/") else name)
    except OSError:
        pass
    return found


def _tracker_snapshot(now_ts: float) -> dict:
    with _telem_lock:
        t = dict(_telem)
    alive_age = float(now_ts - t["alive_ts"]) if t.get("alive_ts") else -1.0
    bbox_age = float(now_ts - t["bbox_ts"]) if t.get("bbox_ts") else -1.0
    telem_age = float(now_ts - t["last_ts"]) if t.get("last_ts") else -1.0
    lost_age = float(now_ts - t["lost_ts"]) if t.get("lost_ts") else -1.0
    name = t.get("tracker")
    nano_pids = _find_pids_by_cmd_keyword("nanotrack_fc")
    light_pids = _find_pids_by_cmd_keyword("lighttrack_fc")
    active = "nanotrack" if nano_pids else ("lighttrack" if light_pids else name)
    pid = None
    if active == "nanotrack" and nano_pids:
        pid = nano_pids[0]
    elif active == "lighttrack" and light_pids:
        pid = light_pids[0]
    metrics = _ps_metrics_for_pids([pid]) if pid else {}
    m = metrics.get(pid, {}) if pid else {}
    tracking = bool(t.get("tracking")) and bbox_age >= 0 and bbox_age < 1.5
    return {
        "name": active,
        "running": bool(nano_pids or light_pids),
        "pid": pid,
        "cpu_pct": m.get("cpu_pct"),
        "rss_mb": m.get("rss_mb"),
        "threads": m.get("threads"),
        "etime": m.get("etime"),
        "tracking": tracking,
        "cam": t.get("cam"),
        "bbox_norm": t.get("bbox_norm"),
        "conf": t.get("conf") if t.get("conf") is not None else t.get("score"),
        "fps": t.get("fps"),
        "track_ms_avg": t.get("track_ms_avg"),
        "track_ms_max": t.get("track_ms_max"),
        "cx": t.get("cx"),
        "cy": t.get("cy"),
        "h": t.get("h"),
        "score": t.get("score"),
        "lost_streak": t.get("lost_streak"),
        "alive_age_s": alive_age,
        "bbox_age_s": bbox_age,
        "telem_age_s": telem_age,
        "lost_age_s": lost_age,
        "packets": int(t.get("packets") or 0),
        "cmd_port": NANOTRACK_CMD_PORT if active == "nanotrack" else LIGHTTRACK_CMD_PORT,
        "viz_port": NANOTRACK_VIZ_PORT if active == "nanotrack" else LIGHTTRACK_VIZ_PORT,
        "ui_port": NANOTRACK_UI_PORT if active == "nanotrack" else LIGHTTRACK_UI_PORT,
        "nano_running": bool(nano_pids),
        "light_running": bool(light_pids),
    }


def _build_stats_json_snapshot() -> bytes:
    now_ts = time.time()
    orch_pids = _find_pids_by_cmd_keyword("orch_daemon")
    orch_metrics = _ps_metrics_for_pids(orch_pids[:1]) if orch_pids else {}
    orch_m = orch_metrics.get(orch_pids[0], {}) if orch_pids else {}
    s: dict[str, Any] = {
        "now_ts": now_ts,
        "project": "drone-tracking",
        "telem_listen": {
            "port": VISION_TELEMETRY_PORT,
            "ok": _telem_bind_ok,
            "error": _telem_bind_error,
        },
        "orchestrator": {
            "running": bool(orch_pids),
            "pid": orch_pids[0] if orch_pids else None,
            "cpu_pct": orch_m.get("cpu_pct"),
            "rss_mb": orch_m.get("rss_mb"),
            "threads": orch_m.get("threads"),
            "etime": orch_m.get("etime"),
            "shm_cameras": _shm_cameras_present(),
        },
        "tracker": _tracker_snapshot(now_ts),
        "ports": {
            "stats": STATS_WEB_PORT,
            "telemetry_udp": VISION_TELEMETRY_PORT,
            "nano_cmd": NANOTRACK_CMD_PORT,
            "light_cmd": LIGHTTRACK_CMD_PORT,
            "nano_mjpeg": NANOTRACK_VIZ_PORT,
            "light_mjpeg": LIGHTTRACK_VIZ_PORT,
            "nano_ui": NANOTRACK_UI_PORT,
            "light_ui": LIGHTTRACK_UI_PORT,
        },
    }
    temps = _read_linux_thermal_zones_c_cached()
    s["temperatures_c"] = temps
    s["temperature_max_c"] = max((t["c"] for t in temps), default=None)
    try:
        s["http_clients"] = _snapshot_http_clients_stats_cached()
    except Exception as e:
        s["http_clients"] = {"error": str(e), "ss_available": False}
    try:
        s["system"] = _snapshot_system_monitoring()
    except Exception as e:
        s["system"] = {"error": str(e)}
    return json.dumps(s, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def _stats_json_body_cached() -> bytes:
    global _stats_json_cache
    now = time.time()
    ttl_s = STATS_WEB_JSON_CACHE_MS / 1000.0
    with _stats_json_cache_lock:
        if _stats_json_cache is not None:
            ts, body = _stats_json_cache
            if (now - ts) < ttl_s:
                return body
    body = _build_stats_json_snapshot()
    with _stats_json_cache_lock:
        _stats_json_cache = (now, body)
    return body


def _dashboard_html() -> str:
    return f"""<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>drone-tracking stats</title>
  <style>
    body {{ font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; margin: 16px; background:#0f1115; color:#e8eaed; }}
    h2,h3 {{ margin: 0 0 8px; }}
    a {{ color:#8ab4f8; }}
    .grid {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); gap: 12px; }}
    .card {{ border: 1px solid #2a2f3a; border-radius: 8px; padding: 12px; background:#161a22; }}
    .k {{ color: #9aa0a6; font-size: 12px; text-transform: uppercase; letter-spacing: .04em; }}
    .v {{ font-weight: 600; margin-top: 6px; word-break: break-all; }}
    .badge {{ display: inline-block; padding: 2px 8px; border-radius: 999px; font-size: 12px; font-weight: 700; }}
    .ok {{ background: #143d2a; color: #81c995; }}
    .warn {{ background: #3d3014; color: #fdd663; }}
    .bad {{ background: #3d1418; color: #f28b82; }}
    .muted {{ background: #2a2f3a; color: #9aa0a6; }}
    pre {{ background: #0b0d12; padding: 12px; border-radius: 8px; overflow: auto; font-size: 12px; border:1px solid #2a2f3a; }}
  </style>
</head>
<body>
  <h2>drone-tracking</h2>
  <p style="color:#9aa0a6;font-size:14px;">Порт {STATS_WEB_PORT} · опрос {STATS_WEB_POLL_MS} ms · <code>/stats.json</code></p>
  <div class="grid">
    <div class="card"><div class="k">Трекер</div><div class="v" id="tracker">-</div></div>
    <div class="card"><div class="k">Телеметрия / bbox</div><div class="v" id="telem">-</div></div>
    <div class="card"><div class="k">Orchestrator / SHM</div><div class="v" id="orch">-</div></div>
    <div class="card"><div class="k">Температуры</div><div class="v" id="temps">-</div></div>
    <div class="card"><div class="k">HTTP клиенты</div><div class="v" id="httpcli">-</div></div>
    <div class="card"><div class="k">Память / нагрузка</div><div class="v" id="sysmem">-</div></div>
    <div class="card"><div class="k">Диски</div><div class="v" id="sysdisk">-</div></div>
    <div class="card"><div class="k">Процессы стека</div><div class="v" id="sysproc">-</div></div>
    <div class="card"><div class="k">Порты</div><div class="v" id="ports">-</div></div>
  </div>
  <h3 style="margin-top:16px;">Raw JSON</h3>
  <pre id="raw">Loading...</pre>
  <script>
    function ageStr(sec, prec) {{
      if (sec == null || !isFinite(sec) || sec < 0) return "—";
      return sec.toFixed(prec) + " s";
    }}
    function badge(cls, text) {{
      return '<span class="badge ' + cls + '">' + text + '</span>';
    }}
    function escapeHtml(str) {{
      return String(str).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
    }}
    async function tick() {{
      const r = await fetch('/stats.json', {{cache: 'no-store'}});
      const s = await r.json();
      document.getElementById('raw').textContent = JSON.stringify(s, null, 2);
      const pa = {STATS_WEB_PREC_AGE};
      const tr = s.tracker || {{}};
      let trHtml = '';
      if (!tr.running) trHtml = badge('bad', 'не запущен');
      else if (tr.tracking) trHtml = badge('ok', 'tracking') + ' ' + badge('muted', escapeHtml(tr.name || '?'));
      else trHtml = badge('warn', 'ожидание цели') + ' ' + badge('muted', escapeHtml(tr.name || '?'));
      if (tr.pid) trHtml += '<br/><small>pid ' + tr.pid + '</small>';
      if (tr.fps != null) trHtml += '<br/><small>FPS ' + Number(tr.fps).toFixed(1) +
        ' · track ' + Number(tr.track_ms_avg ?? 0).toFixed(1) + '/' + Number(tr.track_ms_max ?? 0).toFixed(1) + ' ms</small>';
      if (tr.cpu_pct != null) trHtml += '<br/><small>CPU ' + Number(tr.cpu_pct).toFixed(1) +
        '% · RSS ' + Number(tr.rss_mb ?? 0).toFixed(0) + ' MB</small>';
      document.getElementById('tracker').innerHTML = trHtml;

      const listenOk = s.telem_listen && s.telem_listen.ok;
      let teHtml = listenOk ? badge('ok', 'UDP listen') : badge('warn', 'UDP offline');
      teHtml += '<br/>telem age=' + ageStr(tr.telem_age_s, pa) + ' · packets ' + (tr.packets ?? 0);
      teHtml += '<br/>bbox age=' + ageStr(tr.bbox_age_s, pa) + ' · alive=' + ageStr(tr.alive_age_s, pa);
      if (tr.bbox_norm) {{
        teHtml += '<br/><small>bbox [' + tr.bbox_norm.map(function(x){{return Number(x).toFixed(3);}}).join(', ') +
          '] conf=' + Number(tr.conf ?? tr.score ?? 0).toFixed(3) + '</small>';
      }} else if (tr.lost_age_s >= 0 && tr.lost_age_s < 5) {{
        teHtml += '<br/>' + badge('bad', 'lost');
      }}
      document.getElementById('telem').innerHTML = teHtml;

      const orch = s.orchestrator || {{}};
      let oHtml = orch.running ? badge('ok', 'orch pid ' + orch.pid) : badge('bad', 'orch нет');
      const cams = orch.shm_cameras || [];
      if (cams.length) {{
        cams.forEach(function(c) {{ oHtml += '<br/><small>' + escapeHtml(c) + '</small>'; }});
      }} else {{
        oHtml += '<br/>' + badge('muted', 'нет SHM');
      }}
      document.getElementById('orch').innerHTML = oHtml;

      function tempBadge(c) {{
        if (c == null || !isFinite(c)) return badge('muted', 'N/A');
        if (c >= 85) return badge('bad', c.toFixed(1) + ' °C');
        if (c >= 70) return badge('warn', c.toFixed(1) + ' °C');
        return badge('ok', c.toFixed(1) + ' °C');
      }}
      let tHtml = '';
      const temps = s.temperatures_c || [];
      if (!temps.length) tHtml = badge('muted', 'Нет данных');
      else {{
        if (s.temperature_max_c != null) tHtml += tempBadge(Number(s.temperature_max_c)) + ' max<br/>';
        temps.forEach(function(z) {{
          tHtml += '<small>' + escapeHtml(z.zone || z.id || '?') + ': ' + Number(z.c).toFixed(1) + ' °C</small><br/>';
        }});
      }}
      document.getElementById('temps').innerHTML = tHtml;

      const hc = s.http_clients || {{}};
      let hHtml = '';
      if (hc.error) hHtml = badge('bad', escapeHtml(hc.error));
      else if (!hc.ss_available) hHtml = badge('muted', 'ss недоступен');
      else {{
        hHtml += badge(hc.video_total > 0 ? 'ok' : 'muted', 'видео ' + (hc.video_total ?? 0)) +
          ' ' + badge('muted', 'всего ' + (hc.web_total ?? 0)) + '<br/>';
        [
          ['MJPEG nano', hc.mjpeg_nano_port, hc.mjpeg_nano],
          ['MJPEG light', hc.mjpeg_light_port, hc.mjpeg_light],
          ['UI nano', hc.ui_nano_port, hc.ui_nano],
          ['UI light', hc.ui_light_port, hc.ui_light],
          ['stats', hc.stats_port, hc.stats],
        ].forEach(function(row) {{
          hHtml += '<small>' + escapeHtml(row[0]) + ' :' + row[1] + ' → ' + row[2] + '</small><br/>';
        }});
      }}
      document.getElementById('httpcli').innerHTML = hHtml;

      const sys = s.system || {{}};
      let memHtml = '';
      if (sys.error) memHtml = badge('bad', escapeHtml(sys.error));
      else {{
        const cpuPct = sys.cpu_usage_pct;
        if (cpuPct != null) {{
          const ccls = cpuPct >= 90 ? 'bad' : (cpuPct >= 70 ? 'warn' : 'ok');
          memHtml += badge(ccls, 'CPU ' + Number(cpuPct).toFixed(0) + '%') + '<br/>';
        }}
        if (sys.load_1 != null) {{
          memHtml += badge('muted', 'load ' + Number(sys.load_1).toFixed(2)) +
            ' <small>(' + Number(sys.load_per_cpu_1).toFixed(2) + '/cpu)</small><br/>';
        }}
        if (sys.mem_used_mb != null && sys.mem_total_mb != null) {{
          const mp = sys.mem_used_pct;
          const mcls = (mp != null && mp >= 90) ? 'bad' : ((mp != null && mp >= 75) ? 'warn' : 'ok');
          memHtml += badge(mcls, 'RAM ' + Number(sys.mem_used_mb).toFixed(0) +
            ' / ' + Number(sys.mem_total_mb).toFixed(0) + ' MB');
          if (mp != null) memHtml += ' <small>(' + Number(mp).toFixed(0) + '%)</small>';
          memHtml += '<br/>';
        }}
        if (sys.uptime_s != null) {{
          const uh = Math.floor(sys.uptime_s / 3600);
          const um = Math.floor((sys.uptime_s % 3600) / 60);
          memHtml += '<small>uptime: ' + uh + 'ч ' + um + 'м</small><br/>';
        }}
      }}
      if (!memHtml) memHtml = badge('muted', 'Нет данных');
      document.getElementById('sysmem').innerHTML = memHtml;

      let diskHtml = '';
      if (!sys.error) {{
        function diskLine(label, d) {{
          if (!d) return '';
          return '<small>' + escapeHtml(label) + ': ' + Number(d.avail_mb).toFixed(0) +
            ' / ' + Number(d.total_mb).toFixed(0) + ' MB (' + Number(d.used_pct).toFixed(0) + '%)</small><br/>';
        }}
        diskHtml += diskLine('root', sys.disk_root);
        diskHtml += diskLine('shm', sys.disk_shm);
        if (sys.logs_dir_mb != null) diskHtml += '<small>logs: ' + Number(sys.logs_dir_mb).toFixed(1) + ' MB</small><br/>';
      }}
      if (!diskHtml) diskHtml = badge('muted', 'Нет данных');
      document.getElementById('sysdisk').innerHTML = diskHtml;

      let procHtml = '';
      (sys.processes || []).forEach(function(p) {{
        if (!p.running) {{
          procHtml += '<small>' + escapeHtml(p.label) + ': ' + badge('bad', 'нет') + '</small><br/>';
          return;
        }}
        procHtml += '<small>' + escapeHtml(p.label) + ' pid ' + p.pid +
          ' · RSS ' + (p.rss_mb != null ? Number(p.rss_mb).toFixed(0) + ' MB' : '—') +
          ' · CPU ' + (p.cpu_pct != null ? Number(p.cpu_pct).toFixed(1) + '%' : '—') +
          (p.etime ? ' · ' + escapeHtml(p.etime) : '') + '</small><br/>';
      }});
      if (!procHtml) procHtml = badge('muted', 'Нет данных');
      document.getElementById('sysproc').innerHTML = procHtml;

      const ports = s.ports || {{}};
      let pHtml = '';
      Object.keys(ports).forEach(function(k) {{
        pHtml += '<small>' + escapeHtml(k) + ': ' + ports[k] + '</small><br/>';
      }});
      if (tr.viz_port) {{
        pHtml += '<br/><a href="http://' + location.hostname + ':' + tr.viz_port + '/" target="_blank">MJPEG</a> · ';
        pHtml += '<a href="http://' + location.hostname + ':' + tr.ui_port + '/" target="_blank">Capture UI</a>';
      }}
      document.getElementById('ports').innerHTML = pHtml || badge('muted', '—');
    }}
    setInterval(() => tick().catch(()=>{{}}), {STATS_WEB_POLL_MS});
    tick();
  </script>
</body>
</html>
"""


def run_server() -> None:
    os.makedirs(LOGS_DIR, exist_ok=True)
    threading.Thread(target=_telem_listener_thread, daemon=True, name="TelemUDP").start()
    # Warm CPU sample
    _read_cpu_usage_pct()
    time.sleep(0.25)
    _read_cpu_usage_pct()

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:
            return

        def _send(self, code: int, content_type: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            parsed = urllib.parse.urlparse(self.path)
            path = parsed.path or "/"
            if path in ("/stats", "/stats.json"):
                return self._send(200, "application/json; charset=utf-8", _stats_json_body_cached())
            if path in ("/", "/index.html"):
                html = _dashboard_html().encode("utf-8")
                return self._send(200, "text/html; charset=utf-8", html)
            if path in ("/health", "/healthz"):
                body = b'{"ok":true}\n'
                return self._send(200, "application/json; charset=utf-8", body)
            return self._send(404, "text/plain; charset=utf-8", b"Not found\n")

    class _Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    try:
        with _Server(("", STATS_WEB_PORT), Handler) as httpd:
            print(
                f"[stats_web] http://0.0.0.0:{STATS_WEB_PORT}/ "
                f"(poll={STATS_WEB_POLL_MS}ms cache={STATS_WEB_JSON_CACHE_MS}ms)",
                flush=True,
            )
            httpd.serve_forever()
    except OSError as e:
        print(f"[stats_web] port {STATS_WEB_PORT} unavailable: {e}", flush=True)
        raise SystemExit(1) from e


if __name__ == "__main__":
    run_server()
