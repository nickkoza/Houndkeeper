"""What the host says about itself ([4WJ]): its release, its services, CPU temperature, the Pi's throttling and
under-voltage flags (a sagging supply: the arm-on-board spec's [EkS]), and whether its clock is synchronised ([aVJ])."""
import subprocess, time
from pathlib import Path

THROTTLE_BITS = {0: "under_voltage_now", 1: "arm_freq_capped_now", 2: "throttled_now", 3: "soft_temp_limit_now",
                 16: "under_voltage_since_boot", 17: "arm_freq_capped_since_boot", 18: "throttled_since_boot",
                 19: "soft_temp_limit_since_boot"}   # vcgencmd get_throttled (Raspberry Pi documentation, checked 2026-10-02)


def throttle_flags(text):
    """'throttled=0x50005' -> each flag"""
    v = int(text.strip().split("=")[-1], 16) if text else 0
    return {name: bool(v >> bit & 1) for bit, name in THROTTLE_BITS.items()}


def read_throttled():
    try: return subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2).stdout
    except (OSError, subprocess.TimeoutExpired): return None


def read_cpu_temp_c():
    try: return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000.0
    except (OSError, ValueError): return None


def service_states(names):
    out = {}
    for n in names:
        try: out[n] = subprocess.run(["systemctl", "is-active", n], capture_output=True, text=True, timeout=2).stdout.strip() or "unknown"
        except (OSError, subprocess.TimeoutExpired): out[n] = "unknown"
    return out


def read_clock_synced():
    """timesyncd's word that the clock follows its source ([aVJ]); None off the Pi"""
    try: out = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"], capture_output=True, text=True,
                              timeout=2).stdout.strip()
    except (OSError, subprocess.TimeoutExpired): return None
    return {"yes": True, "no": False}.get(out)


def read_core_clock():
    """vcgencmd's core_freq and core_freq_min ('core_freq=250\ncore_freq_min=250'); None off the Pi"""
    try: return "".join(subprocess.run(["vcgencmd", "get_config", k], capture_output=True, text=True, timeout=2).stdout
                        for k in ("core_freq", "core_freq_min"))
    except (OSError, subprocess.TimeoutExpired): return None


def core_clock_fixed(text):
    """the Pi's SPI clock is divided from the core clock; WS2812 timing holds only when that clock cannot change
    (core_freq_min equal to core_freq). Unknown counts as not fixed."""
    v = {}
    for line in (text or "").splitlines():
        k, _, n = line.partition("=")
        try: v[k.strip()] = int(n)
        except ValueError: pass
    return v.get("core_freq", 0) > 0 and v.get("core_freq") == v.get("core_freq_min")


def report(release, services, cpu_temp_c, throttled, arm, speaker, clock_synced=None):
    return {"time": time.time(), "release": release, "services": services, "cpu_temp_c": cpu_temp_c,
            "throttle": throttle_flags(throttled) if throttled else None, "arm": arm, "speaker": speaker,
            "clock_synced": clock_synced}
