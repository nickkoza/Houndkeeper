"""The host's report [4WJ] (its values, and that it goes to the laptop at least once a second) and updates only on
purpose [871] (the operator's stowed-and-still confirmation; the gate the host itself does not yet have).
Laptop-only: no Pi, no network beyond a fake socket."""
import json
import subprocess
import threading
import time
from pathlib import Path

import pytest

from arm_host import health, hostd

ARM_HOST = Path(hostd.__file__).resolve().parent


class _Run:
    """a stand-in for subprocess.run: systemctl and vcgencmd answer as a Pi would"""
    def __init__(self, states, throttled):
        self.states, self.throttled = states, throttled
    def __call__(self, args, **kw):
        if args[:2] == ["systemctl", "is-active"]: out = self.states.get(args[2], "") + "\n"
        elif args[:2] == ["vcgencmd", "get_throttled"]: out = self.throttled + "\n"
        else: raise OSError(args[0])
        return subprocess.CompletedProcess(args, 0, stdout=out, stderr="")


def _pi(monkeypatch, tmp_path, states, throttled, millideg="61834"):
    monkeypatch.setattr(health.subprocess, "run", _Run(states, throttled))
    temp = tmp_path / "temp"; temp.write_text(millideg + "\n")
    real_path = health.Path
    monkeypatch.setattr(health, "Path", lambda p: temp if p == "/sys/class/thermal/thermal_zone0/temp" else real_path(p))


def test_the_report_carries_the_hosts_actual_values(monkeypatch, tmp_path):
    """[4WJ] release, each service's state, CPU temperature in C, the throttle and under-voltage flags as the Pi
    reports them, the arm and the speaker"""
    _pi(monkeypatch, tmp_path, {"hk-hostd": "active", "hk-arm": "failed"}, "throttled=0x50005")
    h = hostd.Host(("127.0.0.1", 9), arm_present=lambda: True, speaker_connected=lambda: False, release="42",
                   services=("hk-hostd", "hk-arm", "hk-wrist"))
    r = h.health()
    assert r["release"] == "42"
    assert r["services"] == {"hk-hostd": "active", "hk-arm": "failed", "hk-wrist": "unknown"}
    assert r["cpu_temp_c"] == pytest.approx(61.834)
    assert r["throttle"]["under_voltage_now"] and r["throttle"]["throttled_now"] and r["throttle"]["under_voltage_since_boot"]
    assert not r["throttle"]["soft_temp_limit_now"] and not r["throttle"]["arm_freq_capped_now"]
    assert r["arm"] == "present" and r["speaker"] == "disconnected"
    assert abs(r["time"] - time.time()) < 5
    json.dumps(r)                                                       # it goes over the wire as JSON


def test_a_healthy_pi_reports_no_flags_and_a_missing_sensor_reports_none(monkeypatch, tmp_path):
    """[4WJ] a clear supply reads clear (not missing); off the Pi, temperature and throttle are None, not made up"""
    _pi(monkeypatch, tmp_path, {"hk-hostd": "active"}, "throttled=0x0")
    r = hostd.Host(("127.0.0.1", 9), lambda: False, lambda: True).health()
    assert r["throttle"] is not None and not any(r["throttle"].values()) and r["services"] == {"hk-hostd": "active"}
    assert r["arm"] == "absent" and r["speaker"] == "connected"
    def off_the_pi(*a, **k): raise OSError("no vcgencmd or systemctl")
    monkeypatch.setattr(health.subprocess, "run", off_the_pi)
    monkeypatch.setattr(health, "Path", lambda p: tmp_path / "absent")
    r = hostd.Host(("127.0.0.1", 9), lambda: False, lambda: True).health()
    assert r["cpu_temp_c"] is None and r["throttle"] is None and r["services"] == {"hk-hostd": "unknown"}


class _Sock:
    sent = []
    def __init__(self, *a): pass
    def sendto(self, data, addr): _Sock.sent.append((time.monotonic(), json.loads(data), addr))


class _Server:
    """stands in for the HTTP server: lets the daemon run for `run_s`, then stops it as systemd would"""
    run_s = 3.6
    def __init__(self, *a): pass
    def serve_forever(self): time.sleep(_Server.run_s); raise SystemExit(0)


def test_the_report_goes_to_the_laptop_at_least_once_a_second(monkeypatch, tmp_path):
    """[4WJ] the daemon's own loop: a JSON report to the laptop's address and report port, every second or sooner"""
    cfg = tmp_path / "host.json"
    cfg.write_text(json.dumps({"laptop_ip": "192.0.2.77", "report_port": 47999,
                               "features": {"voice": False, "ears": False, "eyes": False}}))
    monkeypatch.setattr(hostd, "CONFIG", cfg)
    monkeypatch.setattr(hostd.socket, "socket", _Sock); _Sock.sent = []
    monkeypatch.setattr(hostd, "ThreadingHTTPServer", _Server)
    monkeypatch.setattr(hostd.signal, "signal", lambda *a: None)
    _pi(monkeypatch, tmp_path, {"hk-hostd": "active"}, "throttled=0x0")
    t0 = time.monotonic()
    with pytest.raises(SystemExit): hostd.main()
    sent = list(_Sock.sent); _Sock.sent = []
    assert len(sent) >= 3, len(sent)
    assert all(addr == ("192.0.2.77", 47999) for _t, _r, addr in sent)
    times = [t0] + [t for t, _r, _a in sent]
    assert max(b - a for a, b in zip(times, times[1:])) <= 1.15, times
    assert {"release", "services", "cpu_temp_c", "throttle", "arm", "speaker", "time"} <= set(sent[-1][1])
    assert sent[-1][1]["services"] == {"hk-hostd": "active"}


def test_a_push_is_refused_unless_the_operator_confirms_the_robot_is_still_and_the_arm_stowed(tmp_path):
    """[871] deploy.sh stops before building or sending anything without --robot-still (today the operator's
    confirmation stands in for the arm adapter's stowed state)"""
    for args in ([], ["--yes"], ["--robot-stil"]):
        r = subprocess.run(["/bin/bash", str(ARM_HOST / "deploy.sh"), "pi.invalid", *args], capture_output=True, text=True,
                           timeout=10, cwd=tmp_path, env={"PATH": str(tmp_path)})   # no ssh, scp or tar to reach for
        assert r.returncode == 2 and "--robot-still" in r.stdout, (args, r.stdout, r.stderr)


@pytest.mark.xfail(strict=True, reason="[871] not built: release.activate has no stowed/docked gate; the arm adapter "
                   "does not report its state yet, so only deploy.sh's --robot-still (an operator's word) stands for it")
def test_the_host_itself_refuses_an_update_while_the_arm_is_out(tmp_path):
    """[871] a release is applied only while the arm is stowed and the robot docked or still: the host's own check,
    not only the operator's flag"""
    from arm_host import release
    for v in ("1", "2"):
        d = tmp_path / "releases" / v; d.mkdir(parents=True); (d / "check.sh").write_text("#!/bin/sh\nexit 0\n"); (d / "check.sh").chmod(0o755)
    release.activate(tmp_path, "1", restart=lambda: None)
    import inspect
    assert any(p in inspect.signature(release.activate).parameters for p in ("arm_stowed", "stowed", "robot_still")), \
        "activate takes no arm or robot state to refuse on"


def test_nothing_on_the_host_updates_it_by_itself():
    """[871] the host never updates itself: the image's set-up turns off apt's own timers"""
    text = "".join(p.read_text() for p in ARM_HOST.glob("*.sh")) + "".join(p.read_text() for p in (ARM_HOST / "systemd").iterdir())
    assert "apt-daily" in text and "mask" in text
