"""[vRJ] [F2q]: the phone cockpit's procedures run from the page's commands on the robot's own streams, show live
numbers, give a result beside the file's value, and write only on confirm; the Status page's link numbers."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "src")); sys.path.insert(0, str(HK / "real" / "calibration"))
pytest.importorskip("dimos")                                                   # no DimOS here: skip; a broken module: fail
import houndkeeper.cockpit_data as cd  # noqa: E402


def harness(tmp_path, monkeypatch):
    """a CockpitData without the DimOS run: its buffers fed by hand, params and the runs log in tmp_path"""
    import measure
    params = tmp_path / "robot.yaml"
    params.write_text("go2_air:\n  walk_power_W:      {value: 170,   source: estimate, note: \"est.\"}\n"
                      "  video_stream:      {value: {drop: 0.03, stall_per_min: 2, stall_s: 0.6}, source: estimate, note: \"est.\"}\n")
    (tmp_path / "params").mkdir(); (tmp_path / "params" / "robot.yaml").symlink_to(params)
    monkeypatch.setattr(measure, "PARAMS", params); monkeypatch.setattr(cd, "HK", tmp_path)
    monkeypatch.setattr(cd, "RUNS_LOG", tmp_path / "runs.jsonl")
    m = object.__new__(cd.CockpitData)
    m._buf = cd.Buffers(); m._state = cd.ProcedureState(); m._t0 = 0.0; m._truth = {}; m._pending = {}; m._frame_n = 0
    return m, params


def test_a_stream_run_measures_drops_and_writes_only_on_confirm(tmp_path, monkeypatch):
    m, params = harness(tmp_path, monkeypatch)
    m.handle({"op": "start", "id": "stream"}, now=1000.0)
    assert m._state.phase == "recording"
    rng = np.random.default_rng(0); t = np.arange(1000.0, 1600.0, 1 / 15)
    keep = rng.random(len(t)) > 0.04
    for s0 in (1100.0, 1300.0, 1500.0): keep &= ~((t >= s0) & (t < s0 + 0.7))
    m._buf.frames.extend(t[keep].tolist())
    live = cd.live_numbers("stream", m._buf.arrays(1000.0), 1000.0, 1600.0)
    assert 0.02 < live["drop"] < 0.06
    m.handle({"op": "stop"}, now=1600.0)
    s = m._state
    assert s.phase == "done" and s.writes == ["go2_air.video_stream"] and 0.02 < s.result["drop"] < 0.06
    assert s.current["go2_air.video_stream"]["source"] == "estimate"
    assert "estimate" in params.read_text()                                  # nothing written yet
    run = json.loads((tmp_path / "runs.jsonl").read_text().splitlines()[-1])
    assert run["id"] == "stream" and "go2_air.video_stream" in run["would_write"]
    m.handle({"op": "write", "run": s.run}, now=1601.0)
    assert "measured" in params.read_text() and m._state.writes == []


def test_the_taped_square_waits_for_the_measured_end_then_reports_drift(tmp_path, monkeypatch):
    m, _ = harness(tmp_path, monkeypatch)
    m.handle({"op": "start", "id": "taped-square"}, now=0.0)
    s = np.linspace(0, 50, 5001)                                               # five laps of a 2.5 m square, by its perimeter
    side = (s % 10) // 2.5; along = (s % 10) % 2.5
    x = np.select([side == 0, side == 1, side == 2, side == 3], [along, np.full_like(s, 2.5), 2.5 - along, np.zeros_like(s)])
    y = np.select([side == 0, side == 1, side == 2, side == 3], [np.zeros_like(s), along, np.full_like(s, 2.5), 2.5 - along])
    yaw = side * math.pi / 2
    x = x + 0.0012 * s                                                         # odometry drifting forward 6 cm over 50 m
    m._buf.odom.extend(zip((s * 2.5).tolist(), x.tolist(), y.tolist(), yaw.tolist()))
    live = cd.live_numbers("taped-square", m._buf.arrays(0.0), 0.0, 125.0)
    assert live["laps"] == 5 and abs(live["walked_m"] - 50) < 1
    m.handle({"op": "stop"}, now=125.0)
    assert m._state.result == {} and "enter" in m._state.message               # needs the tape first
    m.handle({"op": "truth", "forward_cm": 0, "left_cm": 0, "heading_deg": 0}, now=126.0)
    r = m._state.result
    assert abs(r["end_error_m"] - 0.06) < 0.01 and r["walked_m"] > 49 and m._state.writes == []


def test_power_and_link_status_from_the_battery(tmp_path, monkeypatch):
    m, _ = harness(tmp_path, monkeypatch)
    m.handle({"op": "start", "id": "power"}, now=0.0)
    bt = np.arange(0.0, 1500.0, 1.0); P = np.where(bt < 300, 60.0, 170.0)
    soc = 100 - np.r_[0, np.cumsum(P[1:])] / 3600 / 236.8 * 100
    m._buf.battery.extend(zip(bt.tolist(), np.floor(soc).tolist(), np.full(len(bt), 29.0).tolist(), P.tolist()))
    ot = np.arange(0.0, 1500.0, 0.1); s = np.clip(ot - 300, 0, None) * 0.4
    m._buf.odom.extend(zip(ot.tolist(), (2 * np.sin(s / 2)).tolist(), (2 - 2 * np.cos(s / 2)).tolist(), (s / 2).tolist()))
    m._buf.frames.extend(np.arange(1440.0, 1500.0, 1 / 15).tolist())
    m.handle({"op": "stop"}, now=1500.0)
    r = m._state.result
    assert abs(r["walk_power_W"] - 170) < 3 and abs(r["stand_power_W"] - 60) < 3
    assert set(m._state.writes) == {"go2_air.walk_power_W", "go2_air.stand_power_W"}
    st = m.link(1500.0)
    assert st.power_w == 170.0 and st.battery_v == 29.0 and 14 < st.camera_fps < 16 and st.drop < 0.01


def test_reset_returns_to_the_list_and_unknown_procedures_are_ignored(tmp_path, monkeypatch):
    m, _ = harness(tmp_path, monkeypatch)
    m.handle({"op": "start", "id": "nonsense"}, now=0.0)
    assert m._state.phase == "idle"
    m.handle({"op": "start", "id": "turn-lag"}, now=0.0); m.handle({"op": "stop"}, now=1.0)
    assert m._state.phase == "done" and "too little" in m._state.message
    m.handle({"op": "reset"})
    assert m._state.phase == "idle" and len(m._state.catalog) == len(cd.PROCEDURES)


def test_the_morning_feed_publishes_the_latest_log_once_and_again_when_a_newer_one_comes(tmp_path, monkeypatch):
    """[Ndt]: the feed puts the newest night's log on its channel, within the phone's bound, and again only when a
    newer night's log is written"""
    import os
    from types import SimpleNamespace
    from houndkeeper import morning_log as ml
    def log(stamp, prose):
        d = tmp_path / stamp; d.mkdir()
        (d / "morning_log.json").write_text(json.dumps({"night": {"kind": "sim", "date": None, "dir": str(d), "recording": None},
            "facts": {}, "finds": [], "prose": [prose], "route": [], "start": None, "stage_s": {}}))
        return d
    log("20261001_230000", "the first night")
    monkeypatch.setenv("HK_MORNING_RUNS", str(tmp_path))
    f = object.__new__(cd.MorningLogFeed); f._seen = None; sent = []; f.morning_log = SimpleNamespace(publish=sent.append)
    f.check(); f.check()
    assert [m["prose"] for m in sent] == [["the first night"]]
    log("20261002_230000", "the second night"); f.check()
    assert [m["prose"] for m in sent] == [["the first night"], ["the second night"]]
    assert all(len(json.dumps(m, separators=(",", ":"))) <= ml.PHONE_MAX_BYTES for m in sent)


def test_the_morning_page_is_registered_and_reads_only():
    """[Ndt]: the Morning page is one of the phone cockpit's pages, its one channel received and never sent"""
    pc = pytest.importorskip("houndkeeper.phone_cockpit")
    from dimos.web.cockpit import cockpit
    man = cockpit(**pc.phone_layout()).blueprints[0].kwargs["manifest"]
    panel = next(p for p in man["panels"] if p["kind"] == "hk-morning-log")
    ch = {c["ch"]: c for c in man["channels"]}
    assert panel["id"] in man["pages"] and panel["channels"] == ["morning_log"]
    assert ch["morning_log"]["dir"] == "rx" and ch["morning_log"]["publish"] == "none" and ch["morning_log"]["delivery"] == "latest"
