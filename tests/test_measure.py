"""[4hm]: the measurements read off a recording (real/calibration/measure.py) recover known inputs, so the same
analysis on the real robot's recording measures the real robot."""
import math
import sys
from pathlib import Path

from houndkeeper import workspace

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "real" / "calibration"))


def test_frame_gaps_recover_drops_and_stalls():
    import measure
    rng = np.random.default_rng(0); period = 1 / 15; minutes = 10
    t = np.arange(0, minutes * 60, period)
    keep = rng.random(len(t)) > 0.03                                    # 3% lost singly
    for s0 in rng.uniform(0, minutes * 60 - 1, 2 * minutes):            # twice a minute, 0.6 s of nothing
        keep &= ~((t >= s0) & (t < s0 + 0.6))
    r = measure.frame_gaps(t[keep] + rng.normal(0, 0.002, keep.sum()))
    assert abs(r["period_s"] - period) < 0.003
    assert 0.02 < r["drop"] < 0.04
    assert 1.6 < r["stall_per_min"] < 2.2 and 0.5 < r["stall_s"] < 0.8


def test_plane_spread_recovers_range_noise():
    import measure
    rng = np.random.default_rng(1)
    class Msg:
        def __init__(self, p): self.data = type("D", (), {"as_numpy": lambda s: (p, None)})(); self.ts = 0.0
    scans = []
    for _ in range(20):
        yz = rng.uniform([-1, 0.2], [1, 1.5], (400, 2))                 # a wall at x = 2, 2 cm range noise
        scans.append(Msg(np.c_[2 + rng.normal(0, 0.02, 400), yz]))
    r = measure.plane_spread(scans, (1.8, 2.2, -1, 1, 0.2, 1.5))
    assert abs(r["range_sd"] - 0.02) < 0.002 and r["returns_per_scan"] == 400


def test_odometry_scale_and_heading_drift_against_the_truth():
    import measure
    s = np.linspace(0, 40, 4001); t = s / 0.4                           # 40 m round a 4 m circle at 0.4 m/s
    th = s / 2.0; T = {"wall_t": t, "true_x": 2 * np.sin(th), "true_y": 2 - 2 * np.cos(th), "true_yaw": th}
    drift = np.radians(0.5) * np.sqrt(s)                                # heading drifting 0.5 deg per root metre
    yaw = th + drift; ds = np.r_[0, np.diff(s)] * 1.02                  # odometry 2% long
    x = np.cumsum(ds * np.cos(yaw)); y = np.cumsum(ds * np.sin(yaw))
    r = measure.odom_vs_truth(t, x, y, yaw, T)
    assert abs(r["scale"] - 0.02) < 0.003
    assert abs(r["end_yaw_deg"] - 0.5 * math.sqrt(40)) < 0.2
    assert abs(r["yaw_deg_per_root_m"] - 0.5) < 0.05


def test_writing_a_measurement_keeps_the_file_one_line_per_parameter(tmp_path):
    import measure, yaml
    p = tmp_path / "robot.yaml"
    p.write_text("go2_air:\n  video_stream:      {value: {drop: 0.03, stall_per_min: 2, stall_s: 0.6}, source: estimate, note: \"est.\"}\n"
                 "  crouch_height_m:   {value: null,  source: unknown}\nkit:\n  budget_kg: {value: 1.8, source: estimate}\n")
    measure.set_param("go2_air.video_stream", {"drop": 0.011, "stall_per_min": 0.4, "stall_s": 0.3}, "test", params=p)
    measure.set_param("go2_air.crouch_height_m", 0.21, "tape", params=p)
    d = yaml.safe_load(p.read_text())
    assert d["go2_air"]["video_stream"]["value"]["drop"] == 0.011 and d["go2_air"]["video_stream"]["source"] == "measured"
    assert d["go2_air"]["crouch_height_m"]["value"] == 0.21 and "tape" in d["go2_air"]["crouch_height_m"]["note"]
    assert d["kit"]["budget_kg"]["source"] == "estimate" and len(p.read_text().splitlines()) == 5


def test_power_split_recovers_walking_and_standing_power():
    import measure
    rng = np.random.default_rng(2); Wh = 236.8
    bt = np.arange(0, 25 * 60, 1.0)                                     # the battery at 1 Hz: 5 min standing, 20 walking
    P = np.where(bt < 300, 60.0, 170.0) + rng.normal(0, 8, len(bt))
    used = np.r_[0, np.cumsum((P[1:] + P[:-1]) / 2 * np.diff(bt))] / 3600
    soc = np.floor(100 - used / Wh * 100)                                # the pack reports whole percent
    ot = np.arange(0, 25 * 60, 0.1); s = np.clip(ot - 300, 0, None) * 0.4  # odometry at 10 Hz: still, then 0.4 m/s round a 2 m circle
    x, y, yaw = 2 * np.sin(s / 2), 2 - 2 * np.cos(s / 2), s / 2
    r = measure.power_split(bt, soc, P, (ot, x, y, yaw), Wh)
    assert abs(r["walk_power_W"] - 170) < 2 and abs(r["stand_power_W"] - 60) < 2
    assert 1190 < r["walk_s"] <= 1200 and 290 < r["stand_s"] <= 300
    assert abs(r["charge_vs_mean"]) < 0.06                              # whole percent over 25 min: within a few percent


def test_writing_stand_power_adds_its_line_beside_walk_power(tmp_path):
    import measure, yaml
    p = tmp_path / "robot.yaml"
    p.write_text("go2_air:\n  walk_power_W:      {value: 170,   source: estimate, note: \"est.\"}\n  mass_kg:           {value: 15,    source: vendor}\n")
    measure.set_param("go2_air.walk_power_W", 182.4, "power", params=p)
    measure.set_param("go2_air.stand_power_W", 61.0, "power", params=p, add_after="walk_power_W")
    measure.set_param("go2_air.stand_power_W", 63.5, "again", params=p, add_after="walk_power_W")
    d = yaml.safe_load(p.read_text()); lines = p.read_text().splitlines()
    assert d["go2_air"]["walk_power_W"]["value"] == 182.4 and d["go2_air"]["stand_power_W"]["value"] == 63.5
    assert d["go2_air"]["stand_power_W"]["source"] == "measured" and len(lines) == 4 and lines[2].startswith("  stand_power_W:")


# ---- [KbT] the gap report: sim against a perturbed sim recovers the perturbations -----------------------------------
class _Obs:
    def __init__(self, ts, data): self.ts, self.data = ts, data


class _Pts:
    def __init__(self, p): self.p = p
    def as_numpy(self): return (self.p, None)


class _Odom:
    def __init__(self, x, y, yaw):
        self.position = type("P", (), dict(x=x, y=y, z=0.0))()
        self.orientation = type("Q", (), dict(w=math.cos(yaw / 2), x=0.0, y=0.0, z=math.sin(yaw / 2)))()


class _FakeStore:
    """a recording made up from known quantities: ten minutes' frames (images in the still first seconds) with given noise, frames lost singly and in
    stalls, lidar scans of a wall with given range spread and points, odometry with a given scale error"""
    def __init__(self, seed, read=0.01, shot=0.001, drop=0.0, stalls_per_min=0.0, pts=6000, range_sd=0.02, scale=0.0):
        rng = np.random.default_rng(seed); self.s = {}
        period = 1 / 15; t = np.arange(0, 600, period); keep = rng.random(len(t)) >= drop    # ten minutes of frames
        for s0 in rng.uniform(10, 590, int(stalls_per_min * 10)): keep &= ~((t >= s0) & (t < s0 + 0.6))
        level = np.tile(np.linspace(0.06, 0.88, 320), (240, 1))                     # a scene spanning the brightness levels
        frames = []
        for k, ts in enumerate(t[keep]):
            img = None
            if ts < 12.0:                                                           # images only where they are analysed (standing still)
                L = level + rng.normal(0, 1, level.shape) * np.sqrt(shot * level + read ** 2)
                img = np.repeat(np.clip(np.round(L * 255), 0, 255).astype(np.uint8)[..., None], 3, -1)
            frames.append(_Obs(ts, img))
        self.s["color_image"] = frames
        ot = np.arange(0, 60, 0.05); s = np.clip(ot - 10, 0, None) * 0.4            # still 10 s, then 0.4 m/s round a 4 m circle
        th = s / 2.0; self.truth = {"wall_t": ot, "true_x": 2 * np.sin(th), "true_y": 2 - 2 * np.cos(th), "true_yaw": th}
        ds = np.r_[0, np.diff(s)] * (1 + scale); x = np.cumsum(ds * np.cos(th)); y = np.cumsum(ds * np.sin(th))
        self.s["odom"] = [_Obs(a, _Odom(b, c, d)) for a, b, c, d in zip(ot, x, y, th)]
        scans = []
        for ts in np.arange(0, 60, 0.5):                                            # 2 scans a second, a wall at x = 3 among other points
            n_wall = pts // 4; yz = rng.uniform([-1, 0.3], [1, 1.5], (n_wall, 2))
            wall = np.c_[3 + rng.normal(0, range_sd, n_wall), yz]
            rest = rng.uniform([-3, -3, 0], [2.5, 3, 1.5], (pts - n_wall, 3))
            scans.append(_Obs(ts, _Pts(np.r_[wall, rest])))
        self.s["lidar"] = scans; self.s["battery"] = []
    def stream(self, name): return self.s[name]


_BOX = (2.85, 3.15, -0.8, 0.8, 0.4, 1.4)
_PARAMS = {"video_stream": {"value": {"drop": 0.03, "stall_per_min": 2, "stall_s": 0.6}}, "lidar": {"value": {"points_per_s": 21600, "range_sd": 0.02}},
           "camera_imperfections": {"value": {"read_noise": 0.004, "shot_noise": 0.0003}}, "frame_delay_s": {"value": {"mean": 0.15}},
           "odom_delay_s": {"value": {"mean": 0.03}}, "odom_drift": {"value": {"scale_sd": 0.02, "xy_sd": 0.005, "yaw_sd_deg": 0.5}}}


def _row(rows, key): return next(r for r in rows if r["quantity"] == key)


def test_gap_report_recovers_known_gaps_between_sim_and_a_perturbed_sim():
    import measure
    sim = measure.analyse(_FakeStore(1), box=_BOX)
    fake = _FakeStore(2, read=0.015, shot=0.002, drop=0.04, stalls_per_min=3, pts=4500, range_sd=0.03, scale=0.03)
    real = measure.analyse(fake, box=_BOX, truth=fake.truth)
    rows = measure.gap_rows(real["values"], sim["values"], _PARAMS)
    d = _row(rows, "stream.drop"); assert abs(d["real"] - 0.04) < 0.008 and d["action"].startswith("write go2_air.video_stream.drop")
    assert 2.6 <= _row(rows, "stream.stall_per_min")["real"] <= 3.1 and abs(_row(rows, "stream.stall_s")["real"] - 0.6) < 0.1
    n = _row(rows, "noise.read_noise"); assert n["kind"] == "matched" and 1.3 < n["real"] / n["sim"] < 1.7 and n["action"].startswith("raise go2_air.camera_imperfections.read_noise")
    s = _row(rows, "noise.shot_noise"); assert 1.5 < s["real"] / s["sim"] < 2.6 and s["action"].startswith("raise")
    p = _row(rows, "lidar.points_per_message"); assert p["real"] == 4500 and p["sim"] == 6000 and "lower go2_air.lidar.points_per_s x0.75" in p["action"]
    w = _row(rows, "wall.range_sd"); assert abs(w["real"] - 0.03) < 0.003 and w["action"].startswith("write go2_air.lidar.range_sd = 0.03")
    o = _row(rows, "odom.scale"); assert abs(o["real"] - 0.03) < 0.003
    assert "power" in real["skipped"] and "odom" in sim["skipped"]
    md = measure.gap_markdown(rows, "fake-real", "fake-sim", real["skipped"])
    assert "| camera effective read noise" in md and md.count("\n|") >= len(rows) + 1


def test_gap_report_says_agree_when_nothing_differs():
    import measure
    a = measure.analyse(_FakeStore(3), box=_BOX)["values"]; b = measure.analyse(_FakeStore(4), box=_BOX)["values"]
    rows = measure.gap_rows(b, a, _PARAMS)
    for k in ("noise.read_noise", "noise.shot_noise", "lidar.points_per_message"): assert _row(rows, k)["action"].startswith("agree"), k


def test_gap_reads_the_sim_reference_file(tmp_path, monkeypatch):
    import measure, json
    ref = json.loads(measure.SIM_REFERENCE.read_text())
    assert {"made", "params", "recordings", "summary"} <= set(ref) and len(ref["recordings"]) >= 4
    assert not {r["house"] for r in ref["recordings"]} & measure.HOLD_OUT
    for k in ("noise.read_noise", "lidar.points_per_message", "wall.range_sd", "lag.frame_lag_vs_odom_s", "stream.drop"): assert k in ref["summary"], k
    sv = {k: v["median"] for k, v in ref["summary"].items()}
    real = dict(sv, **{"noise.read_noise": sv["noise.read_noise"] * 2})        # a "real" twice as noisy as the sim's nights
    rows = measure.gap_rows(real, sv, measure.go2_params())
    assert "x2.00" in _row(rows, "noise.read_noise")["gap"] and _row(rows, "lidar.points_per_message")["action"].startswith("agree")
    assert _row(rows, "stream.drop")["flag"]                                   # the sim's stream is stamped on the wall clock


_RECORDINGS = workspace.dimos() / "recordings"          # the DimOS checkout beside this one; tests skip without it
_S95 = _RECORDINGS / "20261002-214714-unitree-go2-agentic" / "memory.db"
_S70 = _RECORDINGS / "20261001-232147-unitree-go2-agentic" / "memory.db"    # house 70 night 1, a tuning house: another night


class _Noisier:
    """a frame given extra noise when it is read (only the frames an analysis reads are decoded)"""
    def __init__(self, img, sd, rng): self.img, self.sd, self.rng = img, sd, rng
    def to_numpy(self):
        from floor_mosaic2 import arr
        a = arr(self.img).astype(np.float32)
        return np.clip(a + self.rng.normal(0, self.sd * 255, a.shape[:2])[..., None], 0, 255)


class _Perturbed:
    """a sim recording seen through known changes: frames stamped late by lag_s and every drop_every-th lost, still frames
    given extra noise, the lidar's scans thinned to keep_pts of their points"""
    def __init__(self, st, lag_s=0.08, drop_every=20, extra_sd=0.02, keep_pts=0.5, seed=0):
        self.st, self.lag, self.every, self.sd, self.keep = st, lag_s, drop_every, extra_sd, keep_pts; self.rng = np.random.default_rng(seed)
    def stream(self, name):
        if name == "color_image":
            out = []
            for i, o in enumerate(self.st.stream(name)):
                if i % self.every == self.every - 1: continue
                out.append(_Obs(o.ts + self.lag, _Noisier(o.data, self.sd, self.rng)))
            return out
        if name == "lidar":
            return [_Obs(o.ts, _Pts(np.asarray(o.data.as_numpy()[0])[: int(len(o.data.as_numpy()[0]) * self.keep)])) for o in self.st.stream(name)]
        return self.st.stream(name)


def test_gap_on_a_perturbed_sim_night_recovers_lag_drops_noise_and_lidar_thinning():
    import measure, pytest
    if not _S95.exists(): pytest.skip("house 95 night 1 recording not on this machine")
    sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))
    st = measure.store(_S95); box = measure.sim_wall_box(95, -3.3)
    base = measure.analyse(st, box=box)["values"]; pert = measure.analyse(_Perturbed(st), box=box)["values"]
    rows = measure.gap_rows(pert, base, measure.go2_params())
    assert abs((pert["lag.frame_lag_vs_odom_s"] - base["lag.frame_lag_vs_odom_s"]) - 0.08) < 0.035
    assert pert["stream.drop"] > base["stream.drop"]          # one frame in 20 lost on top (the sim's own wall-clock stamps are
                                                              # irregular, so the clean recovery is the synthetic test's)
    want = math.hypot(base["noise.read_noise"], 0.02)                                 # independent noise adds in variance
    assert abs(pert["noise.read_noise"] - want) < 0.25 * want and _row(rows, "noise.read_noise")["action"].startswith("raise")
    assert abs(pert["lidar.points_per_message"] / base["lidar.points_per_message"] - 0.5) < 0.02
    assert "lower go2_air.lidar.points_per_s x0.5" in _row(rows, "lidar.points_per_message")["action"]
    assert pert["wall.range_sd"] == base["wall.range_sd"] or abs(pert["wall.range_sd"] - base["wall.range_sd"]) < 0.004   # thinning keeps the spread


def test_gap_command_smoke_on_a_second_sim_night(tmp_path):
    import measure, pytest, subprocess
    if not _S95.exists(): pytest.skip("house 95 night 1 recording not on this machine")
    out = tmp_path / "gap.md"
    r = subprocess.run([sys.executable, str(HK / "real/calibration/measure.py"), "gap", str(_S95), "--box", *map(str, measure.sim_wall_box(95, -3.3)),
                        "--out", str(out)], capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stderr[-2000:]
    md = out.read_text(); assert "| frames dropped singly" in md and "| camera effective read noise" in md and "sim_reference.json" in md


def test_gap_report_on_two_different_sim_nights_reports_each_nights_own_values(tmp_path):
    """[KbT] the gap report run on two genuinely different sim nights (house 70 night 1 as "real", house 95 night 1 as
    the sim, both measured now): each side's numbers are that night's own (as the sim reference recorded them for that
    night), and the gap and action follow from them. Only tuning and demo houses: no hold-out night is read."""
    import measure, pytest, subprocess, json
    if not (_S95.exists() and _S70.exists()): pytest.skip("house 95 and house 70 night 1 recordings not on this machine")
    ref = {(r["house"], r["night"]): r for r in json.loads(measure.SIM_REFERENCE.read_text())["recordings"]}
    h70, h95 = ref[(70, 1)], ref[(95, 1)]
    assert Path(h70["db"]).parent.name == _S70.parent.name and Path(h95["db"]).parent.name == _S95.parent.name
    assert not {70, 95} & measure.HOLD_OUT
    out, js = tmp_path / "gap.md", tmp_path / "gap.json"
    r = subprocess.run([sys.executable, str(HK / "real/calibration/measure.py"), "gap", str(_S70), "--box", *map(str, h70["wall_box"]),
                        "--sim", str(_S95), "--sim-box", *map(str, h95["wall_box"]), "--out", str(out), "--json", str(js)],
                       capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stderr[-2000:]
    rows = {x["quantity"]: x for x in json.loads(js.read_text())["rows"]}
    for k in ("lidar.points_per_message", "wall.returns_per_scan", "wall.range_sd", "noise.read_noise", "stream.period_s"):
        assert rows[k]["real"] == pytest.approx(h70["values"][k], rel=0.02), k
        assert rows[k]["sim"] == pytest.approx(h95["values"][k], rel=0.02), k
    assert rows["lidar.points_per_message"]["real"] != rows["lidar.points_per_message"]["sim"]       # two nights, not one
    k = rows["lidar.points_per_message"]["real"] / rows["lidar.points_per_message"]["sim"]
    assert rows["lidar.points_per_message"]["gap"] == f"x{k:.2f}"
    assert rows["lidar.points_per_message"]["action"].startswith(("lower" if k < 1 else "raise", "agree"))
    md = out.read_text()
    assert f"Real: `{_S70}`" in md and f"`{_S95}` (measured now)" in md


def test_day_one_lag_searches_past_the_nightly_reach_and_never_reports_a_failed_turn_as_zero(monkeypatch):
    """[GWP], [gJq]: day one's lag is searched out to a second, held to the robot's gate; a turn too short or too
    loose to measure is said and gives no number (never 0 s), and a lag past the nightly check's reach is said"""
    import measure, floor_mosaic2
    est = {}
    monkeypatch.setattr(floor_mosaic2, "lag_detail_from_turns", lambda st, max_lag=0.3: dict(est, max_lag=max_lag))
    est.update(lag_s=None, corr=0.0, pairs=12, turning=0, edge=False)
    d = measure.day_one_lag(None); assert d["refused"] and "too few" in d["refused"] and d["max_lag"] == measure.DAY_ONE_MAX_LAG
    est.update(lag_s=0.42, corr=0.9, pairs=900, turning=400)
    d = measure.day_one_lag(None); assert d["refused"] is None and d["past_nightly"] and "0.3" in d["past_nightly"]
    est.update(lag_s=0.12)
    d = measure.day_one_lag(None); assert d["refused"] is None and d["past_nightly"] is None
    est.update(corr=0.5)
    assert "correlation" in measure.day_one_lag(None)["refused"]


# ---- [GWP] day one's pass bands: the mount night to night, the dark floor's compression ----------------------------
def test_band_verdict_reads_pass_outside_and_fails_from_the_study_steps():
    """[GWP] inside the band passes, at or past the step the study saw raise false alarms fails, untested between is
    outside (never a pass); a band with no failing step tested says so"""
    import measure
    assert measure.band_verdict("mount", 0.08)[0] == "pass" and measure.band_verdict("mount", -0.3)[0] == "outside"
    assert measure.band_verdict("mount", 0.5)[0] == "fails" and "19.0" in measure.band_verdict("mount", 0.7)[1]
    assert measure.band_verdict("lag", 0.015)[0] == "pass" and measure.band_verdict("lag", -0.06)[0] == "fails"
    assert measure.band_verdict("drift", 1.0)[0] == "pass" and measure.band_verdict("drift", 1.5)[0] == "outside"
    assert measure.band_verdict("noise", 3.0)[0] == "pass" and measure.band_verdict("noise", 6.0)[0] == "outside"


def _check(pitch=0.0, yaw=0.0, up=0.0, judged=True):
    return {"change": {"pitch_deg": pitch, "yaw_deg": yaw, "up_m": up, "lag_s": None}, "judged": judged}


def test_mount_series_flags_a_head_that_moved_between_nights():
    """[GWP] positive control: night 2's floor disagrees with the calibration by 0.5 deg more pitch than night 1's (a
    knock between them): flagged as moved; the same two nights a few hundredths apart pass; one judged night is not
    enough to say the mount stayed put"""
    import measure
    r = measure.mount_series([("n1", _check(0.02, 0.05, 0.001)), ("n2", _check(0.52, 0.05, 0.001))])
    assert r["verdict"] == "outside" and len(r["moved"]) == 1 and "pitch moved +0.5" in r["moved"][0] and r["off"]
    r = measure.mount_series([("n1", _check(0.02, 0.05, 0.001)), ("n2", _check(0.05, 0.1, 0.002))])
    assert r["verdict"] == "pass" and not r["moved"] and not r["off"]
    r = measure.mount_series([("n1", _check(0.02)), ("n2", _check(0.9, judged=False))])
    assert r["verdict"] == "not judged" and "1 of 2" in r["note"]
    r = measure.mount_series([("n1", _check(0.0, yaw=0.0)), ("n2", _check(0.0, yaw=0.35))])
    assert r["verdict"] == "outside" and "yaw" in r["moved"][0]                     # yaw's band is the check's 0.3 deg


def test_mount_series_on_the_sim_houses_own_checks_passes_and_catches_a_planted_knock():
    """[GWP] negative control on real pipeline output: the nightly checks of tuning houses rendered with one robot
    mount (nothing moved between them) pass; the same with 0.5 deg planted on one fails it (positive control)"""
    import measure, pytest, json
    runs = [HK / "sim/house/runs" / f"nights_s{s}" / "calibration_check.json" for s in (70, 71, 72, 73, 90, 96)]
    docs = [(p.parent.name, json.loads(p.read_text())) for p in runs if p.exists()]
    docs = [(n, d) for n, d in docs if d.get("judged") and not d.get("warnings")]
    if len(docs) < 3: pytest.skip("the sim houses' calibration checks are not on this machine")
    assert measure.mount_series(docs)["verdict"] == "pass"
    n, d = docs[-1]; planted = json.loads(json.dumps(d)); planted["change"]["pitch_deg"] += 0.5
    r = measure.mount_series(docs[:-1] + [(n, planted)])
    assert r["verdict"] == "outside" and any(n in m and "pitch" in m for m in r["moved"])


def _dark_floor(seed, sd=0.008, H=480, W=640):
    """a frame whose lower half is a dark, finely textured floor (about 30 grey levels) with sensor grain, its upper half lit"""
    import cv2
    r = np.random.default_rng(seed)
    g = cv2.GaussianBlur(r.normal(0, 1, (H, W)).astype(np.float32), (0, 0), 2) * 0.04 + 0.12
    g[: H // 2] += 0.4; g += r.normal(0, sd, (H, W))
    return np.repeat(np.clip(g * 255, 0, 255).astype(np.uint8)[..., None], 3, -1)


def test_compression_measure_tells_jpeg_qualities_apart_on_a_dark_floor():
    """[GWP] [4hm] the dark floor's zeroed share rises steadily as JPEG quality falls (q95, 75, 50, 30), and the
    uncompressed floor has the least; only the dark tiles are read (their level is the floor's, not the lit half's)"""
    import measure
    raw = [_dark_floor(i) for i in range(4)]
    m = [measure.compression_measures(raw)] + [measure.compression_measures(measure.reencode(raw, q)) for q in (95, 75, 50, 30)]
    flat = [x["flat"] for x in m]; ac = [x["ac"] for x in m]
    assert all(b > a + 0.01 for a, b in zip(flat, flat[1:])), flat
    assert all(b < a for a, b in zip(ac, ac[1:])), ac
    assert all(x["dark_level"] < 0.2 for x in m)


def test_compression_compare_sets_the_real_floor_beside_the_sims_at_q50_and_q30(tmp_path):
    """[GWP] the sim's frames are the floor at q75 (the sim's own setting): the compare measures the real frames, the
    sim's and the sim's re-encoded at q50 and q30 on the same dark tiles, a real stream at q30 reads flatter than the
    sim's at q50, and the side-by-side picture has a row for each (64-pixel tiles, enlarged 3x)"""
    import measure, cv2
    sim = measure.reencode([_dark_floor(100 + i) for i in range(4)], 75)
    r = measure.compression_compare(measure.reencode([_dark_floor(i) for i in range(4)], 30), sim, tmp_path / "sheet.png")
    assert set(r) >= {"real", "sim", "sim_q50", "sim_q30", "sheet"} and r["real"]["flat"] > r["sim_q50"]["flat"]
    assert r["sheet"]["rows"] == ["real", "sim", "sim q50", "sim q30"]
    im = cv2.imread(str(tmp_path / "sheet.png"), cv2.IMREAD_GRAYSCALE); assert im.shape == (4 * 64 * 3, 6 * 64 * 3)


def test_only_the_dark_floor_is_read():
    """[GWP] tiles are read in the frame's lower half and within the dark band only: a lit floor gives none"""
    import measure
    g = np.full((480, 640), 0.12, np.float32); assert len(measure.dark_tiles(g)) == 3 * 10
    assert measure.dark_tiles(g + 0.4) == [] and measure.compression_measures([np.full((480, 640, 3), 200, np.uint8)]) is None
