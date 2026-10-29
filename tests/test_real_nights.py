"""[V35]: a floor map from real nights alone: the steps that read the sim's truth make it optional, and a recording is
checked for what the map-building steps read before they run. [Lzt]: tonight's decisions become the robot's list and a
night the morning log and the phone read with no sim file; [vO9], [71R], [iDT]: a real night scored by the user's
marks, its flags judged, nights pooled into the sim's figures."""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def _picture(path, rng, shift=0):
    h, w = 160, 240
    tiles = np.repeat(np.repeat(rng.uniform(60, 200, (h // 20 + 1, w // 20 + 1, 3)), 20, 0), 20, 1)[:h, :w]
    rgb = np.clip(np.roll(tiles, shift, 1) + rng.normal(0, 2, (h, w, 3)), 0, 255).astype(np.uint8)
    np.savez(path, rgb=rgb, dist=np.ones((h, w), np.float16), occ=np.zeros((h, w), bool))


def test_a_run_without_truth_files_or_a_seed_has_no_truth(tmp_path):
    import floor_agreement as fa
    assert not fa.has_truth(tmp_path, None)
    assert not fa.has_truth(tmp_path, 95)
    for k in (1, 2): (tmp_path / f"nights_s95_n{k}.truth.json").write_text("{}")
    assert fa.has_truth(tmp_path, 95) and not fa.has_truth(tmp_path, None)


def test_the_seed_comes_from_a_sim_run_dir_name_only():
    import floor_agreement as fa
    assert fa.seed_of(Path("runs/nights_s95")) == 95
    assert fa.seed_of(Path("tmp/realreplay/run")) is None
    assert fa.seed_of(Path("kitchen_shelf")) is None


def test_two_real_nights_disagreement_needs_no_truth(tmp_path):
    """the same floor twice agrees, the floor moved by a tile does not; neither reads a truth file (none exist), and the
    rug and chairs, known only from the truth, are not reported"""
    import floor_agreement as fa
    _picture(tmp_path / "a.npz", np.random.default_rng(2)); _picture(tmp_path / "b.npz", np.random.default_rng(2))
    _picture(tmp_path / "c.npz", np.random.default_rng(2), shift=10)
    same, _ = fa.agreement(tmp_path, None, tmp_path / "a.npz", tmp_path / "b.npz")
    moved, _ = fa.agreement(tmp_path, None, tmp_path / "a.npz", tmp_path / "c.npz")
    assert set(same) == {"all", "open"} and same["all"] == same["open"]
    assert same["all"][0] < 0.01 < 0.10 < moved["all"][0]


def test_the_floor_map_error_on_a_night_without_truth_is_skipped_and_says_so():
    import floor_fidelity as ff
    with pytest.raises(SystemExit, match="sim only"):
        ff.truth(None, 1, (10, 10))


def test_a_recording_is_checked_for_what_the_steps_read():
    import real_nights as rn
    full = set(rn.STREAMS); tf = set(rn.TF_PAIRS)
    assert rn.problems(full, tf, 1.0) == []
    no_lidar = rn.problems(full - {"lidar"}, tf, 1.0)
    assert [s for s, _ in no_lidar] == ["missing"] and "lidar" in no_lidar[0][1]
    no_cam = rn.problems(full, tf - {("base_link", "camera_link")}, 1.0)
    assert [s for s, _ in no_cam] == ["missing"]
    off = rn.problems(full, tf, 0.5)
    assert [s for s, _ in off] == ["warn"]


def test_the_share_of_a_path_on_the_floor_grid():
    import real_nights as rn
    xy = np.array([[0, 0], [rn.HX - 0.1, 0], [0, -(rn.HY - 0.1)], [rn.HX + 1, 0]])
    assert rn.canvas_share(xy) == pytest.approx(0.75)
    assert rn.canvas_share(xy, margin=0.5) == pytest.approx(0.25)
    assert rn.canvas_share(np.zeros((0, 2))) == 0.0


# ---- a whole tidy night from its recordings alone ([Lzt]) and its score without the truth ([vO9], [71R], [iDT]) ----

def _decision(x, y, decision="confirmed", ts=100.0, **kw):
    return dict(x=x, y=y, x_robot=x - 0.1, y_robot=y + 0.1, ts=ts, decision=decision, **kw)


def test_tonights_decisions_become_the_robots_list():
    """[Lzt], [71R]: kept decisions only (confirmed or unsure, never floor-sized), one place per thing (folded within
    30 cm into the strongest), and the light check's hook drops only what it rejected"""
    import real_tidy as rt
    D = [_decision(1.0, 1.0, "unsure", ts=90), _decision(1.1, 1.05, ts=95), _decision(3.0, 0.0, "not_a_find"),
         _decision(2.0, 2.0, kind="floor_change"), _decision(-1.0, 0.5, "unsure", ts=120)]
    c, how = rt.candidates(D)
    assert how == {"checked": False} and [(x["x"], x["decision"], x["members"]) for x in c] == [(1.1, "confirmed", 2), (-1.0, "unsure", 1)]
    assert all(x["check"] is None for x in c)
    k, how = rt.candidates(D, check={"c1": {"keep": True, "score": 0.8}, "c2": {"keep": False, "score": 0.01}})
    assert [x["id"] for x in k] == ["c1"] and how == {"checked": True, "dropped": 1}
    D[1]["check"] = {"score": 0.02, "kept": False}; D[4]["check"] = {"score": 0.6, "kept": True}   # confirm_stream --check's
    k, how = rt.candidates(D)                       # the rejected one goes before the fold: the unseen one beside it stays
    assert [x["x"] for x in k] == [1.0, -1.0] and how == {"checked": True, "dropped": 1}
    # the stand-in night's card game: an early look the check rejected, a later one it kept, 25 cm apart; the kept one
    # stays and leads its place, as the bench folds the launch arm
    E = [_decision(2.0, 0.0, ts=90, check={"score": 0.2, "kept": False}), _decision(2.0, 0.25, ts=95, check={"score": 0.74, "kept": True}),
         _decision(2.1, 0.1, ts=99, check={"score": 0.3, "kept": True})]
    k, how = rt.candidates(E)
    assert [(x["y"], x["members"], x["check"]["score"]) for x in k] == [(0.25, 2, 0.74)] and how == {"checked": True, "dropped": 1}


def test_a_replayed_night_gives_a_morning_log_and_the_phone_list_without_sim_files(tmp_path, monkeypatch):
    """[Lzt]: the night directory made from a recording alone (no traj.csv, no truth, no catalog entry) gives the morning
    log: its walk from the recording's own odometry, each place filed for the user to see to, said as places marked on a
    walk without stops; the phone's message lists them; pictures withheld for an uncatalogued recording ([R1X])"""
    import real_tidy as rt
    from houndkeeper import morning_log as ml
    db = tmp_path / "rec" / "memory.db"; db.parent.mkdir(); db.write_bytes(b"")
    s = np.linspace(0, 1, 200)
    monkeypatch.setattr(ml, "recording_odometry", lambda p: np.stack([1.79e9 + 600 * s, 5 * s, 0 * s, 0 * s], 1))
    monkeypatch.setattr(ml, "kind_of", lambda rid: "unknown")
    c, _ = rt.candidates([_decision(1.0, 1.0, ts=1.79e9 + 60), _decision(3.0, -1.0, "unsure", ts=1.79e9 + 300)])
    root = tmp_path / "nights"; d = rt.night_dir(root / "20261013_220000", c, db, 1.79e9, 1.79e9 + 600)
    assert sorted(p.name for p in d.iterdir()) == ["night.json", "recording.txt", "report.json"]
    opened = []; real_path = ml.Night.path
    monkeypatch.setattr(ml.Night, "path", lambda self, name: opened.append(name) or real_path(self, name))
    ml.write(d)
    doc = json.loads((d / "morning_log.json").read_text())
    assert doc["facts"]["walked_m"]["value"] == 5 and doc["facts"]["walked_m"]["source"] == "recording.txt"
    assert doc["facts"]["stops"]["value"] == 2 and [f["fate"] for f in doc["finds"]] == ["other", "other"]
    assert any("walked my rounds without stopping" in p for p in doc["prose"])
    assert not any("went to see" in p or "see to" in p for p in doc["prose"]) and doc["pictures_note"].startswith("Pictures withheld")
    assert doc["facts"]["ended"]["source"] == doc["facts"]["minutes"]["source"] == "night.json" and doc["facts"]["minutes"]["value"] == 10
    assert "traj.csv" in opened and not set(opened) & {"truth.json", "score.json", "score.txt"}
    m = ml.morning_message(root)
    assert [f["k"] for f in m["finds"]] == [1, 2] and all(f["jpg"] is None for f in m["finds"])


def _marks(toys, night="n1"):
    return {"night": night, "toys": toys}


def _toy(i, x, y, floor="hard", brick=False, half=0.05):
    return {"id": f"t{i}", "name": f"toy {i}", "x": x, "y": y, "half_long_m": half, "brick": brick, "floor": floor}


def _cands(*xy):
    return [{"id": f"c{k + 1}", "x": x, "y": y, "decision": "confirmed"} for k, (x, y) in enumerate(xy)]


def _score_night():
    sys.path.insert(0, str(HK / "real" / "tests")); import score_night as sn
    return sn


def test_a_real_night_is_scored_against_the_users_marks():
    """[vO9]: a marked toy is found by a candidate within the sim's reach (30 cm, or 15 cm past half a long toy), by
    floor kind with the bricks apart; candidates near no toy are flags"""
    sn = _score_night()
    marks = _marks([_toy(1, 0, 0), _toy(2, 2, 0, "carpet"), _toy(3, 4, 0, "rug", brick=True), _toy(4, 6, 0, "rug", half=0.40)])
    r = sn.score_night(marks, _cands((0.25, 0), (2.4, 0), (4.1, 0), (6.5, 0), (9, 9)))
    assert [t["found"] for t in r["toys"]] == [True, False, True, True]
    assert r["floor"] == {"hard": [1, 1], "carpet": [0, 1], "rug": [1, 1]} and r["bricks"] == [1, 1] and r["main"] == [2, 3]
    assert [f["ids"] for f in r["flags"]] == [["c2"], ["c5"]] and r["unjudged"] == 2 and not r["final"]


def test_unmatched_flags_are_judged_before_a_night_counts():
    """[71R]: a flag the user judges a real change counts neither way, a false alarm counts against the robot, and
    nights are pooled only when every flag is judged"""
    sn = _score_night()
    marks = _marks([_toy(1, 0, 0)])
    open_ = sn.score_night(marks, _cands((0, 0), (3, 3), (5, 5)))
    with pytest.raises(SystemExit, match="not yet judged"):
        sn.pool([open_])
    r = sn.score_night(marks, _cands((0, 0), (3, 3), (5, 5)), {"c2": "false_alarm", "c3": "real_change"})
    assert (r["false_alarms"], r["real_changes"], r["unjudged"], r["final"]) == (1, 1, 0, True)
    with pytest.raises(SystemExit, match="judgements"):
        sn.score_night(marks, _cands((3, 3)), {"c1": "maybe"})


def test_real_nights_pool_into_the_sims_figures():
    """[iDT]: pooled nights in the sim's own figures (found, its share and 95% interval, false alarms a night over every
    night, laid out or not, per floor kind, the bricks apart)"""
    sn = _score_night()
    a = sn.score_night(_marks([_toy(1, 0, 0), _toy(2, 2, 0, "carpet", brick=True)], "a"), _cands((0, 0), (8, 8)), {"c2": "false_alarm"})
    b = sn.score_night(_marks([], "b"), _cands((1, 1), (5, 5)), {"c1": "false_alarm", "c2": "false_alarm"})
    p = sn.pool([a, b])
    assert (p["found"], p["n"], p["main"], p["tiny"], p["fp_per_night"]) == (1, 2, [1, 1], [0, 1], 1.5)
    assert p["laid_out_nights"] == 1 and p["floor"]["hard"] == [1, 1] and p["found_share_95"] == sn.wilson(1, 2)
    assert sn.wilson(38, 40) == pytest.approx((0.835, 0.986), abs=0.001)


def test_the_rehearsals_stand_in_marks_are_placed_by_a_rigid_fit():
    """[vO9] (the rehearsal's stand-in for the user's marks): the turn and shift between two frames recovered from
    matched points, never a mirror"""
    sys.path.insert(0, str(HK / "real" / "tests")); import rehearsal_score as rs
    rng = np.random.default_rng(3); P = rng.uniform(-4, 4, (30, 2)); th = 0.3
    R0 = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); Q = P @ R0.T + [1.0, -2.0]
    R, t, rms = rs.rigid_fit(P, Q)
    assert np.allclose(R, R0) and np.allclose(t, [1.0, -2.0]) and rms < 1e-9 and np.linalg.det(R) > 0


def test_the_live_tidy_places_its_camera_without_the_sims_model(monkeypatch):
    """[Lzt]: with DimOS's transform at the frame's time, the live tidy's head-camera pose needs no MuJoCo and no sim
    scene (the real robot has neither); the sim's kinematic copy is only the fallback"""
    tm = pytest.importorskip("houndkeeper.tidy_module")
    T = np.eye(4); T[:3, 3] = [1.0, 2.0, 0.3]
    class Tr:
        def to_matrix(self): return T
    class Buf:
        def get(self, *a, **k): return Tr()
    class Me:
        tfbuffer = Buf()
    monkeypatch.setitem(sys.modules, "mujoco", None)               # importing it now fails
    assert np.allclose(tm.TidyArm._head_camera(Me(), 123.0), T)


# ---- a find-and-flag night on the dog alone ([jFy]): the preflight ([qDh]), no arm, the night's place ([q66]) ----

def _stops(path, **drop):
    d = {"without_dimos": {"lying": "2026-11-02", "standing": "2026-11-02"},
         "through_dimos": {"lying": "2026-11-02", "standing": "2026-11-02"}, "deadman": "2026-11-01"}
    for k, v in drop.items():
        if v is None: d.pop(k)
        else: d[k].pop(v)
    path.write_text(json.dumps(d)); return path


def _house(root, refused=None, truth=False):
    """a house's run directory as real_nights.sh leaves it: both baselines, the pictures, views, mount and the gate"""
    from houndkeeper import real_night as rn
    h = root / "house"; (h / "shape").mkdir(parents=True)
    for n in (1, 2): (root / f"b{n}.db").write_bytes(b"")
    (h / "recordings.txt").write_text(f"1 {root / 'b1.db'}\n2 {root / 'b2.db'}\n")
    for f in rn.HOUSE_FILES: (h / f).write_text("{}")
    (h / "shape" / "gate.json").write_text(json.dumps({"nights": {"1": {"refused": []}, "2": {"refused": refused or []}}}))
    if truth: (h / "nights_s70_n1.truth.json").write_text("{}")
    return h


def test_the_night_refuses_to_start_naming_each_thing_not_in_order(tmp_path):
    """[jFy], [qDh]: no stops on record, a failed isolation check, no house frame and no operating area each refuse
    the night, all named at once; with them in order it starts"""
    from houndkeeper import real_night as rn
    bad = lambda real, now: ["the robot network's isolation check failed: robot reached out"]
    why, _ = rn.preflight(tmp_path / "nohouse", "real", None, tmp_path / "nostops.json", bad)
    text = "\n".join(why)
    assert len(why) == 4 and "stops are not on record" in text and "isolation check failed" in text
    assert "no house frame" in text and "no operating area" in text
    area = tmp_path / "area.json"; area.write_text("{}")
    why, notes = rn.preflight(_house(tmp_path), "real", area, _stops(tmp_path / "stops.json"), lambda real, now: [])
    assert why == [] and notes == []


@pytest.mark.parametrize("drop,what", [(dict(deadman=None), "dead-man"), (dict(without_dimos="standing"), "without dimos has no date tested standing"),
                                       (dict(through_dimos="lying"), "through dimos has no date tested lying")])
def test_each_stop_is_tested_lying_and_standing_and_the_deadman_passed(tmp_path, drop, what):
    """[qDh], [U4p]: a stop not tested in either pose, or the dead-man not passed, refuses the night and says which"""
    from houndkeeper import real_night as rn
    assert rn.stops_problems(_stops(tmp_path / "s.json")) == []
    p = rn.stops_problems(_stops(tmp_path / "s.json", **drop))
    assert len(p) == 1 and what in p[0]


def test_the_house_must_be_complete_pass_its_gate_and_hold_no_truth(tmp_path):
    """[qDh]: a house whose frame the gate refused, whose baseline recording is gone, whose near views are missing or
    which holds a sim truth file is not in order"""
    from houndkeeper import real_night as rn
    assert rn.house_problems(_house(tmp_path / "a")) == []
    assert any("refused by its gate" in p and "night 2" in p for p in rn.house_problems(_house(tmp_path / "b", refused=["turned 40 deg"])))
    assert any("holds sim truth" in p for p in rn.house_problems(_house(tmp_path / "c", truth=True)))
    h = _house(tmp_path / "d"); (h / "views_near_n2.npz").unlink(); (tmp_path / "d" / "b1.db").unlink()
    p = rn.house_problems(h)
    assert any("views_near_n2.npz" in x for x in p) and any("night 1's recording is gone" in x for x in p)


def test_the_stand_in_skips_only_the_dogs_own_checks_and_says_so(tmp_path):
    """[jFy], [qDh]: the sim robot standing in for the dog is not asked for the dog's stops or robot network, but the
    disk and the house are still checked, and the notes say what was skipped"""
    from houndkeeper import real_night as rn
    seen = []
    why, notes = rn.preflight(_house(tmp_path), "sim", None, tmp_path / "nostops.json", lambda real, now: seen.append(real) or [])
    assert why == [] and seen == [False] and any("stops" in n for n in notes) and any("operating area" in n for n in notes)
    why, _ = rn.preflight(tmp_path / "nohouse", "sim", None, None, lambda real, now: [])
    assert why and "no house frame" in why[0]


def test_the_launcher_refuses_before_any_robot_session(tmp_path):
    """[jFy]: real/night.sh stops at the preflight, naming what is missing, before DimOS or tonight's directory"""
    import os, subprocess
    env = dict(os.environ, HK_HOUSE=str(tmp_path / "nohouse"), HK_REAL_NIGHTS=str(tmp_path / "nights"), ROBOT="real")
    env.pop("AREA", None)
    r = subprocess.run(["bash", str(HK / "real" / "night.sh")], env=env, capture_output=True, text=True, timeout=120)
    assert r.returncode == 1 and "not starting the night" in r.stdout and "no house frame" in r.stdout
    assert not (tmp_path / "nights").exists()
    r = subprocess.run(["bash", str(HK / "real" / "night.sh")], env=dict(env, ROBOT="sim"), capture_output=True, text=True, timeout=60)
    assert r.returncode == 2 and "STANDIN_ARGS" in r.stdout


def test_the_launcher_reads_no_sim_file():
    """[jFy], [1B2]: the night's launcher names no truth file, no house generator, scorer or sim-only module, and the
    robot code it runs is held by the truth boundary's guard"""
    import test_truth_boundary as tb
    text = (HK / "real" / "night.sh").read_text()
    code = "\n".join(l.split("#", 1)[0] for l in text.splitlines() if not l.lstrip().startswith("#"))
    assert not tb.TRUTH.search(code)
    names = set(__import__("re").findall(r"([A-Za-z_]\w*)\.py\b", code)) | set(__import__("re").findall(r"-m houndkeeper\.(\w+)", code))
    assert names and not names & (tb.GENERATOR | tb.SCORERS | tb.SIM | tb.ANALYSIS)
    assert "real/night.sh" in tb.ROBOT_SHELL and "src/houndkeeper/real_night.py" in tb.ROBOT


def test_without_the_arm_the_walk_watches_only_and_reports_what_it_skipped(monkeypatch):
    """[jFy]: with no arm the tidy walks its route watching only, loads nothing of the arm, and its report names every
    step skipped; with no route it refuses"""
    tm = pytest.importorskip("houndkeeper.tidy_module")
    ARM_STEPS = tm.ARM_STEPS
    walked = []
    class Me:
        WATCH_ONLY = False
        def __init__(self): self._status = {"state": "starting", "log": []}; self.said = []
        def _say(self, t): self.said.append(t)
        def _sweep_and_pick(self, sw): walked.append((sw, self.WATCH_ONLY))
    monkeypatch.setitem(sys.modules, "pick", None)                 # nothing of the arm may load
    me = Me(); tm.TidyArm._walk_without_arm(me, {"route": [[1, 0]], "run_dir": "h", "night": 3})
    assert walked == [({"route": [[1, 0]], "run_dir": "h", "night": 3}, True)] and me._ctx == {"handled": []}
    assert me._status["arm"] == "none" and me._status["arm_skipped"] == list(ARM_STEPS) and "no arm" in me.said[0]
    with pytest.raises(RuntimeError, match="only a sweep"):
        tm.TidyArm._walk_without_arm(Me(), None)


def test_a_dog_alone_night_is_laid_out_and_its_log_says_the_arm_was_skipped(tmp_path, monkeypatch):
    """[jFy], [q66], [oWO]: tonight's directory under the real nights' root, named by its evening; night/ holds the
    report (the walk's own: no arm, the steps skipped), the recording's name, night.json with its route, and the log,
    which says the arm's steps were skipped, takes the battery from the robot's own stream, and cuts each place's
    picture from the light check's best view of it; the Morning page finds it beside the sim's nights"""
    import datetime as dt
    import real_tidy as rt
    from houndkeeper import morning_log as ml, real_night as rn
    monkeypatch.setenv("HK_REAL_NIGHTS", str(tmp_path / "nights"))
    assert rn.night_name(dt.datetime(2026, 11, 3, 2, 30)) == "20261102" and rn.night_name(dt.datetime(2026, 11, 3, 21)) == "20261103"
    tn = rn.tonight_dir(dt.datetime(2026, 11, 3, 21)); assert tn == tmp_path / "nights" / "20261103"
    db = tmp_path / "rec" / "memory.db"; db.parent.mkdir(); db.write_bytes(b"")
    s = np.linspace(0, 1, 50)
    monkeypatch.setattr(ml, "recording_odometry", lambda p: np.stack([1.79e9 + 600 * s, 3 * s, 0 * s, 0 * s], 1))
    monkeypatch.setattr(ml, "kind_of", lambda rid: "real")
    monkeypatch.setattr(ml.Night, "battery", lambda self: (82.0, 74.4))
    v = {"score": 0.6, "kept": True, "views": [{"ts": 1.79e9 + 61, "box_full": [10, 20, 40], "score": 0.6},
                                              {"ts": 1.79e9 + 62, "box_full": [0, 0, 40], "score": 0.1}]}
    c, how = rt.candidates([_decision(1.0, 1.0, ts=1.79e9 + 60, check=v)])
    assert how["checked"] and c[0]["view"] == {"ts": 1.79e9 + 61, "box_full": [10, 20, 40]}
    (tn / "night").mkdir(parents=True); (tn / "night" / "route.json").write_text(json.dumps({"waypoints": [[0, 0], [2, 0]], "length_m": 2.0}))
    steps = ["the look from standing", "the close look", "the pick", "the bin"]          # tidy_module.ARM_STEPS
    walk = {"state": "done", "arm": "none", "arm_skipped": steps, "sweep": [{"waypoint": 2, "not_reached": "gave up"}], "items": []}
    d = rt.night_dir(tn / "night", c, db, 1.79e9, 1.79e9 + 600, walk, "find-and-flag", "route.json")
    rep = json.loads((d / "report.json").read_text()); nj = json.loads((d / "night.json").read_text())
    assert rep["arm"] == "none" and rep["arm_skipped"] == steps and rep["walk_state"] == "done"
    assert rep["sweep_waypoints_not_reached"] == [{"waypoint": 2, "not_reached": "gave up"}] and rep["items"][0]["view"]["box_full"] == [10, 20, 40]
    assert (nj["mode"], nj["route"]) == ("find-and-flag", "route.json")
    ml.write(d)
    doc = json.loads((d / "morning_log.json").read_text())
    assert doc["facts"]["no_arm"]["source"] == "report.json" and any("no arm as yet" in p and "the close look" in p for p in doc["prose"])
    assert (doc["facts"]["battery_used_pct"]["value"], doc["facts"]["battery_left_pct"]["value"]) == (8, 74)
    assert doc["facts"]["battery_left_pct"]["source"] == "recording.txt" and not any("no account of the battery" in p for p in doc["prose"])
    img = np.zeros((100, 120, 3), np.uint8); img[20:60, 10:50] = 200
    monkeypatch.setattr(ml.Night, "frame", lambda self, ts: img if ts == 1.79e9 + 61 else None)
    crop = ml.view_crop(ml.Night(d), rep["items"][0]["view"])
    assert crop[0].shape == (40, 40, 3) and crop[0].min() == 200 and crop[1:4] == ([10, 20, 50, 60], [120, 100], "recording.txt")
    corners = ml.view_crop(ml.Night(d), {"ts": 1.79e9 + 61, "box_full": [10, 20, 50, 60]})   # the robot's check writes corners
    assert corners[0].shape == (40, 40, 3) and corners[1] == [10, 20, 50, 60]
    sim_root = tmp_path / "dimos_tidy"; (sim_root / "20261103_200000").mkdir(parents=True)
    (sim_root / "20261103_200000" / "morning_log.json").write_text("{}")
    assert ml.latest_night([sim_root, tmp_path / "nights"]) == sim_root / "20261103_200000"   # began 8 pm, the real one 1.79e9 (Sep 2026)
    nj["started"] = dt.datetime(2026, 11, 3, 21).timestamp(); (d / "night.json").write_text(json.dumps(nj))
    assert ml.latest_night([sim_root, tmp_path / "nights"]) == d


def test_a_night_that_fails_before_its_walk_ends_leaves_nothing_where_nights_are_counted(tmp_path):
    """[jFy], [oWO]: a night past its preflight that cannot finish its walk (here: no route on a house of empty files)
    leaves nothing in the nights' root, where the scorer counts a night and the first two are tuning nights; its work
    is kept beside the root, marked aborted"""
    import os, subprocess
    h = _house(tmp_path); fake = tmp_path / "bin"; fake.mkdir()          # no other DimOS session, as far as it can tell
    (fake / "pgrep").write_text("#!/bin/sh\nexit 1\n"); (fake / "pgrep").chmod(0o755)
    env = dict(os.environ, PATH=f"{fake}:{os.environ['PATH']}", HK_HOUSE=str(h), HK_REAL_NIGHTS=str(tmp_path / "nights"), ROBOT="sim",
               STANDIN_ARGS="--simulation mujoco --mujoco-room none", CUDA_VISIBLE_DEVICES="")
    r = subprocess.run(["bash", str(HK / "real" / "night.sh")], env=env, capture_output=True, text=True, timeout=300)
    assert r.returncode == 1 and not (tmp_path / "nights").exists()
    assert "no route" in r.stdout and len(list((tmp_path / "nights.work").glob("*/ABORTED"))) == 1
