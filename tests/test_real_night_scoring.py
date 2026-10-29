"""[vO9], [71R], [wTj], [oWO], [Vd3]: a real night's score kept as files beside the night and made from the phone: the
marks laid out before the night (houndkeeper.night_scoring, the Marks page's side), each flag near no mark judged (the
Review page's side) until the night is final, the first two real nights kept apart as tuning nights when pooled
(real/tests/score_night.py), and the flags' crops cut from the recording, people-screened, with nothing kept from an
unscreened recording (real/tests/flag_crops.py)."""
import base64
import datetime as dt
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "src")); sys.path.insert(0, str(HK / "real" / "tests")); sys.path.insert(0, str(HK / "sim" / "house"))

import score_night as sn  # noqa: E402


def _toy(i, x, y, floor="hard", brick=False):
    return {"id": f"t{i}", "name": f"toy {i}", "x": x, "y": y, "half_long_m": 0.05, "brick": brick, "floor": floor}


def _cands(*xy):
    return [{"id": f"c{k + 1}", "x": x, "y": y, "decision": "confirmed"} for k, (x, y) in enumerate(xy)]


# ---- pooling keeps the tuning nights apart ([wTj]) ----

def test_the_first_two_real_nights_are_kept_apart_from_the_launch_number():
    """[wTj]: a tuning night's toys and false alarms are reported on their own, never in the launch number, and the
    pool says why; an unjudged tuning night does not hold the launch number back"""
    m = {"night": "a", "toys": [_toy(1, 0, 0), _toy(2, 2, 0)]}
    t1 = sn.score_night(dict(m, night="t1"), _cands((0, 0), (5, 5)), {"c2": "false_alarm"}, tuning=True)
    t2 = sn.score_night(dict(m, night="t2"), _cands((9, 9)), tuning=True)                       # not yet judged
    n3 = sn.score_night(dict(m, night="n3"), _cands((0, 0), (2, 0), (7, 7)), {"c3": "false_alarm"})
    assert (t1["tuning"], n3["tuning"]) == (True, False)
    p = sn.pool([t1, t2, n3])
    assert (p["nights"], p["found"], p["n"], p["fp_per_night"], p["fp"]) == (1, 2, 2, 1.0, {"n3": 1})
    assert p["tuning_nights"] == ["t1", "t2"] and "not counted" in p["note"]
    assert (p["tuning"]["found"], p["tuning"]["n"], p["tuning"]["fp"]) == (1, 4, {"t1": 1, "t2": 0})
    assert p["tuning"]["unjudged"] == ["t2"]
    plain = sn.pool([n3])
    assert plain["tuning_nights"] == [] and plain["tuning"] is None and plain["note"] == ""
    with pytest.raises(SystemExit, match="not yet judged"):
        sn.pool([t1, sn.score_night(dict(m, night="n4"), _cands((9, 9)))])


def test_the_command_line_marks_a_tuning_night(tmp_path):
    """[wTj]: score_night.py night --tuning writes a tuning night's result"""
    (tmp_path / "marks.json").write_text(json.dumps({"night": "x", "toys": [_toy(1, 0, 0)]}))
    (tmp_path / "candidates.json").write_text(json.dumps({"candidates": _cands((0, 0))}))
    sn.main(["night", str(tmp_path / "marks.json"), str(tmp_path / "candidates.json"), "--tuning", "--out", str(tmp_path / "r.json")])
    assert json.loads((tmp_path / "r.json").read_text())["tuning"] is True


# ---- the Marks page's side ([vO9], [oWO]) ----

@pytest.fixture
def ns(monkeypatch, tmp_path):
    pytest.importorskip("dimos")
    from houndkeeper import night_scoring
    monkeypatch.setenv("HK_REAL_NIGHTS", str(tmp_path / "nights"))
    md = tmp_path / "map"; md.mkdir()
    np.save(md / "house_lidar.npy", np.array([[0.0, 0.0, 0.0], [6.0, 4.0, 0.0], [6.0, 4.0, 1.0], [0.0, 4.0, 1.0]]))
    monkeypatch.setenv("HK_MAP_DIR", str(md))
    return night_scoring


def test_tonight_is_the_date_its_evening_began(ns):
    """[oWO]: a night is named by the date its evening began; before 6 am is the evening before"""
    assert ns.night_name(dt.datetime(2026, 10, 15, 21, 30)) == "20261015"
    assert ns.night_name(dt.datetime(2026, 10, 16, 2, 10)) == "20261015"
    assert ns.night_name(dt.datetime(2026, 10, 16, 6, 0)) == "20261016"


def test_marks_are_saved_in_the_scorers_shape_from_the_short_list(ns):
    """[vO9]: a tap per toy in the house's frame, its name from the short list (which gives its length for the sim's
    reach, and whether it is a 1 x 1 brick), its floor kind; saved to marks.json beside the night, logged"""
    d = ns.root() / "20261015"
    m = ns.save_marks(d, [{"name": "1 x 1 brick", "x": 1.0, "y": 2.0, "floor": "rug"},
                          {"name": "long toy (a sword, a wand)", "x": 3.0, "y": 1.0, "floor": "carpet"}], seen="")
    on_disk = json.loads((d / "marks.json").read_text())
    assert on_disk == m and m["night"] == "20261015" and m["frame"] == "house" and m["saved"]
    assert [(t["id"], t["brick"], t["floor"]) for t in m["toys"]] == [("t1", True, "rug"), ("t2", False, "carpet")]
    assert sn.score_night(m, _cands((3.3, 1.0)))["toys"][1]["found"] is True          # a long toy's longer reach
    log = [json.loads(l) for l in (d / "score_actions.log.jsonl").read_text().splitlines()]
    assert log[-1]["what"] == "marks saved" and log[-1]["toys"] == 2 and log[-1]["when"]


@pytest.mark.parametrize("toy,why", [({"name": "rocket", "x": 0, "y": 0, "floor": "hard"}, "short list"),
                                     ({"name": "ball", "x": 0, "y": 0, "floor": "lawn"}, "floor"),
                                     ({"name": "ball", "x": float("nan"), "y": 0, "floor": "hard"}, "number")])
def test_a_mark_off_the_list_or_off_the_map_is_refused(ns, toy, why):
    with pytest.raises(ValueError, match=why):
        ns.save_marks(ns.root() / "20261015", [toy], seen="")


def test_marks_are_refused_once_the_night_has_run_or_when_stale(ns):
    """[oWO]: marks are made before the night, never after its flags are seen; a page that loaded older marks cannot
    overwrite newer ones"""
    d = ns.root() / "20261015"
    a = ns.save_marks(d, [{"name": "ball", "x": 1, "y": 1, "floor": "hard"}], seen="")
    with pytest.raises(ValueError, match="changed since"):
        ns.save_marks(d, [{"name": "ball", "x": 2, "y": 1, "floor": "hard"}], seen="")
    ns.save_marks(d, [{"name": "ball", "x": 2, "y": 1, "floor": "hard"}], seen=a["saved"])
    (d / "candidates.json").write_text(json.dumps({"candidates": []}))
    with pytest.raises(ValueError, match="already run"):
        ns.save_marks(d, [], seen=json.loads((d / "marks.json").read_text())["saved"])


def test_the_marks_message_has_tonight_the_map_the_list_and_what_is_marked(ns):
    """[vO9]: the Marks page's message: tonight's name, the house map in the house's frame, the short list and the floor
    kinds, the marks saved so far, and whether the night has run"""
    now = dt.datetime(2026, 10, 15, 20, 0)
    m = ns.marks_message(now=now)
    assert m["night"] == "20261015" and m["marks"] is None and m["ran"] is False and m["map"]["res"] > 0
    assert [t["name"] for t in m["toys"]][:2] == ["1 x 1 brick", "2 x 2 brick"] and [f["key"] for f in m["floors"]] == list(sn.FLOORS)
    ns.save_marks(ns.root() / "20261015", [{"name": "ball", "x": 1, "y": 1, "floor": "hard"}], seen="")
    assert [t["name"] for t in ns.marks_message(now=now)["marks"]["toys"]] == ["ball"]


# ---- the Review page's side ([71R], [Vd3], [wTj]) ----

def _night(ns, name, toys, cands, crops=None):
    d = ns.root() / name; d.mkdir(parents=True, exist_ok=True)
    if toys is not None: ns.save_marks(d, toys, seen=(json.loads((d / "marks.json").read_text())["saved"] if (d / "marks.json").exists() else ""))
    (d / "candidates.json").write_text(json.dumps({"checked": False, "candidates": cands}))
    if crops:
        (d / "review_crops").mkdir()
        idx = {}
        for cid, ok in crops.items():
            if ok: (d / "review_crops" / f"{cid}.jpg").write_bytes(b"\xff\xd8jpeg" + cid.encode()); idx[cid] = {"file": f"{cid}.jpg"}
            else: idx[cid] = {"withheld": "the people screen found a person"}
        (d / "review_crops" / "index.json").write_text(json.dumps({"note": "", "crops": idx}))
    return d


def test_each_flag_near_no_mark_is_shown_with_its_crop_and_place_until_judged(ns):
    """[71R]: the flags (candidates near no marked toy, merged as the scorer merges them) with their crop, place and
    judgement; a judgement on the page is written to review.json for every candidate in the flag, logged, and the
    night is final only when none is left"""
    d = _night(ns, "20261015", [{"name": "ball", "x": 0, "y": 0, "floor": "hard"}], _cands((0, 0), (3, 3), (3.1, 3.0), (5, 1)),
               crops={"c2": True, "c3": True, "c4": False})
    m = ns.review_message()
    assert m["night"]["name"] == "20261015" and m["night"]["found"] == [1, 1] and m["night"]["final"] is False
    fl = m["flags"]
    assert [f["ids"] for f in fl] == [["c2", "c3"], ["c4"]] and [f["judged"] for f in fl] == [None, None]
    assert base64.b64decode(fl[0]["jpg"]).startswith(b"\xff\xd8") and fl[1]["jpg"] is None and "people screen" in fl[1]["no_picture"]
    assert (fl[0]["x"], fl[0]["y"]) == pytest.approx((3.05, 3.0), abs=0.06) and m["map"] is not None
    r = ns.judge(d, ["c2", "c3"], "false_alarm")
    assert json.loads((d / "review.json").read_text()) == {"c2": "false_alarm", "c3": "false_alarm"} and r["unjudged"] == 1
    r = ns.judge(d, ["c4"], "real_change")
    assert r["final"] and (r["false_alarms"], r["real_changes"]) == (1, 1)
    assert json.loads((d / "result.json").read_text())["final"] is True
    assert ns.review_message()["night"]["final"] is True
    ns.judge(d, ["c4"], None)                                                     # taken back: open again
    assert json.loads((d / "result.json").read_text())["final"] is False
    log = [json.loads(l)["what"] for l in (d / "score_actions.log.jsonl").read_text().splitlines()]
    assert log[-3:] == ["judged false_alarm", "judged real_change", "judgement taken back"]
    with pytest.raises(ValueError, match="false_alarm or real_change"):
        ns.judge(d, ["c2"], "maybe")
    with pytest.raises(ValueError, match="no candidate"):
        ns.judge(d, ["c99"], "false_alarm")


def test_a_night_with_no_marks_still_has_its_flags_judged(ns):
    """[71R], [wTj]: false alarms are counted on every night, laid out or not"""
    _night(ns, "20261015", None, _cands((1, 1)))
    m = ns.review_message()
    assert m["night"]["laid_out"] is False and len(m["flags"]) == 1


def test_the_first_two_nights_that_ran_are_tuning_nights(ns):
    """[wTj], [oWO]: a night's role comes from its place among the nights that ran: the first two tune the light check
    and say so; a night only marked (not run) is no night yet"""
    for n in ("20261014", "20261015", "20261017"): _night(ns, n, None, _cands((1, 1)))
    (ns.root() / "20261016").mkdir()
    assert ns.tuning_names() == ["20261014", "20261015"]
    m = ns.review_message()
    assert [(n["name"], n["tuning"]) for n in m["nights"]] == [("20261017", False), ("20261015", True), ("20261014", True)]
    r = ns.judge(ns.root() / "20261014", ["c1"], "false_alarm")
    assert r["tuning"] is True and json.loads((ns.root() / "20261014" / "result.json").read_text())["tuning"] is True
    assert ns.review_message(show="20261014")["night"]["tuning"] is True


def test_the_review_message_keeps_within_its_bound_dropping_judged_flags_pictures_first(ns):
    """[71R] at phone size: one bounded message; pictures of judged flags are dropped before the unjudged ones', and
    the page says how many were left off"""
    big = {f"c{k}": True for k in range(1, 7)}
    d = _night(ns, "20261015", None, _cands(*[(k * 2.0, 0.0) for k in range(6)]), crops=big)
    for k in range(1, 7): (d / "review_crops" / f"c{k}.jpg").write_bytes(b"\xff\xd8" + bytes(3000))
    ns.judge(d, ["c1"], "false_alarm")
    full = ns.review_message()
    m = ns.review_message(max_bytes=len(json.dumps(full, separators=(",", ":"))) - 3000)
    assert m["flags"][0]["jpg"] is None and all(f["jpg"] for f in m["flags"][1:]) and "left off" in m["pictures_note"]


def test_the_feed_takes_the_pages_commands(ns):
    """[vO9], [71R]: a confirmed save from the Marks page, a judgement and a choice of night from the Review page; an
    unconfirmed save is refused and said"""
    d = _night(ns, "20261015", None, _cands((1, 1)))
    f = object.__new__(ns.ScoreFeed); f._marks_msg = ""; f._review_msg = ""; f._show = None; f._map = None; f._map_stamp = None
    sent = []
    f.marks_state = type("O", (), {"publish": lambda self, m: sent.append(("marks", m))})()
    f.review_state = type("O", (), {"publish": lambda self, m: sent.append(("review", m))})()
    f.on_marks({"op": "save", "confirmed": False, "night": "20261016", "seen": "", "toys": []})
    assert "not confirmed" in f._marks_msg and not (ns.root() / "20261016" / "marks.json").exists()
    f.on_marks({"op": "save", "confirmed": True, "night": "20261016", "seen": "", "toys": [{"name": "ball", "x": 1, "y": 1, "floor": "rug"}]})
    assert (ns.root() / "20261016" / "marks.json").exists() and f._marks_msg.startswith("saved")
    f.on_review({"op": "judge", "night": "20261015", "ids": ["c1"], "verdict": "false_alarm"})
    assert json.loads((d / "review.json").read_text()) == {"c1": "false_alarm"} and sent[-1][0] == "review"
    f.on_review({"op": "show", "night": "../elsewhere"})
    assert "no night" in f._review_msg


def test_the_project_service_carries_the_two_pages():
    """[oWO]: Marks and Review on the project service, each with its state received and its command sent"""
    pytest.importorskip("dimos")
    import houndkeeper.project as pj
    man = pj.project.blueprints[0].kwargs["manifest"]
    kinds = [p["kind"] for p in man["panels"]]
    assert "hk-marks" in kinds and "hk-review" in kinds
    ch = {c["ch"]: c["dir"] for c in man["channels"]}
    assert ch["marks_state"] == "rx" and ch["marks_cmd"] == "tx" and ch["review_state"] == "rx" and ch["review_cmd"] == "tx"


# ---- the flags' crops ([71R], [Vd3]) ----

K = np.array([[600.0, 0, 320], [0, 600.0, 180], [0, 0, 1]]); WH = (640, 360)


def _cam(x, y, yaw):
    f = np.array([math.cos(yaw), math.sin(yaw), 0.0]); left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    p = math.radians(30); z = f * math.cos(p) + np.array([0, 0, -math.sin(p)])
    xo = -left; yo = np.cross(z, xo)
    T = np.eye(4); T[:3, :3] = np.stack([xo, yo, z], 1); T[:3, 3] = [x, y, 0.5]
    return T


def test_each_candidate_gets_one_crop_from_the_frame_nearest_its_decision_and_only_if_clear_of_people():
    """[71R], [Vd3]: a candidate's crop is cut from the frame nearest in time to its decision that sees its floor point
    (the light check's own view choice), around that point; a crop the people screen does not clear is never kept, and a
    candidate no frame saw has none"""
    import flag_crops as fc
    poses = [(k, 1000.0 + k * 0.5, _cam(0.1 * k * 0.5, 0.0, 0.0)) for k in range(60)]
    img = np.zeros((WH[1], WH[0], 3), np.uint8)
    cands = [{"id": "c1", "x_robot": 2.0, "y_robot": 0.0, "ts": 1010.0}, {"id": "c2", "x_robot": 2.5, "y_robot": 0.1, "ts": 1012.0},
             {"id": "c3", "x_robot": -9.0, "y_robot": 9.0, "ts": 1001.0}]
    seen = []
    def screen(c):
        seen.append(c.shape); return len(seen) != 2                       # the second crop holds a "person"
    out = fc.crops_for(cands, poses, lambda k: img, K, None, "", WH, screen)
    assert out["c1"]["img"] is not None and abs(out["c1"]["ts"] - 1010.0) < 2.0
    assert out["c1"]["img"].shape[0] == out["c1"]["img"].shape[1] >= 96
    assert out["c2"]["img"] is None and "people screen" in out["c2"]["withheld"]
    assert out["c3"]["img"] is None and "no frame" in out["c3"]["none"]
    assert len(seen) == 2


def test_no_crop_is_made_from_a_recording_not_cleared_for_its_frames(tmp_path, monkeypatch):
    """[Vd3] ([R1X]): a night whose recording is not screened gets no crops, and the index says why"""
    import flag_crops as fc
    d = tmp_path / "20261015"; (d / "night").mkdir(parents=True)
    (d / "candidates.json").write_text(json.dumps({"candidates": [{"id": "c1", "x_robot": 1, "y_robot": 1, "ts": 1}]}))
    (d / "night" / "recording.txt").write_text(str(tmp_path / "rec" / "memory.db") + "\n")
    def refuse(rid): raise PermissionError(f"{rid} is unscreened: screen it for people before using its frames")
    monkeypatch.setattr(fc, "_cleared", lambda db: refuse("rec"))
    idx = fc.make(d)
    assert idx["crops"] == {} and "withheld" in idx["note"] and not list((d / "review_crops").glob("*.jpg"))


def test_the_review_pages_recorded_message_is_a_review_message(ns):
    """[71R]: tests/fixtures/review_message.json, which the page's tests render from (a drawn map and drawn crops, no
    camera frame), has the shape review_message makes"""
    fix = json.loads((HK / "tests" / "fixtures" / "review_message.json").read_text())
    _night(ns, "20261015", [{"name": "ball", "x": 0, "y": 0, "floor": "hard"}], _cands((0, 0), (3, 3)), crops={"c2": True})
    m = dict(ns.review_message(), message="")
    assert set(fix) == set(m) and fix["v"] == m["v"] and set(fix["night"]) == set(m["night"])
    assert set(fix["flags"][0]) == set(m["flags"][0]) and set(fix["toys"][0]) == set(m["toys"][0]) and set(fix["nights"][0]) == set(m["nights"][0])


# ---- the browser smoke ([vO9], [71R]) ----

def test_the_marks_and_review_pages_at_phone_width(tmp_path):
    """a loopback relay and the project service alone (no robot or sim; its own zenoh discovery bus), a drawn map and a
    night with drawn crops, then tests/browser/score_pages_390.mjs at 390 px: the layout audit on both pages (nothing
    wider than the screen, no text under 13 px, no button under 44 px, nothing moving), a toy marked by taps and saved
    through the confirm dialog, a flag judged. Screenshots: HK_SHOTS_DIR, else this test's tmp dir"""
    import os, shutil, signal, subprocess, time
    pytest.importorskip("dimos")
    from houndkeeper import project_stack as ps
    deno = shutil.which("deno") or str(Path.home() / ".local" / "bin" / "deno")
    need = [Path(deno), ps.DIMOS / "web" / "cockpit" / "dist" / "index.html", HK / "real" / "tests" / "node_modules" / "playwright",
            Path(sys.executable).with_name("dimos")]
    missing = [str(p) for p in need if not p.exists()] + ([] if shutil.which("node") else ["node"])
    if missing: pytest.skip("the browser smoke needs " + ", ".join(missing))
    from PIL import Image
    md = tmp_path / "map"; md.mkdir(); rng = np.random.default_rng(1)
    walls = [[x, y, 1.0] for x in np.arange(0, 8, 0.03) for y in (0, 5)] + [[x, y, 1.0] for y in np.arange(0, 5, 0.03) for x in (0, 8)]
    np.save(md / "house_lidar.npy", np.r_[np.c_[rng.uniform(0, 8, 8000), rng.uniform(0, 5, 8000), np.zeros(8000)], walls])
    nights = tmp_path / "nights"; d = nights / "20261012"; (d / "review_crops").mkdir(parents=True)
    (d / "marks.json").write_text(json.dumps({"night": d.name, "frame": "house", "saved": "2026-10-12T20:00:00",
                                              "toys": [_toy(1, 1, 1), _toy(2, 6, 1, "rug", brick=True)]}))
    (d / "candidates.json").write_text(json.dumps({"checked": False, "candidates": _cands((1, 1), (3, 3), (7, 4))}))
    for c in ("c2", "c3"): Image.fromarray(np.full((240, 240, 3), 140, np.uint8)).save(d / "review_crops" / f"{c}.jpg")   # drawn, no frame
    (d / "review_crops" / "index.json").write_text(json.dumps({"note": "", "crops": {"c2": {"file": "c2.jpg"}, "c3": {"file": "c3.jpg"}}}))
    shots = Path(os.environ.get("HK_SHOTS_DIR") or tmp_path / "shots")
    relay = subprocess.Popen(ps.relay_argv("127.0.0.1", 0, deno), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                             cwd=HK, start_new_session=True)
    svc = None
    try:
        port = None; t0 = time.time()
        while time.time() - t0 < 60 and port is None:
            line = relay.stdout.readline()
            if line.startswith("{"): port = json.loads(line).get("httpPort")
        assert port, "relay not ready"
        url = f"http://127.0.0.1:{port}"; log = open(tmp_path / "project.log", "w")
        env = {**ps.project_env(url), "HK_REAL_NIGHTS": str(nights), "HK_MAP_DIR": str(md), "HK_PROJECT_RUNS": str(tmp_path / "runs"),
               "ZENOH_SCOUT_ADDR": "224.0.0.232:7446"}
        svc = subprocess.Popen(ps.project_argv(url), env=env, stdout=log, stderr=subprocess.STDOUT, cwd=HK, start_new_session=True)
        t0 = time.time()
        while time.time() - t0 < 120 and "relay bridge up" not in (tmp_path / "project.log").read_text():
            assert svc.poll() is None, (tmp_path / "project.log").read_text()[-2000:]
            time.sleep(1)
        r = subprocess.run(["node", str(HK / "tests" / "browser" / "score_pages_390.mjs"), url + "/", str(shots)],
                           capture_output=True, text=True, timeout=240)
        out = json.loads(r.stdout.strip().splitlines()[-1])
        assert out["ok"], out
        assert any("for the night of" in x for x in out["dialogs"]) and "toy car" in out["marks_text"]
        assert "2 left to judge" in out["review_before"] and "1 left to judge" in out["review_after"]
        assert json.loads((d / "review.json").read_text()) and len(out["shots"]) == 6
    finally:
        for p in (svc, relay):
            if p is not None and p.poll() is None:
                os.killpg(p.pid, signal.SIGTERM)
                try: p.wait(20)
                except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL)
