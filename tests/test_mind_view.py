"""[RNC] What he thinks, live: the mind summary built from the tidy module's own state ([ULM]), its size and rate
bounded, every reason the deciding module's own words ([1XG]); the candidates the camera sees projected onto the raw
frame within a pixel of their floor position ([D1U]); the page reads only ([ls3]) and renders from a recorded message
([Ttd], tests/js/hk_mind.test.mjs)."""
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
import time
from pathlib import Path

from houndkeeper import workspace
from types import SimpleNamespace

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "src"))
from houndkeeper import mind  # noqa: E402

FIXTURE = HK / "tests" / "fixtures" / "mind_message.json"
K720 = (797.4756164864929, 796.4872112769983, 643.5352167821186, 349.2783605343087)   # front_camera_720.yaml
D720 = np.array([-0.07309428880537933, -0.02341140740909078, -0.0069305931780026956, 0.009238684474464793])


def camera(x, y, yaw, h=0.30, pitch_deg=25.0):
    """world <- optical (x right, y down, z ahead) for a head camera at (x, y, h), looking along yaw, pitched down"""
    p = math.radians(pitch_deg)
    z = np.array([math.cos(p) * math.cos(yaw), math.cos(p) * math.sin(yaw), -math.sin(p)])
    xr = np.array([math.sin(yaw), -math.cos(yaw), 0.0]); yd = np.cross(z, xr)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = xr, yd, z, (x, y, h)
    return T


def quat(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def night(now=1000.0, view=True):
    """a night part way through: three stops (binned, nothing there, under way) and the sweep's records"""
    rec = lambda x, y, d, hits, seen, **kw: dict(x=x, y=y, decision=d, hits=hits, seen=seen, area_cm2=40, range_m=1.1, **kw)
    sweep = [rec(1.0, 0.5, "confirmed", 6, 8, kind="present", why="confirmed on approach at 1.3 m"),
             rec(2.0, 1.0, "confirmed", 5, 7, kind="present", why="confirmed on approach at 0.9 m"),
             rec(0.5, 1.2, "confirmed", 6, 9, kind="present", why="confirmed on approach at 1.1 m"),
             rec(-1.0, 2.0, "not_a_find", 4, 9, kind="floor", why_not="the floor matches a few cm off"),
             rec(3.0, -1.0, "confirmed", 2, 6, kind="present", why="confirmed, too little for a stop (2 hits)"),
             rec(-2.0, -2.0, "unsure", 1, 3)]
    items = [{"flag": [1.0, 0.5], "confirm": dict(sweep[0]), "label": "toy car", "score": 0.82, "outcome": "binned"},
             {"flag": [2.0, 1.0], "confirm": dict(sweep[1]), "standing": "standing look: nothing near the flag",
              "outcome": "review: not seen standing"},
             {"flag": [0.5, 1.2], "confirm": dict(sweep[2]), "standing": "standing look: dinosaur (0.41) 12 cm from the flag"}]
    status = {"state": "LOOK STANDING", "items": items, "log": []}
    route = [np.array(w, float) for w in ((0, 0), (1, 0), (2, 0), (2, 2), (0, 2), (-2, 2))]
    pose = (0.0, 0.2, *quat(math.pi / 2))
    return dict(status=status, sweep=sweep, goal={"x": 0.5, "y": 1.2, "kind": "find", "why": "confirmed on approach at 1.1 m"},
                route=route, route_at=2, phase=None, note="item 3: walking to park head-on", pose=pose,
                view=(camera(0.0, 0.2, math.pi / 2), K720, D720, (1280, 720), now - 0.2) if view else None,
                worth=lambda r: r.get("hits", 0) >= 4, sweep_arm=False)


def build(state, now=1000.0):
    return mind.decode(mind.Mind().json(state, now=now))


# --- [ULM] the summary ---

def test_the_summary_has_its_shape():
    m = build(night())
    assert set(m) == {"v", "t", "stage", "step", "note", "pose", "goal", "route", "cands", "more", "arm", "view"}
    assert m["v"] == 1 and m["stage"] == "look closer" and m["step"] == "LOOK STANDING"
    assert m["pose"][:2] == [0.0, 0.2] and abs(m["pose"][2] - math.pi / 2) < 1e-3
    assert m["goal"] == {"x": 0.5, "y": 1.2, "kind": "find", "why": "confirmed on approach at 1.1 m"}
    assert m["route"]["plan"] == [[2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [-2.0, 2.0]]           # from the waypoint under way
    assert m["arm"] == {"state": "looking", "item": None}
    for c in m["cands"]:
        assert set(c) == {"id", "x", "y", "r", "status", "conf", "why", "label"} and c["status"] in mind.STATUSES
        assert c["conf"] is None or 0 <= c["conf"] <= 1


def test_each_find_once_with_the_status_its_stop_gave_it():
    by = {c["id"]: c for c in build(night())["cands"]}
    assert [by[i]["status"] for i in ("i1", "i2", "i3")] == ["picked", "ruled out", "looked at"]
    assert by["i1"]["label"] == "toy car" and by["i1"]["conf"] == 0.82                   # the close look's own score
    assert by["i3"]["conf"] == round(6 / 9, 2)                                           # the confirmer's share of frames
    assert by["s4"]["status"] == "ruled out" and by["s5"]["status"] == "ruled out"       # not a find; below the stop gate
    assert by["s6"]["status"] == "new"                                                   # unsure: still in question
    assert not any(k in by for k in ("s1", "s2", "s3"))                                  # the stops' records: one candidate each


@pytest.mark.parametrize("outcome,status", [("binned", "picked"), ("same as item 2", "ruled out"),
                                            ("review: not seen at the flag", "ruled out"), ("review: not re-seen", "ruled out"),
                                            ("review: not seen standing", "ruled out"), ("review: too wide to grip", "looked at")])
def test_a_stops_outcome_gives_its_status(outcome, status):
    assert mind.outcome_status({"outcome": outcome}) == (status, outcome)


def test_ruled_out_is_what_the_tidy_module_clears():
    """the summary's "nothing there" outcomes are the ones the tidy module itself stops keeping the feet off ([tmq])"""
    tidy = pytest.importorskip("houndkeeper.tidy_module")
    for o in mind.NOTHING_THERE:
        r = object.__new__(tidy.TidyArm); r._binned, r._cleared, r._ctx = [], [], {"handled": []}
        r._settled(1, (0.0, 0.0), {"outcome": f"review: {o}"})
        assert len(r._cleared) == 1, o


def test_reasons_are_the_deciding_modules_words():
    """[1XG]: every reason in the message is a string the night's state already held (or the confirmer's decision)"""
    st = night(); m = build(st)
    held = {st["goal"]["why"], st["note"]}
    for r in st["sweep"]: held |= {r.get("why"), r.get("why_not"), r["decision"].replace("_", " ")}
    for rep in st["status"]["items"]: held |= {rep.get("outcome"), rep.get("standing"), rep.get("refine")}
    whys = [m["goal"]["why"]] + [c["why"] for c in m["cands"] if c["why"] is not None]
    assert whys and all(w in held for w in whys), [w for w in whys if w not in held]
    long = night(); long["goal"]["why"] = "the far corner: not yet seen " * 10
    w = build(long)["goal"]["why"]
    assert len(w) == mind.WHY_CHARS and w.endswith("...") and long["goal"]["why"].startswith(w[:-3])


def test_the_tidy_module_sets_its_reasons():
    """the tidy module's decisions carry their reason into the sweep's log and the goal, as it chose them"""
    tidy = pytest.importorskip("houndkeeper.tidy_module")
    r = object.__new__(tidy.TidyArm)
    recs = [dict(x=1.0, y=0.0, decision="confirmed", kind="present", hits=2, range_m=1.4, area_cm2=30),
            dict(x=2.0, y=0.0, decision="confirmed", kind="present", hits=6, range_m=1.2, area_cm2=30)]
    r._cf = SimpleNamespace(decide=lambda t, p: [dict(x) for x in recs])
    r._sweep_log, r._sidestepped = [], {}
    r._pose = lambda: (np.zeros(3), None)
    got = r._decisions()
    assert got["why"] == "confirmed on approach at 1.2 m" and r._sweep_log[1]["why"] == got["why"]
    assert r._sweep_log[0]["why"] == "confirmed, too little for a stop (2 hits)"
    r._aim(np.array([2.0, 0.0]), got["why"], "find")
    assert r._goal == {"x": 2.0, "y": 0.0, "kind": "find", "why": got["why"]}


def test_the_tidy_modules_hook_reads_its_state():
    tidy = pytest.importorskip("houndkeeper.tidy_module")
    assert tidy.TidyArm.MIND is (os.environ.get("HK_MIND") == "1")                     # off unless asked for
    r = object.__new__(tidy.TidyArm)
    st = night(view=False)
    r._status, r._sweep_log, r._route, r._phase, r._note = st["status"], st["sweep"], st["route"], "INTO BIN", "over the bin"
    r._odom = SimpleNamespace(position=SimpleNamespace(x=0.5, y=1.0, z=0.3), orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))
    r._aim((3.0, 1.0), "the sweep, waypoint 4 of 6", "waypoint", wp=3)
    st["status"]["items"][-1]["label"] = "dinosaur"
    m = build(r._mind_state())
    assert m["stage"] == "put away" and m["arm"] == {"state": "into the bin", "item": "dinosaur"}
    assert m["goal"]["why"] == "the sweep, waypoint 4 of 6" and m["route"]["plan"][0] == [2.0, 2.0] and m["pose"] == [0.5, 1.0, 0.0]


def test_a_huge_night_still_fits():
    """the size bound holds whatever the night holds (and drops detail, never the stage or the goal)"""
    rng = np.random.default_rng(0); long = "x" * 1000
    sweep = [dict(x=float(x), y=float(y), decision="confirmed", hits=5, seen=6, area_cm2=30, why=long) for x, y in rng.uniform(-8, 8, (3000, 2))]
    items = [{"flag": [float(x), float(y)], "outcome": "review: " + long, "label": long} for x, y in rng.uniform(-8, 8, (500, 2))]
    st = night(); st["status"] = {"state": "SWEEP", "items": items}; st["sweep"] = sweep; st["goal"]["why"] = long
    st["route"] = [np.array(w) for w in rng.uniform(-8, 8, (400, 2))]; st["route_at"] = 0
    mi = mind.Mind()
    for x, y in np.cumsum(rng.normal(0, 0.3, (5000, 2)), 0):
        mi.walked.add(float(x), float(y))
    assert len(mi.walked.pts) <= mind.WALKED_KEEP + 1
    text = mi.json(st, now=1000.0)
    m = mind.decode(text)
    assert len(text) <= mind.MAX_BYTES and m is not None
    assert m["stage"] == "sweep" and m["goal"]["why"].endswith("...") and m["more"] > 0 and len(m["cands"]) <= mind.MAX_CANDS


def test_a_usual_night_is_small_and_cheap_to_build():
    st = night(); t = []
    for _ in range(7):
        t0 = time.perf_counter(); text = mind.Mind().json(st, now=1000.0); t.append(time.perf_counter() - t0)
    assert len(text) < 4096 and statistics.median(t) < 0.02


def test_the_rate_is_bounded():
    pc = pytest.importorskip("houndkeeper.phone_cockpit")
    from dimos.web.cockpit import cockpit
    man = cockpit(**pc.phone_layout()).blueprints[0].kwargs["manifest"]
    ch = {c["ch"]: c for c in man["channels"]}
    assert mind.PERIOD_S >= 1 / 3 - 1e-9 and ch["mind"]["maxHz"] <= 3.0 and ch["mind"]["delivery"] == "latest"
    assert mind.MAX_BYTES * ch["mind"]["maxHz"] <= 24 * 1024


def test_the_feed_passes_only_mind_messages():
    cd = pytest.importorskip("houndkeeper.cockpit_data")
    f = object.__new__(cd.MindFeed); sent = []; f.mind = SimpleNamespace(publish=sent.append)
    good = mind.Mind().json(night(), now=1.0)
    f._on_json(SimpleNamespace(data=good)); f._on_json(SimpleNamespace(data=json.dumps({"v": 2})))
    f._on_json(SimpleNamespace(data="{" + " " * mind.MAX_BYTES + "}")); f._on_json(SimpleNamespace(data="not json"))
    assert len(sent) == 1 and sent[0]["stage"] == "look closer"


# --- [D1U] over the camera ---

def test_projection_matches_opencvs_fisheye():
    cv2 = pytest.importorskip("cv2")
    T = camera(0.0, 0.0, 0.3); rng = np.random.default_rng(1)
    pts = np.c_[rng.uniform(0.5, 3.0, 200), rng.uniform(-1.5, 1.5, 200), np.zeros(200)]
    uv = mind.project(pts, T, K720, D720, (1280, 720)) * [1280, 720]
    Ti = np.linalg.inv(T); rv, _ = cv2.Rodrigues(Ti[:3, :3])
    Km = np.array([[K720[0], 0, K720[2]], [0, K720[1], K720[3]], [0, 0, 1.0]])
    ref, _ = cv2.fisheye.projectPoints(pts.reshape(-1, 1, 3), rv, Ti[:3, 3], Km, D720)
    assert np.abs(uv - ref.reshape(-1, 2)).max() < 0.5


def test_an_outline_lands_on_its_floor_point():
    """a candidate's projected centre, back through the lens onto the floor, is where it lies (within 1 cm), and its
    outline is in the frame; out of view, behind the camera or with a stale pose, no outline"""
    cv2 = pytest.importorskip("cv2")
    T = camera(0.0, 0.0, 0.0); Km = np.array([[K720[0], 0, K720[2]], [0, K720[1], K720[3]], [0, 0, 1.0]])
    for x, y in ((1.0, 0.0), (1.5, 0.4), (0.8, -0.3)):
        u, v = mind.project(np.array([[x, y, 0.0]]), T, K720, D720, (1280, 720))[0] * [1280, 720]
        a, b = cv2.fisheye.undistortPoints(np.array([[[u, v]]]), Km, D720).ravel()
        ray = T[:3, :3] @ np.array([a, b, 1.0]); hit = T[:3, 3] + ray * (-T[2, 3] / ray[2])
        assert math.hypot(hit[0] - x, hit[1] - y) < 0.01
    cands = [dict(id="a", x=1.0, y=0.0, r=0.08, status="new"), dict(id="b", x=-1.0, y=0.0, r=0.08, status="new"),
             dict(id="c", x=1.2, y=0.1, r=0.08, status="ruled out")]
    view = (T, K720, D720, (1280, 720), 99.8)
    got = mind.in_view(cands, view, now=100.0)
    assert [g["id"] for g in got] == ["a"] and len(got[0]["ring"]) == mind.RING
    assert all(0 <= u <= 1 and 0 <= v <= 1 for u, v in got[0]["ring"])
    assert mind.in_view(cands, view, now=100.0 + mind.VIEW_MAX_AGE_S) == []


# --- [ls3] [Ttd] the page ---

def test_the_page_reads_only():
    pc = pytest.importorskip("houndkeeper.phone_cockpit")
    from dimos.web.cockpit import cockpit
    man = cockpit(**pc.phone_layout()).blueprints[0].kwargs["manifest"]
    panel = next(p for p in man["panels"] if p["kind"] == "hk-mind")
    ch = {c["ch"]: c for c in man["channels"]}
    assert panel["id"] in man["pages"] and panel["channels"][0] == "mind"
    assert all(ch[c]["dir"] == "rx" and ch[c]["publish"] == "none" for c in panel["channels"])


def test_the_recorded_message_is_a_mind_message():
    """the fixture the page's tests render from is what the summary makes (regenerate: python tests/test_mind_view.py)"""
    m = mind.decode(FIXTURE.read_text())
    assert m == build(night()), "the summary changed: regenerate the fixture"


def test_the_page_renders_from_a_recorded_message():
    node = shutil.which("node")
    react = Path(os.environ.get("HK_REACT_DIR", workspace.sibling("dimos-ext") / "web" / "node_modules"))
    if node is None or not (react / "react").exists(): pytest.skip("node or the cockpit's React not here")
    r = subprocess.run([node, "--test", str(HK / "tests" / "js" / "hk_mind.test.mjs")], capture_output=True, text=True,
                       env=dict(os.environ, HK_REACT_DIR=str(react)), timeout=120)
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-3000:]


if __name__ == "__main__":                                       # writes the fixture the page's tests render from
    FIXTURE.parent.mkdir(exist_ok=True)
    FIXTURE.write_text(json.dumps(build(night()), indent=1) + "\n"); print(FIXTURE)
