"""The operating area ([JVr]): stored with the map in the house's frame, each change recorded with its time ([daF]);
published to DimOS's costmap layer in the odometry's frame, carrying the area and nothing else ([rLQ], [mvn]); used by
the tidy so nothing outside it is targeted ([OrO]); and the tidy's safety when unsure ([Qd5])."""
import json, math, sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src")); sys.path.insert(0, str(Path(__file__).resolve().parent))
oa = pytest.importorskip("houndkeeper.operating_area")
from houndkeeper.operating_area_module import AreaPublisher   # noqa: E402

TWO_ROOMS = {"allowed": [[[0, 0], [4, 0], [4, 3], [0, 3]], [[4, 0], [7, 0], [7, 2], [4, 2]]],
             "walls": [[[2, 0], [2, 1]]]}


def test_saved_with_the_map_in_its_frame_and_each_change_logged(tmp_path):
    a = oa.save(tmp_path, TWO_ROOMS, by="cockpit", now=1_790_000_000)
    d = json.loads((tmp_path / oa.FILE).read_text())
    assert d["frame_id"] == "house" and d["by"] == "cockpit" and d["changed"].startswith("2026-")
    assert oa.load(tmp_path).allowed == a.allowed
    oa.save(tmp_path, {"allowed": [[[0, 0], [3, 0], [3, 3]]]}, by="cockpit")
    log = [json.loads(x) for x in (tmp_path / oa.LOG).read_text().splitlines()]
    assert len(log) == 2 and len(log[0]["allowed"]) == 2 and len(log[1]["allowed"]) == 1 and all(e["changed"] for e in log)


@pytest.mark.parametrize("bad, why", [({"allowed": []}, "at least one"), ({"allowed": [[[0, 0], [1, 1]]]}, "3 or more"),
                                      ({"allowed": [[[0, 0], [0.1, 0], [0, 0.1]]]}, "0.25"),
                                      ({"allowed": [[[0, 0], [9e9, 0], [0, 3]]]}, "1 km"),
                                      ({"frame_id": "world", "allowed": TWO_ROOMS["allowed"]}, "house"),
                                      ({"allowed": [[[0, 0], [float("nan"), 0], [0, 3]]]}, "not a number")])
def test_a_bad_area_is_refused_and_nothing_written(tmp_path, bad, why):
    with pytest.raises(ValueError, match=why): oa.save(tmp_path, bad, by="t")
    assert not (tmp_path / oa.FILE).exists() and not (tmp_path / oa.LOG).exists()


def test_a_dock_outside_the_area_is_refused(tmp_path):
    """[OrO]: the dock, if outside, is refused when the area is set"""
    with pytest.raises(ValueError, match="dock"): oa.save(tmp_path, TWO_ROOMS, by="t", dock=(6.0, 2.5))
    oa.save(tmp_path, TWO_ROOMS, by="t", dock=(6.0, 1.0))


def test_no_area_is_none_and_a_broken_one_raises(tmp_path):
    assert oa.load(tmp_path) is None and oa.load(None) is None
    (tmp_path / oa.FILE).write_text("{not json")
    with pytest.raises(ValueError): oa.load(tmp_path)


def test_published_in_the_odometry_frame_by_the_correction():
    """[rLQ]: the area (house frame) placed where the costmap is built (world), by the localization's world -> house"""
    a = oa.validate(TWO_ROOMS); th = math.radians(10)
    C = np.array([[math.cos(th), -math.sin(th), 0.5], [math.sin(th), math.cos(th), -0.2], [0, 0, 1]])
    z = json.loads(oa.zones_json(a, C))
    assert z["frame_id"] == "world"
    w = np.array(z["allowed"][0]); back = w @ C[:2, :2].T + C[:2, 2]
    assert np.allclose(back, TWO_ROOMS["allowed"][0], atol=1e-3)
    assert np.allclose(json.loads(oa.zones_json(a))["allowed"][0], TWO_ROOMS["allowed"][0])   # no correction: as drawn


def test_the_message_carries_the_area_and_no_sensor_data():
    """[mvn]: the layer gets only the boundary; nothing goes into the lidar or point cloud streams"""
    sent = []
    pub = AreaPublisher("/nonexistent", sent.append)
    pub.area = oa.validate(TWO_ROOMS); pub._stamp = None
    pub.reload = lambda: False
    assert pub.tick(0.0)
    assert set(json.loads(sent[0])) == {"frame_id", "allowed", "keepout", "walls"}
    import houndkeeper.operating_area_module as m, houndkeeper.tidy_module as t
    from dimos.core.stream import Out
    outs = {k for k, v in m.OperatingArea.__annotations__.items() if "Out" in str(v)}
    assert outs == {"keepout_zones"}                                   # the module publishes the boundary alone
    assert "Out[PointCloud2]" not in t.TidyArm.__annotations__.get("keepout_zones", "")


def test_publisher_follows_the_file_and_the_correction(tmp_path):
    sent = []; C = [np.eye(3)]
    pub = AreaPublisher(tmp_path, sent.append, lambda: C[0])
    assert not pub.tick(0.0) and not sent                               # no area: nothing published
    oa.save(tmp_path, TWO_ROOMS, by="t")
    assert pub.tick(1.0) and len(sent) == 1
    assert not pub.tick(2.0)                                            # nothing changed
    C[0] = np.array([[1, 0, 0.05], [0, 1, 0], [0, 0, 1.0]])
    assert pub.tick(3.0) and len(sent) == 2                             # the correction moved it 5 cm
    assert pub.tick(3.0 + AreaPublisher.REPEAT_S) and len(sent) == 3   # again for a late subscriber
    import os, time
    (tmp_path / oa.FILE).write_text("{broken"); os.utime(tmp_path / oa.FILE, ns=(time.time_ns() + 10**9,) * 2)
    pub.tick(20.0)
    assert pub.area is not None and "not read" in pub.error             # the last good area is kept


def test_nearest_way_back_in_and_inside_with_an_inset():
    a = oa.validate(TWO_ROOMS)
    assert oa.inside(a, (1, 1)) and oa.inside(a, (6, 1)) and not oa.inside(a, (6, 2.5))
    assert not oa.inside(a, (0.1, 1.5), inset=0.35) and oa.inside(a, (0.5, 1.5), inset=0.35)
    tgt = oa.nearest_inside(a, (6.0, 3.0))
    assert oa.inside(a, tgt, inset=0.35) and math.dist(tgt, (6.0, 3.0)) < 1.45   # straight down into the second room
    ins, out = oa.keep_inside(a, [(1, 1), (8, 8)])
    assert ins == [(1, 1)] and out == [(8, 8)] and oa.keep_inside(None, [(8, 8)]) == ([(8, 8)], [])


# --- the tidy ([OrO], [Qd5]) ---
tidy = pytest.importorskip("houndkeeper.tidy_module")


def _tidy(area=TWO_ROOMS, pose=(1.0, 1.0), sim=True):
    r = object.__new__(tidy.TidyArm)
    r._area_pub = SimpleNamespace(area=None if area is None else oa.validate(area))
    r._status = {"state": "idle", "log": [], "items": []}; r._cf = None
    r.config = SimpleNamespace(g=SimpleNamespace(simulation="mujoco" if sim else "", replay=False))
    P = {"xy": np.array(pose, float)}; r._P = P
    r._pose = lambda: (np.array([*P["xy"], 0.3]), np.array([1.0, 0, 0, 0]))
    r._yaw = lambda: 0.0; r._publish = lambda note="": None; r._stop = lambda: None
    r._odom_t = None; r.goals = []
    def go(x, y, yaw, timeout=90.0, guard=True):
        r.goals.append((x, y)); P["xy"] = np.array([x, y]); return "reached"
    r._go = go
    r._navigation = SimpleNamespace(cancel_goal=lambda: None)
    return r


def test_the_real_robot_refuses_to_start_without_an_area():
    r = _tidy(area=None, sim=False)
    msg = getattr(tidy.TidyArm.tidy_start, "__wrapped__", tidy.TidyArm.tidy_start)(r)
    assert msg.startswith("refused") and r._status["state"] == "idle"


def test_the_sim_runs_without_an_area_and_the_robot_with_one(monkeypatch):
    monkeypatch.setattr(tidy.threading, "Thread", lambda **kw: SimpleNamespace(start=lambda: None))
    start = getattr(tidy.TidyArm.tidy_start, "__wrapped__", tidy.TidyArm.tidy_start)
    assert start(_tidy(area=None, sim=True)) == "tidy started"
    assert start(_tidy(area=TWO_ROOMS, sim=False)) == "tidy started"


def test_outside_he_goes_back_in_first_by_the_shortest_way():
    r = _tidy(pose=(6.0, 3.0))
    assert r._area_check() is True
    assert len(r.goals) == 1 and oa.inside(r._area(), r.goals[0], inset=0.3) and math.dist(r.goals[0], (6.0, 3.0)) < 1.45
    assert r._status["returns"] and not r._area_check()                 # inside now: nothing more to do


def test_outside_with_no_way_back_he_stops():
    r = _tidy(pose=(6.0, 3.0)); r._go = lambda *a, **k: r.goals.append(a) or "gave up"   # pushed against something
    with pytest.raises(tidy.AreaStop, match="could not get back"): r._area_check()
    assert len(r.goals) == r.RETURN_TRIES


def _clock(monkeypatch, t0):
    T = {"t": t0}
    monkeypatch.setattr(tidy.clock, "now", lambda: T["t"])
    monkeypatch.setattr(tidy.clock, "sleep", lambda s: T.__setitem__("t", T["t"] + s))
    return T


def test_localization_lost_stops_and_reports(monkeypatch):
    """no pose: he stands still at once and says so; still none after LOST_WAIT_S, the run stops with the reason"""
    r = _tidy(); r._odom_t = 100.0; said = []; r._say = said.append; stops = []; r._stop = lambda: stops.append(1)
    _clock(monkeypatch, 100.0 + r.ODOM_LOST_S + 0.5)
    with pytest.raises(tidy.AreaStop, match="localization lost"): r._area_check()
    assert stops and "standing still" in said[0]
    r._odom_t = 200.0; T = _clock(monkeypatch, 200.0 + r.ODOM_LOST_S + 0.5)
    r._tidy_loop = lambda: r._area_check(); said.clear()
    tidy.TidyArm._run(r)
    assert r._status["state"] == "stopped" and "localization lost" in r._status["stopped"] and said


def test_a_pose_gap_that_ends_is_waited_out(monkeypatch):
    """a short gap in the pose (the tidy saw 2.6 s in a sim night the recording did not): stand still, then go on"""
    r = _tidy(); r._odom_t = 100.0; said = []; r._say = said.append
    T = _clock(monkeypatch, 100.0 + r.ODOM_LOST_S + 0.5)
    def sleep(s):
        T["t"] += s
        if T["t"] > 105.0: r._odom_t = T["t"]                   # the pose comes back
    monkeypatch.setattr(tidy.clock, "sleep", sleep)
    assert r._area_check(walking=True) is True                  # the caller sets its goal again
    assert r._status["pose_gaps"] and said[-1] == "pose back: going on"


def test_finds_outside_are_noted_not_pursued():
    """[OrO]: a confirmed find outside the area is logged as seen, never returned for a stop"""
    r = _tidy(); r._sweep_log = []; r._sidestepped = {}
    recs = [{"x": 6.0, "y": 2.6, "decision": "confirmed", "kind": "present", "hits": 30, "area_cm2": 300, "range_m": 1.0},
            {"x": 1.0, "y": 1.0, "decision": "confirmed", "kind": "present", "hits": 30, "area_cm2": 300, "range_m": 1.0}]
    r._cf = SimpleNamespace(decide=lambda t, p: [dict(x) for x in recs])
    rec = r._decisions()
    assert (rec["x"], rec["y"]) == (1.0, 1.0)
    assert r._sweep_log[0].get("outside_area") and "not visited" in r._sweep_log[0]["why"]
    r._cf = SimpleNamespace(keepout=lambda: [(6.0, 2.6, 0.1), (1.2, 1.0, 0.1)])
    r._binned, r._cleared, r._ctx = [], [], {"handled": []}
    assert r._blockers(reach=9.0) == [(1.2, 1.0)]                      # nor walked out to, to unblock


def test_the_route_and_the_flags_keep_inside():
    r = _tidy(); said = []; r._say = said.append
    ins, out = r._split_inside([np.array([1.0, 1.0]), np.array([6.0, 2.8]), np.array([5.0, 1.0])])
    assert len(ins) == 2 and len(out) == 1
    r._note_outside(out[0], "a find")
    assert r._status["outside_area"][0]["what"] == "a find" and "not visited" in said[0]


def test_sweep_route_plans_inside_the_area_only(tmp_path):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "house"))
    import sweep_route as sr
    xs, ys = np.meshgrid(np.arange(0.025, 8, 0.05), np.arange(0.025, 4, 0.05))
    floor = np.c_[xs.ravel(), ys.ravel()]
    m = sr.Map.from_points([floor], [np.zeros((0, 2))])
    full = sr.plan(m, (1.0, 1.0, 0.0))
    area = oa.validate({"allowed": [[[0, 0], [4, 0], [4, 4], [0, 4]]]})
    m2 = sr.limit_to_area(sr.Map.from_points([floor], [np.zeros((0, 2))]), area)
    pts = sr.plan(m2, (1.0, 1.0, 0.0))
    assert any(p[0] > 4.5 for p in full)                               # control: without the area it goes there
    assert pts and all(oa.inside(area, p, inset=0.3) for p in pts)


def test_the_morning_log_notes_what_was_seen_outside(tmp_path, monkeypatch):
    """[OrO]: finds outside the area are in the morning log as seen, not visited; [Qd5]: a return and a stop are said"""
    import test_morning_log as tml
    from houndkeeper import morning_log as ml
    d = tml.night(tmp_path, monkeypatch=monkeypatch)
    rep = json.loads((d / "report.json").read_text())
    rep["sweep"] = [{"x": 6.0, "y": 2.6, "decision": "confirmed", "outside_area": True},
                    {"x": 6.01, "y": 2.61, "decision": "confirmed", "outside_area": True},     # the same spot again
                    {"x": 9.0, "y": 2.6, "decision": "unsure", "outside_area": True}]          # not confirmed: not counted
    rep["returns"] = [{"from": [6, 3], "to": [6, 1.6]}]; rep["stopped"] = "localization lost: no pose for 2.5 s"
    (d / "report.json").write_text(json.dumps(rep))
    H = ml.household(ml.Night(d)); text = " ".join(ml.prose_text(H))
    assert H["facts"]["outside_seen"]["value"] == 1 and "beyond the bounds you set, and, as instructed, left them be" in text
    assert "went straight back in" in text and "lost my bearings" in text


# --- the Area page ([daF], [HZO]) ---

def _feed(md):
    from houndkeeper.operating_area_module import AreaFeed
    f = object.__new__(AreaFeed); f._md = Path(md); f._message = ""; f._map = None; f._map_stamp = None
    f.area_state = SimpleNamespace(publish=lambda m: None)
    return f


def _house(md):
    xs, ys = np.meshgrid(np.arange(-3, 8, 0.1), np.arange(-2, 3, 0.1))
    floor = np.c_[xs.ravel(), ys.ravel(), np.zeros(xs.size)]
    wall = np.c_[np.full(50, -3.0), np.linspace(-2, 3, 50), np.full(50, 1.0)]
    np.save(Path(md) / "house_lidar.npy", np.vstack([floor, wall]).astype(np.float32))


def test_the_page_gets_the_house_map_the_area_its_changes_and_the_warning(tmp_path):
    _house(tmp_path); oa.save(tmp_path, TWO_ROOMS, by="t")
    st = _feed(tmp_path).state()
    m = st["map"]
    assert m["res"] == 0.05 and m["x0"] < -3 and m["y0"] < -2 and m["w"] * m["res"] > 11 and len(m["png"]) < 64_000
    assert st["area"]["allowed"] == oa.load(tmp_path).allowed and st["history"][0]["polygons"] == 2
    assert st["warning"] == oa.WARNING and "not a safety barrier" in st["warning"] and "gate" in st["warning"]
    assert len(json.dumps(st)) < 256 * 1024
    assert _feed(tmp_path / "nothing").state()["map"] is None
    np.save(tmp_path / "house_lidar.npy", np.random.default_rng(0).uniform(-3, 3, (500, 2)))   # a 2D map: walls only
    f = _feed(tmp_path); m2 = f.state()["map"]
    assert m2 is not None and m2["w"] > 100 and m2["png"] != m["png"]


def test_a_save_needs_the_confirm_and_the_change_it_replaced(tmp_path):
    f = _feed(tmp_path)
    f.handle({"op": "save", "area": TWO_ROOMS})                               # not confirmed
    assert "not confirmed" in f._message and oa.load(tmp_path) is None
    f.handle({"op": "save", "confirmed": True, "seen": "", "area": TWO_ROOMS})
    a = oa.load(tmp_path); assert a.by == "the phone cockpit" and f._message.startswith("saved ")
    f.handle({"op": "save", "confirmed": True, "seen": "", "area": {"allowed": [[[0, 0], [3, 0], [3, 3]]]}})   # a stale tab
    assert "changed since" in f._message and len(oa.load(tmp_path).allowed) == 2
    f.handle({"op": "save", "confirmed": True, "seen": a.changed, "area": {"allowed": [[[0, 0], [0.1, 0], [0, 0.1]]]}})
    assert "0.25" in f._message and len(oa.load(tmp_path).allowed) == 2       # refused, nothing written
    (tmp_path / "dock.json").write_text(json.dumps({"x": 9.0, "y": 9.0}))
    f.handle({"op": "save", "confirmed": True, "seen": a.changed, "area": TWO_ROOMS})
    assert "dock" in f._message
    assert len((tmp_path / oa.LOG).read_text().splitlines()) == 1             # only the one save was recorded


def test_the_docs_carry_the_warning():
    """[HZO]: where the area is drawn (the page, above) and in the docs"""
    HK = Path(__file__).resolve().parents[1]
    for doc in ("docs/specs/operating-area.md", "real/README.md"):
        assert "not a safety barrier" in (HK / doc).read_text().lower(), doc


def test_the_area_page_at_phone_width(tmp_path):
    """[daF] [HZO]: a loopback relay and the Area page's service alone (houndkeeper.area, no robot or sim), then
    tests/browser/area_390.mjs at 390 px, on a private zenoh discovery bus so a running stack does not collide: the warning shown, the layout audit, an area drawn by taps, confirmed, stored
    with the map and logged. Screenshots: HK_SHOTS_DIR, else this test's tmp dir"""
    import os, shutil, signal, subprocess, time
    from houndkeeper import project_stack as ps
    deno = shutil.which("deno") or str(Path.home() / ".local" / "bin" / "deno"); HK = Path(__file__).resolve().parents[1]
    need = [Path(deno), ps.DIMOS / "web" / "cockpit" / "dist" / "index.html", HK / "real" / "tests" / "node_modules" / "playwright",
            Path(sys.executable).with_name("dimos")]
    missing = [str(p) for p in need if not p.exists()] + ([] if shutil.which("node") else ["node"])
    if missing: pytest.skip("the browser smoke needs " + ", ".join(missing))
    md = tmp_path / "map"; md.mkdir(); _house(md)
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
        url = f"http://127.0.0.1:{port}"; log = open(tmp_path / "area.log", "w")
        svc = subprocess.Popen([str(Path(sys.executable).with_name("dimos")), "--robot-id", "area", "--relay-url", url, "--viewer", "none",
                                "run", "houndkeeper.area"], env={**os.environ, "HK_MAP_DIR": str(md), "ZENOH_SCOUT_ADDR": "224.0.0.231:7446"}, stdout=log,
                               stderr=subprocess.STDOUT, cwd=HK, start_new_session=True)
        t0 = time.time()
        while time.time() - t0 < 120 and "relay bridge up" not in (tmp_path / "area.log").read_text():
            assert svc.poll() is None, (tmp_path / "area.log").read_text()[-2000:]
            time.sleep(1)
        r = subprocess.run(["node", str(HK / "tests" / "browser" / "area_390.mjs"), url + "/", str(shots)],
                           capture_output=True, text=True, timeout=240)
        out = json.loads(r.stdout.strip().splitlines()[-1])
        assert out["ok"], out
        assert "not a safety barrier" in out["warning"] and any("not a safety barrier" in d for d in out["dialogs"])
        a = oa.load(md)
        assert a is not None and len(a.allowed) == 1 and len(a.allowed[0]) == 4 and a.by == "the phone cockpit"
        assert len((md / oa.LOG).read_text().splitlines()) == 1 and len(out["shots"]) == 3
    finally:
        for p in (svc, relay):
            if p is not None and p.poll() is None:
                os.killpg(p.pid, signal.SIGTERM)
                try: p.wait(20)
                except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL)
