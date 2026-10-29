"""[HuM], [jIh], [pI7]: the sweep route is planned on the robot's own map, from its dock as it saw it, and goes only where the
robot can walk: floor the dock reaches, clear of what the map holds, a last pass along the far wall (2 Oct 2026: the
demo room's route sent 13 waypoints beyond its walls, and the robot stood ~1 min through the failures)."""
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]; SIM = HK / "sim" / "house"
sys.path.insert(0, str(SIM))


def _room():
    """a mapped 8 x 5 m room (floor seen everywhere inside its walls) with a 1.5 x 1 m table in it, and beside it a
    second room behind a solid wall, also seen (through a window), that the dock cannot reach"""
    import sweep_route as sr
    xs, ys = np.arange(0, 12, sr.RES) + sr.RES / 2, np.arange(0, 5, sr.RES) + sr.RES / 2
    X, Y = np.meshgrid(xs, ys); P = np.c_[X.ravel(), Y.ravel()]
    wall = (P[:, 0] < 0.1) | (P[:, 0] > 11.9) | (P[:, 1] < 0.1) | (P[:, 1] > 4.9) | (abs(P[:, 0] - 8.0) < 0.1)
    table = (abs(P[:, 0] - 4.0) < 0.75) & (abs(P[:, 1] - 2.5) < 0.5)
    return sr.Map.from_points([P[~wall & ~table]], [P[wall | table]]), (1.0, 1.0, 0.0)


def test_the_route_stays_on_floor_the_dock_reaches():
    import sweep_route as sr
    m, dock = _room(); pts = sr.plan(m, dock)
    assert pts and all(0.1 < x < 7.9 and 0.1 < y < 4.9 for x, y in pts)        # never in the room behind the wall
    assert not any(abs(x - 4.0) < 0.75 and abs(y - 2.5) < 0.5 for x, y in pts)  # never in the table


def test_waypoints_stand_clear_of_what_the_map_holds():
    import sweep_route as sr
    m, dock = _room(); clr = m.clearance()
    for x, y in sr.plan(m, dock, clear_m=0.35): assert clr[m.cell(x, y)] >= 0.6


def test_rows_start_at_the_dock_and_end_along_the_far_wall():
    import sweep_route as sr
    m, dock = _room(); ys = [y for _, y in sr.plan(m, dock)]
    assert abs(ys[0] - 0.8) < 0.15 and max(ys) > 4.9 - 0.75 - 0.525               # 0.7 m clear of each wall; the far
                                                                                 # pass within half a spacing of it
    assert len({round(y, 1) for y in ys}) == 4                                   # 1.05 m apart over 3.4 m: 4 passes


def test_the_route_reads_no_house_plan():
    """[1B2]: the route module never imports the generator"""
    import ast
    tree = ast.parse((SIM / "sweep_route.py").read_text())
    names = {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    names |= {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    assert not names & {"make_house", "make_nights"}


def test_a_first_night_without_a_map_plans_nothing(tmp_path):
    (tmp_path / "recs.txt").write_text("")
    r = subprocess.run([sys.executable, str(SIM / "sweep_route.py"), "--recordings", str(tmp_path / "recs.txt")],
                       capture_output=True, text=True)
    assert r.returncode == 3 and "explore first" in r.stderr


RUN = SIM / "runs" / "nights_s72"


@pytest.mark.skipif(not (RUN / "floor_frame.json").exists(), reason="house 72's recorded baselines are not on this machine")
def test_a_recorded_house_route_keeps_off_its_true_walls_and_furniture():
    """the route from house 72's two recorded baseline nights, scored on the generator's plan (the test may read it)"""
    import sweep_compare as sc
    start = json.loads((RUN / "nights_s72_n1.truth.json").read_text())["start_pos"]
    pts, _m, dock = sc.map_route(72); g, reach, F = sc.truth_floor(72)
    s = sc.score(72, pts, g, reach, F, np.array(start))
    assert math.dist(dock[:2], start) < 0.1 and s["in_obstacle"] == 0 and s["unreachable"] == 0 and s["cover_1.0"] > 0.6


def _hall(line_nights=(), nights=2):
    """an 8 x 5 m room seen on `nights` nights, the dock at its east end, and a thin line (one 5 cm cell wide) across its
    whole width at x = 3 m on the tall layers of the nights in line_nights (0-based): night 2's ridge across house 72's bay"""
    import sweep_route as sr
    xs, ys = np.arange(0, 8, sr.RES) + sr.RES / 2, np.arange(0, 5, sr.RES) + sr.RES / 2
    X, Y = np.meshgrid(xs, ys); P = np.c_[X.ravel(), Y.ravel()]
    wall = (P[:, 0] < 0.1) | (P[:, 0] > 7.9) | (P[:, 1] < 0.1) | (P[:, 1] > 4.9)
    line = P[(abs(P[:, 0] - 3.025) < 0.02) & ~wall]
    talls = [line if k in line_nights else np.zeros((0, 2)) for k in range(nights)]
    return [P[~wall]] * nights, [P[wall]] * nights, talls, (7.0, 2.5, 0.0)


def test_a_one_night_line_across_the_dock_s_bay_does_not_cut_it_off():
    """[pI7]: a tall-layer line on one of two nights leaves the floor beyond it on the route"""
    import sweep_route as sr
    fl, st, tl, dock = _hall(line_nights=(1,))
    pts = sr.plan(sr.Map.from_points(fl, st, talls=tl), dock)
    assert pts and min(x for x, _ in pts) < 2.0


def test_a_mark_on_both_nights_still_blocks():
    """[pI7]: the same line on both nights is something that stands there: the route stays on the dock's side"""
    import sweep_route as sr
    fl, st, tl, dock = _hall(line_nights=(0, 1))
    pts = sr.plan(sr.Map.from_points(fl, st, talls=tl), dock)
    assert pts and min(x for x, _ in pts) > 3.0


def test_a_one_night_map_keeps_its_tall_marks():
    """[pI7]: with one night mapped, its one tall layer counts"""
    import sweep_route as sr
    fl, st, tl, dock = _hall(line_nights=(0,), nights=1)
    pts = sr.plan(sr.Map.from_points(fl, st, talls=tl), dock)
    assert pts and min(x for x, _ in pts) > 3.0


def test_nothing_planned_says_why_and_exits_non_zero(tmp_path, monkeypatch, capsys):
    """[pI7]: a dock walled into a pocket too tight for the clearance asked: the reason on stderr, exit 5, not an empty
    route with exit 0 (house 72, 13 Oct 2026)"""
    import sweep_route as sr
    xs, ys = np.arange(0, 8, sr.RES) + sr.RES / 2, np.arange(0, 5, sr.RES) + sr.RES / 2
    X, Y = np.meshgrid(xs, ys); P = np.c_[X.ravel(), Y.ravel()]
    wall = (P[:, 0] < 0.1) | (P[:, 0] > 7.9) | (P[:, 1] < 0.1) | (P[:, 1] > 4.9) | ((abs(P[:, 0] - 6.6) < 0.05) & (P[:, 1] < 1.4))
    wall |= (abs(P[:, 1] - 1.4) < 0.05) & (P[:, 0] > 6.6)                           # a 1.3 x 1.3 m closet round the dock
    for n in (1, 2): (tmp_path / f"n{n}.db").write_text("")
    (tmp_path / "recs.txt").write_text(f"1 {tmp_path / 'n1.db'}\n2 {tmp_path / 'n2.db'}\n")
    monkeypatch.setattr(sr, "night_map", lambda db, corrections=None: (P[~wall], P[wall], (7.25, 0.75, 0.0)))
    monkeypatch.setattr(sys, "argv", ["sweep_route.py", "--recordings", str(tmp_path / "recs.txt"), "--night", "3",
                                      "--near", "dock", "--radius", "5", "--clear", "2"])
    with pytest.raises(SystemExit) as e: sr.main()
    err = capsys.readouterr().err
    assert e.value.code == 5 and "no waypoints" in err and "0.70 m asked" in err and "pocket" in err
