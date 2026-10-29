"""[o0E], [WD9], [JQk], [tWE]: the detection baselines' one placement and one scorer, on synthetic cameras and floors
(sim/house/baselines): a box's bottom-centre lands where the camera model says it should, and back; points of one
thing across frames merge into one candidate; a find is the tidy's own rule; false positives are counted; tiers, floor
kinds and unknown kinds are sorted as fixed; the frontier's answers load into the same detections."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))

from baselines import frontier as fr, place as pl, prompts as pr, score as sc   # noqa: E402

K = [[400.0, 0, 320.0], [0, 400.0, 180.0], [0, 0, 1]]
D = [-0.073, -0.023, -0.0069, 0.0092]          # the head camera's own fisheye coefficients (front_camera_720.yaml)


def camera(x=0.0, y=0.0, z=0.30, yaw=0.0, pitch_deg=10.0):
    """world <- optical for a camera at (x, y, z) facing yaw (rad) along the floor, tipped pitch_deg down"""
    p = math.radians(pitch_deg)
    fwd = np.array([math.cos(yaw) * math.cos(p), math.sin(yaw) * math.cos(p), -math.sin(p)])
    right = np.array([math.sin(yaw), -math.cos(yaw), 0.0]); down = np.cross(fwd, right)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = right, down, fwd, [x, y, z]; return T


@pytest.mark.parametrize("dist", [None, D])
def test_floor_point_projects_back_to_its_pixel(dist):
    T = camera(1.0, 2.0, yaw=0.7)
    for xy in [(1.0 + 1.5 * math.cos(0.7), 2.0 + 1.5 * math.sin(0.7)), (2.2, 2.9), (1.8, 3.4)]:
        uv = pl.project(xy, T, K, dist)
        x, y, r = pl.to_floor(uv[0], uv[1], T, K, dist)
        assert math.hypot(x - xy[0], y - xy[1]) < 1e-3
        assert abs(r - math.hypot(xy[0] - 1.0, xy[1] - 2.0)) < 1e-3


def test_principal_point_lands_straight_ahead_at_the_pitch_range():
    T = camera(pitch_deg=10.0); x, y, r = pl.to_floor(320.0, 180.0, T, K)
    assert abs(x - 0.30 / math.tan(math.radians(10))) < 1e-6 and abs(y) < 1e-9


def test_pixels_at_or_above_the_horizon_and_beyond_reach_are_not_placed():
    T = camera(pitch_deg=0.0)
    assert pl.to_floor(320.0, 180.0, T, K) is None          # level: the centre ray never meets the floor
    assert pl.to_floor(320.0, 100.0, T, K) is None          # above the horizon
    assert pl.to_floor(320.0, 200.0, T, K, rmax=2.5) is None   # 20 px below it: 6 m out
    assert pl.to_floor(320.0, 300.0, T, K, rmax=2.5) is not None


def test_a_box_stands_on_its_bottom_centre():
    T = camera(); f = {"id": "n/1", "ts": 5.0, "T_world_cam": T.tolist(), "K": K, "D": None}
    uv = pl.project((1.2, 0.1), T, K)
    box = [uv[0] - 20, uv[1] - 50, uv[0] + 20, uv[1]]
    p = pl.place({"label": "toy", "box": box, "score": 0.4}, f)
    assert math.hypot(p["x"] - 1.2, p["y"] - 0.1) < 1e-3 and p["ts"] == 5.0 and p["label"] == "toy" and p["frame"] == "n/1"


def test_odometry_to_sim_frame_and_back(tmp_path):
    t = np.linspace(0, 10, 11); rows = ["wall_t,sim_t,true_x,true_y,true_yaw,odom_x,odom_y,odom_yaw,host_t"]
    for ti in t:   # odometry drifted: 0.2 m in x and 0.1 rad of heading by the end
        rows.append(f"{ti},{ti},{ti * 0.1},{0.0},{0.0},{ti * 0.1 + 0.02 * ti},{0.0},{0.01 * ti},{ti}")
    p = tmp_path / "traj.csv"; p.write_text("\n".join(rows) + "\n")
    f = pl.odom_to_true(str(p))
    x, y = f(1.2, 0.0, 10.0)                      # the robot's own spot at t=10 is where it truly stood
    assert abs(x - 1.0) < 1e-9 and abs(y) < 1e-9
    xo, yo = f.inverse(*f(2.0, 0.5, 7.0), 7.0)
    assert abs(xo - 2.0) < 1e-9 and abs(yo - 0.5) < 1e-9


def test_one_thing_seen_in_several_frames_is_one_candidate():
    pts = [{"x": 1.0 + dx, "y": 2.0, "score": s, "frame": f"f{i}"} for i, (dx, s) in enumerate([(0, 0.5), (0.1, 0.3), (-0.12, 0.6)])]
    pts.append({"x": 3.0, "y": 2.0, "score": 0.2, "frame": "f9"})
    c = sc.merge(pts)
    assert len(c) == 2 and sorted(x["frames"] for x in c) == [1, 3]
    big = next(x for x in c if x["frames"] == 3)
    assert abs(big["x"] - 1.0) < 0.05


def toy(x, y, **kw):
    return dict({"id": "t", "name": "brick", "x": x, "y": y, "reach": sc.reach(0.05), "tier": "medium", "floor": "hard", "named": True}, **kw)


def test_a_find_is_within_the_tidys_reach_and_the_rest_are_false_positives():
    toys = [toy(0, 0, id="a"), toy(5, 0, id="b"), toy(10, 0, id="c", reach=sc.reach(0.20))]
    cands = [{"x": 0.29, "y": 0, "score": 1}, {"x": 5.31, "y": 0, "score": 1}, {"x": 10.34, "y": 0, "score": 1},
             {"x": 20, "y": 0, "score": 1}, {"x": 30, "y": 0.05, "score": 1}]
    r = sc.score_night(cands, toys, stains=[{"x": 30, "y": 0, "r": 0.1}])
    assert [t["found"] for t in r["toys"]] == [True, False, True]   # 29 cm in; 31 cm out; a long toy's 35 cm reach
    assert r["found"] == 2 and r["fp"] == 3 and r["fp_on_stain"] == 1 and r["fp_near_toy"] == 1   # 5.31: just past b


def test_reach_is_score_tidys():
    assert sc.reach(0.05) == sc.NEAR_M and abs(sc.reach(0.25) - 0.40) < 1e-9


def test_tiers_and_floor_kinds():
    assert [sc.tier_of(a) for a in (987.7, 150.0, 149.9, 50.0, 31.9)] == ["easy", "easy", "medium", "medium", "hard"]
    rug = {"x": 0.0, "y": 0.0, "w": 2.0, "h": 1.0, "yaw_deg": 90.0}
    assert sc.floor_kind(0.0, 0.9, [], rug) == "rug"            # turned: its long side along y
    assert sc.floor_kind(0.9, 0.0, [], rug) == "hard"
    assert sc.floor_kind(3.0, 3.0, [(2.0, 4.0, 2.0, 4.0)], rug) == "carpet"


def test_tally_by_key_and_unknown_kinds():
    nights = [{"toys": [toy(0, 0, found=True, named=True), toy(1, 0, found=False, named=False, tier="hard")]},
              {"toys": [toy(0, 0, found=True, named=False, tier="hard")]}]
    assert sc.tally(nights, "named") == {True: [1, 1], False: [1, 2]}
    assert sc.tally(nights, "tier") == {"medium": [1, 1], "hard": [1, 2]}
    assert pr.named("lego_1x1") and not pr.named("triceratops")
    assert not (set(pr.KINDS.values()) - set(pr.PROMPTS)) and not (pr.GENERIC & set(pr.KINDS.values()))


def test_frontier_batches_and_answers_load(tmp_path):
    import cv2
    frames = []
    for i in range(30):
        png = tmp_path / f"{i}.png"; cv2.imwrite(str(png), np.full((360, 1280, 3), 128, np.uint8))
        frames.append({"id": f"n{i % 3}/{i}", "night": f"n{i % 3}", "seed": 70, "ts": float(i), "png": str(png), "width": 1280,
                       "height": 360, "K": K, "D": None, "T_world_cam": camera().tolist()})
    man = tmp_path / "manifest.json"; man.write_text(json.dumps({"frames": frames}))
    strata = {f["id"]: (["hard", "easy", "empty"][i % 3 if i < 24 else 0], [], []) for i, f in enumerate(frames)}
    ctx = {f"n{k}": {"toys": [], "to_true": None} for k in range(3)}
    import unittest.mock as um
    with um.patch.object(fr, "stratum", lambda f, toys, tt: strata[f["id"]]), um.patch.object(fr, "SUBSET", 12), \
            um.patch.object(fr, "PILOT", 5):
        chosen = fr.choose(frames, strata, n=12)
        assert len(chosen) == 12 and {f["night"] for f in chosen} == {"n0", "n1", "n2"}
        assert sum(strata[f["id"]][0] == "hard" for f in chosen) == round(12 * fr.SHARES["hard"])
        files = fr.prepare(str(man), str(tmp_path / "fx"), ctx=ctx)
    S = json.loads((tmp_path / "fx" / "frontier_set.json").read_text())
    assert len(S["frames"]) == 12 and {e["id"] for e in S["frames"]} <= {f["id"] for f in frames}   # a subset of the set
    pilot = (tmp_path / "fx" / "batch_00.txt").read_text().splitlines()
    assert len(pilot) == 5 and all(p.endswith(".jpg") for p in pilot) and len(files) == 2   # the pilot, then the other 7
    assert {e["night"] for e in S["frames"] if e["pilot"]} == {"n0", "n1", "n2"}
    e = S["frames"][0]
    assert abs(e["scale"] - 768 / 1280) < 1e-9 and e["sent_size"] == [768, 216]   # the long side to 768 px
    assert "768 x 216" in (tmp_path / "fx" / "prompt.txt").read_text()
    ans = {"model": "claude-opus-5-5", "frames": {e["jpg"]: [{"label": "toy", "box": [300, 150, 340, 130]}],
                                                    Path(S["frames"][1]["jpg"]).name: "```json\n[{\"label\": \"shoe\", \"box\": [1, 2, 3, 4]}, {\"label\": \"x\"}]\n```",
                                                    "nope.jpg": []}}
    (tmp_path / "fx" / "answers" / "batch_00.json").write_text(json.dumps(ans))
    r = fr.load(str(tmp_path / "fx"), str(tmp_path / "frontier.json"))
    assert len(r["frames"]) == 2 and r["missing_frames"] == len(S["frames"]) - 2 and r["malformed"] == 2
    d0 = r["frames"][0]["detections"][0]
    assert d0["box"] == pytest.approx([500.0, 216.6667, 566.6667, 250.0], abs=1e-3) and d0["score"] == 1.0   # full-frame pixels
    assert r["call_log"][0]["model"] == "claude-opus-5-5" and r["frames"][0]["frame"]["png"] == e["png"]


def test_even_pick():
    from baselines import frames as fm
    assert fm.even_pick(5, 45) == [0, 1, 2, 3, 4]
    p = fm.even_pick(1729, 45)
    assert len(p) == 45 and p[0] == 0 and p[-1] == 1728


def test_a_toy_the_walk_bumped_is_found_where_it_was_dropped_or_where_it_ended():
    t = toy(0, 0, also=[[0.9, 0.0]])
    assert sc.score_night([{"x": 0.85, "y": 0, "score": 1}], [t])["found"] == 1
    assert sc.score_night([{"x": 0.1, "y": 0, "score": 1}], [t])["found"] == 1
    assert sc.score_night([{"x": 0.45, "y": 0, "score": 1}], [t])["fp"] == 1
