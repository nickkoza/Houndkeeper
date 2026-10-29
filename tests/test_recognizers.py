"""[3hK], [En1], [o0E]: the loosened floor comparison and the local recognizers behind it (sim/house/baselines), on
synthetic cameras and verdicts, no model loaded: the sensitivities only loosen and keep floor-sized changes out; a
candidate's crops come from frames that see its floor point, nearest first and apart in time, the point inside the crop
where the recognizer is asked about it; a detection counts for a crop only if it holds that point; a candidate is kept
when its best verdict reaches the threshold (and kept when no frame saw it); labels naming the house are dropped."""
import math
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))

from baselines import floor_arm as fa, place as pl, recheck as rk, recognizers as rc   # noqa: E402

K = [[400.0, 0, 320.0], [0, 400.0, 180.0], [0, 0, 1]]
D = [-0.073, -0.023, -0.0069, 0.0092]


def camera(x=0.0, y=0.0, z=0.30, yaw=0.0, pitch_deg=10.0):
    p = math.radians(pitch_deg)
    fwd = np.array([math.cos(yaw) * math.cos(p), math.sin(yaw) * math.cos(p), -math.sin(p)])
    right = np.array([math.sin(yaw), -math.cos(yaw), 0.0]); down = np.cross(fwd, right)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = right, down, fwd, [x, y, z]; return T


def test_sensitivities_only_loosen_and_today_is_the_live_tidys():
    from confirm_stream import CONFIRM
    assert fa.SENSITIVITIES["today"] == ({}, ("confirmed", "unsure"))
    order = ["today", "loose1", "loose2", "loose3"]
    get = lambda s, k: fa.SENSITIVITIES[s][0].get(k, CONFIRM[k.replace("-", "_")])
    for a, b in zip(order, order[1:]):
        for k in ("hits", "ratio", "span", "tc", "tb"):
            assert get(b, k) <= get(a, k), (a, b, k)
        assert get(b, "unsure-at") >= get(a, "unsure-at")
        assert set(fa.SENSITIVITIES[a][1]) <= set(fa.SENSITIVITIES[b][1])


def test_candidates_keep_the_chosen_decisions_and_never_a_floor_sized_change():
    ds = [{"x_robot": 1, "y_robot": 2, "ts": 3.0, "decision": "confirmed", "kind": "present", "hits": 5},
          {"x_robot": 1, "y_robot": 2, "ts": 3.0, "decision": "unsure", "hits": 2},
          {"x_robot": 1, "y_robot": 2, "ts": 3.0, "decision": "not_a_find", "kind": "present", "hits": 4},
          {"x_robot": 1, "y_robot": 2, "ts": 3.0, "decision": "not_a_find", "kind": "floor_change", "hits": 9}]
    assert [p["label"] for p in fa.candidates(ds)] == ["confirmed", "unsure"]
    loose = fa.candidates(ds, ("confirmed", "unsure", "not_a_find"))
    assert [p["label"] for p in loose] == ["confirmed", "unsure", "not_a_find"]
    assert [p["score"] for p in loose] == [1.0, 0.5, 0.25]


def test_a_sensitivity_reaches_the_replays_command_line():
    n = {"seed": 70, "db": "x.db", "traj": "t.csv"}
    cmd = fa.confirmer_cmd(n, Path("/nonexistent"), Path("o.json"), "loose3")
    assert cmd[cmd.index("--hits") + 1] == "2" and cmd[cmd.index("--slop") + 1] == "0"
    assert "--hits" not in fa.confirmer_cmd(n, Path("/nonexistent"), Path("o.json"), "today")


def test_views_are_the_nearest_frames_that_see_the_point_apart_in_time():
    xy = (2.0, 0.0)
    poses = [(k, 0.25 * k, camera(x=0.05 * k)) for k in range(40)]     # walking straight at it, 4 frames a second
    (vs,) = rk.pick_views([xy], poses, 640, 360, K, D, "equidistant")
    assert 1 <= len(vs) <= rk.VIEWS
    rs = [r for _, _, _, r in vs]
    assert rs == sorted(rs) and rk.RANGE_M[0] <= rs[0] and rs[-1] <= rk.RANGE_M[1]
    ts = [poses[i][1] for i, _, _, _ in vs]
    assert all(abs(a - b) >= rk.SEP_S for j, a in enumerate(ts) for b in ts[j + 1:])
    for i, u, v, r in vs:                                          # each view's pixel is the point's own
        x, y, _ = pl.to_floor(u, v, poses[i][2], K, D)
        assert math.hypot(x - xy[0], y - xy[1]) < 1e-3
    (none,) = rk.pick_views([(-3.0, 0.0)], poses, 640, 360, K, D, "equidistant")   # behind every frame
    assert none == []


def test_the_crop_holds_the_point_and_stays_in_the_image():
    for u, v, r in [(320, 180, 1.0), (5, 355, 0.6), (630, 10, 2.0), (320, 340, 0.3)]:
        x0, y0, s = rk.crop_box(u, v, r, 400.0, 640, 360)
        assert 0 <= x0 and x0 + s <= 640 and 0 <= y0 and y0 + s <= 360 and rk.CROP_MIN <= s <= rk.CROP_MAX
    x0, y0, s = rk.crop_box(320, 180, 1.0, 400.0, 640, 360)
    assert x0 <= 320 < x0 + s and y0 <= 180 < y0 + s and s == 160


def test_a_detection_counts_only_where_it_holds_the_candidates_point():
    assert rc.holds([100, 100, 150, 150], (120, 140), 384)
    assert rc.holds([100, 100, 150, 150], (160, 140), 384)        # within the grown margin (38 px)
    assert not rc.holds([100, 100, 150, 150], (250, 140), 384)

    class Fake(rc.Recognizer):
        def detect(self, rgb, ts=0.0):
            return [{"label": "ball", "box": [0, 0, 40, 40], "score": 0.9}, {"label": "toy", "box": [170, 170, 220, 220], "score": 0.3}]
    v = Fake().verify(np.zeros((384, 384, 3), np.uint8), (192, 192))
    assert v == {"score": 0.3, "label": "toy"}


def test_labels_naming_the_house_are_dropped_and_things_kept():
    for lab in ["floor", "Hardwood floor", "living room", "coffee table", "skirting board", "rug"]:
        assert rc.structural(lab), lab
    for lab in ["toy", "stuffed toy", "wooden block", "ball", "shoe", "teacup", "board game", "slipper"]:
        assert not rc.structural(lab), lab


def test_a_candidate_is_kept_by_its_best_verdict_and_kept_when_unseen():
    F = {"arm": "floor", "nights": {"n": {"points": [{"x": 1.0, "y": 1.0, "ts": 0, "score": 0.5},
                                                     {"x": 3.0, "y": 3.0, "ts": 0, "score": 1.0},
                                                     {"x": 5.0, "y": 5.0, "ts": 0, "score": 1.0}]}}}
    k1, k2 = rk.key("n", 1.0, 1.0), rk.key("n", 3.0, 3.0)
    index = {"nights": {"n": {k1: [{"id": k1 + "#0"}, {"id": k1 + "#1"}], k2: [{"id": k2 + "#0"}]}}}
    V = {k1 + "#0": {"score": 0.1}, k1 + "#1": {"score": 0.6}, k2 + "#0": {"score": 0.0}}
    kept = rk.filtered(F, index, V, 0.5)["nights"]["n"]["points"]
    assert [(p["x"], p["recog"]) for p in kept] == [(1.0, 0.6), (5.0, None)]
    assert all(p["score"] == 1.0 for p in kept)
    assert len(rk.filtered(F, index, V, 0.0)["nights"]["n"]["points"]) == 3
