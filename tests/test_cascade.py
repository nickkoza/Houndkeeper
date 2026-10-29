"""[bFx], [3hK], [Btx]: the near-floor grid proposes, a light check per candidate trims (sim/house/baselines/cascade.py),
on synthetic replays and crops, no model loaded: the threshold is the strictest that keeps found at its floor (recall
first), the loosest when none does; a candidate no frame saw is kept whatever the threshold; stage 1 alone is scored as
the floor arm scores its decisions; the multi-view votes read the candidate's own layer; the contrast score sees a
patch that differs from the floor around it and not a plain floor."""
import json
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))

from baselines import cascade as ca, floor_arm as fa, recheck as rk   # noqa: E402


def test_choose_is_the_strictest_threshold_keeping_found():
    curve = [{"thr": t, "found": f} for t, f in [(-1e9, 45), (0.1, 45), (0.2, 45), (0.3, 44), (0.5, 40)]]
    assert ca.choose(curve, 45) == 0.2
    assert ca.choose(curve, 46) == -1e9                      # none reaches it: keep everything


def replay_dir(tmp_path):
    dec = [{"decision": "confirmed", "x_robot": 1.0, "y_robot": 2.0, "ts": 5.0, "cells": [[1.0, 2.0]]},
           {"decision": "unsure", "x_robot": 3.0, "y_robot": 1.0, "ts": 6.0, "layer": "near", "cells": [[3.0, 1.0]]},
           {"decision": "not_a_find", "x_robot": 4.0, "y_robot": 1.0, "ts": 7.0},
           {"decision": "confirmed", "x_robot": 5.0, "y_robot": 1.0, "ts": 8.0, "kind": "floor_change"}]
    json.dump({"night": "s1_t1_hr", "decisions": dec}, open(tmp_path / "s1_t1_hr_loose1.json", "w"))
    return tmp_path


def test_candidates_and_their_scores(tmp_path):
    d = replay_dir(tmp_path)
    assert [r["decision"] for r, _ in ca.decisions(d)["s1_t1_hr"]] == ["confirmed", "unsure"]
    k1, k2 = rk.key("s1_t1_hr", 1.0, 2.0), rk.key("s1_t1_hr", 3.0, 1.0)
    A = ca.arm(d, {k1: 0.3, k2: None})                       # k2: no frame saw it
    s = [p["score"] for p in A["nights"]["s1_t1_hr"]["points"]]
    assert s[0] == 0.3 and s[1] == ca.KEPT
    A0 = ca.arm(d, None)
    assert [p["score"] for p in A0["nights"]["s1_t1_hr"]["points"]] == [fa.SCORE["confirmed"], fa.SCORE["unsure"]]


def test_votes_read_the_candidates_own_layer():
    hit = np.zeros((980, 1680)); seen = np.zeros_like(hit)
    i, j = int((ca.HY - 2.0) / ca.RES), int((1.0 + ca.HX) / ca.RES); hit[i, j], seen[i, j] = 3, 4
    near = {"hit": np.zeros((10, 10)), "seen": np.ones((10, 10)), "res": 0.005}
    r = {"cells": [[1.0, 2.0]]}
    assert ca.votes(r, {"hit": hit, "seen": seen}, None) == 0.75
    assert ca.votes(dict(r, layer="near"), {"hit": hit, "seen": seen}, near) == 0.0   # outside the near grid: nothing


def test_contrast_sees_a_patch_not_a_plain_floor():
    floor = np.full((384, 384, 3), 120, np.uint8)
    assert ca.contrast(floor, (192, 192), 30) < 1.0
    toy = floor.copy(); yy, xx = np.mgrid[:384, :384]; toy[np.hypot(xx - 192, yy - 192) <= 30] = (220, 40, 40)
    assert ca.contrast(toy, (192, 192), 30) > 20.0
