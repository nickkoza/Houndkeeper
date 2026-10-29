"""[1B2], [KkQ], [d8d]: the truth reaches robot code's results only from beside it. The pose ablation (true_path.py)
moves the robot's pose lookup onto the sim's true path through the tools' pose_hook; the confirmer's replay is scored
by score_confirm.py; the robot's own tools offer no truth option."""
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "src")); sys.path.insert(0, str(HK / "sim" / "house"))


def test_the_pose_ablation_moves_the_pose_onto_the_true_path(tmp_path):
    """[KkQ]: true_path.hook wraps a pose lookup so it returns the sim's true x, y and heading at that time"""
    import true_path as tp
    p = tmp_path / "traj.csv"
    p.write_text("wall_t,odom_x,odom_y,odom_yaw,true_x,true_y,true_yaw\n0,0,0,0,1,2,0.5\n10,0,0,0,1,2,0.5\n")
    pose = tp.hook(p)(lambda t: np.eye(4))
    M = pose(5.0)
    assert M[0, 3] == pytest.approx(1) and M[1, 3] == pytest.approx(2) and np.arctan2(M[1, 0], M[0, 0]) == pytest.approx(0.5)
    with pytest.raises(SystemExit): tp.main([str(p), "confirm_stream"])      # only the tools that take a pose_hook


def test_the_robots_tools_offer_no_truth_option():
    """[1B2]: floor_mosaic2, calibrate_mount and the confirmer's replay have no --truth; their scorer and the ablation do"""
    pytest.importorskip("dimos")
    import inspect, floor_mosaic2 as fm, calibrate_mount as cm, confirm_stream as cs, score_confirm as sc
    for m in (fm, cm):
        assert "--truth" not in inspect.getsource(m.main) and "pose_hook" in inspect.signature(m.main).parameters
    assert not any("--truth" in a.option_strings for a in cs.parser()._actions)
    assert "--truth" in inspect.getsource(sc.main)


def test_the_replays_scorer_finds_an_item_a_confirmed_spot_reaches():
    """[d8d]: score_confirm.score: an item is found if a confirmed spot reaches within its size + 6 cm; a confirmed spot
    near no item, stain or item gone since last night is a stop for nothing"""
    pytest.importorskip("dimos")
    import score_confirm as sc
    recs = [{"decision": "confirmed", "cells": [(1.0, 1.0)], "range_m": 1.5},
            {"decision": "confirmed", "cells": [(3.0, 3.0)], "range_m": 2.0},
            {"decision": "unsure", "cells": [(5.0, 5.0)], "range_m": 1.0}]
    t = {"items": [{"id": "a", "kind": "toy", "x": 1.05, "y": 1.0, "half": [0.03, 0.03]},
                   {"id": "b", "kind": "toy", "x": 5.0, "y": 5.0, "half": [0.03, 0.03]}], "stains": []}
    per, false, stains = sc.score(recs, t, {})
    assert per == {"toy": [1, 1]} and false == 1 and stains == 0 and t["items"][0]["_range"] == 1.5
