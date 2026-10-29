"""[d8d]: the walk-through alone (HK_TIDY_WATCH_ONLY, the detection bench's recordings) judges every frame and logs each
decision but never stops for one, so the recording is the walk-through the tidy does, at its pace, with no stops."""
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
tidy = pytest.importorskip("houndkeeper.tidy_module")


def _robot(watch_only):
    r = object.__new__(tidy.TidyArm); r.WATCH_ONLY = watch_only
    r._pose = lambda: (np.array([0.0, 0.0, 0.3]), None)
    r._watch_frame = lambda: None; r._publish_keepout = lambda: None; r._stop = lambda: None
    r._decisions = lambda: {"x": 0.5, "y": 0.0, "decision": "confirmed", "kind": "present"}   # a find every second
    reached = iter([False] * 3 + [True] * 100)
    r._navigation = SimpleNamespace(set_goal=lambda g: None, cancel_goal=lambda: None, is_goal_reached=lambda: next(reached),
                                    get_state=lambda: tidy.NavigationState.FOLLOWING_PATH)
    return r


def test_the_tidy_stops_for_a_find():
    assert _robot(False)._walk_watching(np.array([1.0, 0.0]))[0] == "interrupted"


def test_watch_only_walks_on_to_the_waypoint():
    assert _robot(True)._walk_watching(np.array([1.0, 0.0])) == ("reached", None)


def test_only_spots_worth_a_stop_are_kept_out():
    """weak spots mustn't wall the robot in (27 Sep: watch-only walks got 'no path' to every later waypoint; live nights
    spent 39 of 43 stops settling them); spots worth a stop are still kept out, so it doesn't kick a toy"""
    r = object.__new__(tidy.TidyArm)
    r._cf = SimpleNamespace(keepout=lambda: [(0.5, 0.0, 0.08), (1.0, 1.0, 0.08)])
    r._binned, r._cleared, r._ctx = [], [], {"handled": []}
    r._sweep_log = [{"x": 1.0, "y": 1.02, "decision": "confirmed", "kind": "present", "hits": 20, "area_cm2": 300},
                    {"x": 0.5, "y": 0.0, "decision": "unsure", "hits": 3, "area_cm2": 10}]
    sent = []; r.keepout = SimpleNamespace(publish=lambda msg: sent.append(np.asarray(msg.as_numpy()[0] if isinstance(msg.as_numpy(), tuple) else msg.as_numpy())))
    r._publish_keepout()
    assert sent and np.allclose(sent[0][:, :2], [[1.0, 1.0]])


def test_only_spots_with_enough_evidence_are_stopped_for():
    """[6Pn]'s stop gate (27 Sep 2026): a weak spot costs a stop and a close look (~70 s) and was rarely a toy"""
    r = object.__new__(tidy.TidyArm)
    assert r._worth_a_stop({"kind": "present", "hits": 30, "area_cm2": 400})
    assert not r._worth_a_stop({"kind": "present", "hits": 3, "area_cm2": 400})      # seen too few times
    assert not r._worth_a_stop({"kind": "present", "hits": 30, "area_cm2": 9})       # a speck
    assert r._worth_a_stop({"kind": "gone", "hits": 60}) and not r._worth_a_stop({"kind": "gone", "hits": 10})
    assert not r.STOP_GATE["unsure"]
