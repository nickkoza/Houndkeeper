"""[KkQ]: the Confirmer judges in the house's frame while the robot walks in its odometry's: a frame goes in moved by
the live correction, and the spots, their cells and the keep-out discs come back in the odometry's frame."""
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from houndkeeper import live_localize as ll  # noqa: E402
from houndkeeper.localize import se2  # noqa: E402


class _Fixed:
    def __init__(self, C): self.C = C
    def correction(self, t=None): return self.C


class _Confirmer:
    """records what it was given; reports one spot at a fixed house position"""
    hmap = None
    def frame(self, img, T): self.T = T
    def decide(self, t, xy): self.xy = xy; return [{"x": 2.0, "y": 1.0, "cells": [[2.0, 1.0]], "decision": "confirmed"}]
    def keepout(self): return [(2.0, 1.0, 0.1)]


def test_frames_go_in_moved_and_spots_come_back():
    C = se2(0.10, -0.05, math.radians(2.0)); cf = _Confirmer(); hf = ll.HouseFramed(cf, _Fixed(C))
    T = np.eye(4); T[:3, 3] = [1.0, 0.5, 0.3]
    hf.frame(None, T)
    assert np.allclose(cf.T[:2, 3], C[:2, :2] @ [1.0, 0.5] + C[:2, 2]) and abs(cf.T[2, 3] - 0.3) < 1e-9
    rec = hf.decide(0.0, (1.0, 0.5))[0]
    assert np.allclose(cf.xy, C[:2, :2] @ [1.0, 0.5] + C[:2, 2])
    back = C[:2, :2] @ [rec["x"], rec["y"]] + C[:2, 2]
    assert np.allclose(back, [2.0, 1.0], atol=2e-3) and (rec["x_house"], rec["y_house"]) == (2.0, 1.0)
    assert np.allclose(rec["cells"][0], [rec["x"], rec["y"]], atol=0.01)
    x, y, r = hf.keepout()[0]; assert np.allclose(C[:2, :2] @ [x, y] + C[:2, 2], [2.0, 1.0]) and r == 0.1
    hf.hmap = "set"; assert cf.hmap == "set"                                   # settings pass through to the Confirmer


def test_the_live_localizer_matches_scans_as_they_arrive():
    """a room's walls, believed 8 cm and 2 deg off: the correction found live brings them back"""
    walls = [np.c_[np.linspace(a[0], b[0], 200), np.linspace(a[1], b[1], 200), np.full(200, 0.5)]
             for a, b in (((0, 0), (5, 0)), ((5, 0), (5, 4)), ((5, 4), (0, 4)), ((0, 4), (0, 0)), ((2, 1), (2, 2.5)))]
    room = np.vstack(walls); house = room[:, :2]
    E = se2(0.08, 0.0, math.radians(2.0)); Ei = np.linalg.inv(E)             # what the robot believes: the room moved by E^-1
    seen = room.copy(); seen[:, :2] = room[:, :2] @ Ei[:2, :2].T + Ei[:2, 2]
    live = ll.LiveLocalizer(house)
    for k in range(6):
        live.odom((2.5 + 0.05 * k, 2.0)); live.scan(float(k), seen); time.sleep(0.3)
    t0 = time.time()
    while len(live._t) < 3 and time.time() - t0 < 10: time.sleep(0.1)
    live.close(); C = live.correction()
    assert np.allclose(C, E, atol=0.01), C


def test_floor_tracking_moves_spots_back_but_not_frames_twice():
    """[WFI]: with the Confirmer tracking the floor itself, its frames go in as they are, and its spots come back to
    odometry's frame by the floor's correction"""
    C = se2(-0.04, 0.03, math.radians(-1.0)); cf = _Confirmer(); cf.Cf = C
    hf = ll.HouseFramed(cf, ll.FloorTrack(cf), moves_frames=False)
    T = np.eye(4); T[:3, 3] = [1.0, 0.5, 0.3]; hf.frame(None, T)
    assert np.allclose(cf.T, T)
    rec = hf.decide(0.0, (1.0, 0.5))[0]
    assert np.allclose(C[:2, :2] @ [rec["x"], rec["y"]] + C[:2, 2], [2.0, 1.0], atol=2e-3)
