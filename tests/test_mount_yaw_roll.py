"""[KkQ] characterisation of a known limit of calibrate.mount_correction (docs/research/test-quality-2026-10-03.md, finding 2):
on walking lanes both ways, roll is only weakly observable, and a true roll leaks into the yaw found. With the mount
truly pitched 0.6 deg, turned 0.4 deg and rolled -0.3 deg, the correction comes back with pitch right but yaw ~0.51 deg
and roll ~-0.24 deg (measured 3 Oct 2026; the review earlier that day saw 0.54 and -0.18). This test pins that behaviour so a change to it is seen: if the procedure
gains a view that separates yaw from roll (a turn on the spot, a wall), this test fails and should be replaced by one
that asserts the true yaw and roll. It is not a statement that the coupling is acceptable for the floor map."""
import math

from test_calibrate_mutants import K, _floor_tex, _pose, _render, cal

import numpy as np


def _lanes(true):
    tex = _floor_tex(np.random.default_rng(0)); corr = cal._rot(*(math.radians(a) for a in true)); frames = []
    for lane in (-0.4, 0.4):
        for x in np.arange(-1.5, 1.6, 0.5):
            for heading, x0 in ((0.0, x - 1.5), (math.pi, x + 1.5)):
                T = _pose(x0, lane, heading); frames.append((_render(tex, T, corr), T, K))
    return frames


def test_a_true_roll_leaks_into_the_yaw_found_from_walking_lanes():
    """[KkQ] pitch is found; yaw is pulled about 0.1 deg off by a -0.3 deg roll, and the roll is under-read"""
    (p, y, r), scores = cal.mount_correction(_lanes((0.6, 0.4, -0.3)))
    print(f"found pitch {p:.3f} yaw {y:.3f} roll {r:.3f} (true 0.6, 0.4, -0.3)")
    assert abs(p - 0.6) < 0.1, p
    assert abs(y - 0.51) < 0.05 and y - 0.4 > 0.06, y          # the coupling: yaw off by well over the 0.03 deg it
    assert abs(r + 0.235) < 0.05 and r - (-0.3) > 0.03, r         # reaches with no roll; roll under-read
