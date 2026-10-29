"""[nAH] UMH: odometry reaches the robot late and unevenly, in order; DimOS's Go2 connection stamps it on arrival (so the
pose it gives is late by the delay), or, once the robot's own time is carried, with the time it describes."""
import sys
from pathlib import Path

from houndkeeper import workspace

import numpy as np

sys.path.insert(0, str(workspace.dimos()))
from dimos.simulation.mujoco.odom_delay import OdomDelay  # noqa: E402


def _run(stamp):
    d = OdomDelay(0.03, 0.01, seed=1, stamp=stamp); out = []
    for k in range(400):
        t = k * 0.005; got = d.push(t, [t, 0, 0], [1, 0, 0, 0])
        if got is not None: out.append((t, got[2], got[0][0]))
    return (np.array(v) for v in zip(*out))


def test_stamped_on_arrival_by_default():
    now, stamp, x = _run("arrival")
    assert np.all(np.diff(stamp) >= 0) and np.all(stamp <= now + 1e-9)
    lag = stamp - x; assert 0.015 < np.median(lag) < 0.045            # the state is older than its stamp says


def test_stamped_at_source_when_carried():
    now, stamp, x = _run("source")
    assert np.allclose(stamp, x) and 0.015 < np.median(now - stamp) < 0.045
