"""[tmq]: kept-out spots stay kept out until binned or found empty by the close look; the ones walling the robot in are
the nearest unsettled ones."""
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
tidy = pytest.importorskip("houndkeeper.tidy_module")


def _robot(spots, at=(0.0, 0.0)):
    r = object.__new__(tidy.TidyArm)
    r._cf = SimpleNamespace(keepout=lambda: [(x, y, 0.08) for x, y in spots])
    r._binned, r._cleared, r._ctx = [], [], {"handled": []}
    r._pose = lambda: (np.array([at[0], at[1], 0.3]), None)
    return r


def test_blockers_are_the_nearest_unsettled_spots():
    r = _robot([(0.4, 0.0), (1.0, 0.2), (3.0, 0.0)])
    assert r._blockers() == [(0.4, 0.0), (1.0, 0.2)]            # within reach, nearest first


def test_settled_spots_stop_blocking():
    r = _robot([(0.4, 0.0), (1.0, 0.2)])
    r._settled(1, (0.4, 0.0), {"outcome": "review: not seen at the flag"})    # nothing there: cleared
    r._settled(2, (1.0, 0.2), {"outcome": "binned"})                           # binned: gone
    assert r._blockers() == []
    r2 = _robot([(0.4, 0.0)]); r2._ctx["handled"] = [(1, np.array([0.4, 0.0]))]
    r2._settled(1, (0.4, 0.0), {"outcome": "review: too wide to grip"})        # a real item left for review:
    assert r2._blockers() == [] and not r2._cleared                           # handled, and still kept out


def test_a_find_cleared_by_the_standing_look_stops_being_kept_out():
    r = _robot([(0.4, 0.0)])
    r._settled(1, (0.4, 0.0), {"outcome": "review: not seen standing"})
    assert r._blockers() == [] and r._cleared
