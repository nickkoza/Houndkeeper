"""[BPL]: the night's balance record calls a fall a fall: the trunk tipped past 45 deg or dropped well below its lying
height. Lying itself (0.111-0.126 m on odometry, tilt up to 11 deg, the sim pickup nights of 18 Oct) is not a fall."""
import math
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


def _fell(z, tilt_deg):
    from houndkeeper.tidy_module import TidyArm
    t = TidyArm.__new__(TidyArm); t._live_loc = None
    t._balance = {"n": 0, "max_deg": 0.0, "min_z": 9.0, "over5": 0, "over10": 0, "fell": False}
    h = math.radians(tilt_deg) / 2
    msg = types.SimpleNamespace(ts=None, position=types.SimpleNamespace(x=0.0, y=0.0, z=z),
                                orientation=types.SimpleNamespace(w=math.cos(h), x=math.sin(h), y=0.0, z=0.0))
    TidyArm._on_odom(t, msg); return t._balance["fell"]


def test_lying_is_not_a_fall():
    assert not _fell(0.111, 11.0) and not _fell(0.126, 4.5) and not _fell(0.30, 5.0)


def test_tipped_over_or_dropped_below_lying_is_a_fall():
    assert _fell(0.30, 60.0) and _fell(0.05, 2.0)
