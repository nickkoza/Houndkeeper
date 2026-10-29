"""[uKM] 2: the camera's stamp lag against the odometry is found from the robot's own turning (no truth, no target):
frames whose image slides with the turn, stamped late, and the lag recovered."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from houndkeeper import calibrate as cal  # noqa: E402


def test_the_frames_lag_is_found_from_turning():
    rng = np.random.default_rng(0); scene = rng.random((120, 2400)) * 255                   # a wide textured wall
    t = np.arange(0, 40, 0.05); rate = 0.6 * np.sin(2 * math.pi * t / 7) + 0.3 * np.sin(2 * math.pi * t / 2.3)   # turning, rad/s
    yaw = np.cumsum(rate) * 0.05; f = 400.0; lag = 0.08
    frames = []
    for k, tk in enumerate(t):
        x0 = int(round(1000 - f * yaw[k]))                                                   # turning left: the view's window moves
        img = np.zeros((240, 640)); img[:120] = scene[:, x0 - 320: x0 + 320]                # left over the wall, the wall slides right
        frames.append((tk + lag, img))                                                       # stamped late
    got, c = cal.frame_lag(frames, t, yaw)
    assert abs(got - lag) < 0.015 and c > 0.8
