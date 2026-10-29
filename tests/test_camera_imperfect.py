"""[kJ6]: the sim's camera frames carry what a real camera does to them ([x3f]): blur from the camera's own motion over
its exposure (long at night), noise, compression."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from houndkeeper.camera_imperfect import Degrade  # noqa: E402

H, W, FOVY = 360, 640, 51.8
F = (H / 2) / math.tan(math.radians(FOVY) / 2)


def _stripes():
    img = np.zeros((H, W, 3), np.uint8); img[:, W // 2:] = 200          # one sharp vertical edge
    return img


def _level_camera(yaw):
    """looking along +x, level, 0.3 m up, turned yaw about world z (MuJoCo camera frame: x right, y up, -z forward)"""
    fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0]); up = np.array([0, 0, 1.0]); right = np.cross(fwd, up)
    return np.array([0, 0, 0.3]), np.column_stack([right, up, -fwd])


def _edge_width(img):
    row = img[H // 4 - 10:H // 4 + 10, :, 0].astype(float).mean(0)         # rows above the horizon (the far wall), noise averaged
    return int(((row > 20) & (row < 180)).sum())


def _run(ev, yaw_rate):
    d = Degrade(FOVY); d.s = dict(d.s, jpeg_quality=100)
    C, R = _level_camera(0.0); d(_stripes(), 0.0, C, R, ev, 1.0)
    C, R = _level_camera(yaw_rate * 0.05); return d(_stripes(), 0.05, C, R, ev, 1.0)


def test_a_turning_camera_blurs_at_night_as_its_exposure_says():
    rate = math.radians(30)                                               # a turn in place
    night = _run(ev=5.0, yaw_rate=rate); day = _run(ev=1.0, yaw_rate=rate)
    expect = F * rate * 0.033                                             # pixels swept in the night's 33 ms exposure
    assert abs(_edge_width(night) - expect) < 0.5 * expect + 2
    assert _edge_width(day) < _edge_width(night) / 3


def test_a_still_camera_stays_sharp_but_noisy():
    still = _run(ev=5.0, yaw_rate=0.0)
    assert _edge_width(still) <= 2
    diff = still.astype(float) - _stripes().astype(float)
    assert 0.5 < diff.std() < 10 and abs(diff.mean()) < 1.0              # noise, not a shift in brightness


def test_the_cameras_own_processing_leaves_less_noise_and_blotchier(monkeypatch):
    """a real camera reduces noise before it compresses (waymark 0008 [kJ6]; audit 28 Sep: the sim added the full raw
    noise, ~11 grey levels at 16x gain): what survives is isp_noise_keep of it, smoothed over isp_noise_blur_px"""
    import numpy as np
    from houndkeeper.camera_imperfect import Degrade
    flat = np.full((180, 320, 3), 100, np.uint8); R = np.eye(3); C = np.zeros(3)
    def added(keep, blur):
        d = Degrade(65.0, None); d.s = dict(d.s, jpeg_quality=100, isp_noise_keep=keep, isp_noise_blur_px=blur)
        d.last = (0.0, C, R); out = d(flat.copy(), 0.0, C, R, ev=5.0, ev_day=1.0).astype(float) - 100.0
        return out.std(), np.corrcoef(out[:, :-1, 0].ravel(), out[:, 1:, 0].ravel())[0, 1]
    raw_sd, raw_c = added(1.0, 0.0); isp_sd, isp_c = added(0.3, 1.0)
    assert abs(isp_sd / raw_sd - 0.3) < 0.05                    # 30% of the noise survives
    assert raw_c < 0.1 < 0.3 < isp_c                            # raw noise is per pixel; what survives is blotchy


def test_the_battery_model_reads_the_robots_parameters():
    """walking power and the pack's size come from params/robot.yaml, the one place they're sourced (28 Sep audit: the
    battery model kept its own 150 W after the parameter moved to 170)"""
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))
    import battery_model as bm
    from houndkeeper import params as P
    p = bm.Params()
    assert p.walk_W == P.get("go2_air.walk_power_W") and p.air_Wh == P.get("go2_air.battery_Wh")
