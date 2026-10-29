"""The stereo pair in the close look (look-closer [GMs]): the item whole where the detector boxed a part, and a part
gripped only where both fingers have floor to land beside it."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))


def test_a_boxed_part_grows_to_the_standing_item_and_a_whole_box_stays():
    import wrist_stereo as ws
    h = np.zeros((100, 160)); h[20:80, 30:130] = 0.05                     # a 5 cm tall item
    robot = np.zeros_like(h, bool); stand = ws.standing(h, robot)
    head = np.zeros_like(robot); head[30:45, 40:60] = True                 # the detector boxed its head
    m, grew = ws.whole(head, stand); assert grew and m[20:80, 30:130].mean() > 0.98      # (cleaning rounds the corners)
    all_of_it = np.zeros_like(robot); all_of_it[21:79, 31:129] = True
    m, grew = ws.whole(all_of_it, stand); assert not grew                  # already whole: the detector's own mask kept


def test_fingers_need_floor_beside_a_part():
    import wrist_stereo as ws
    xs, ys = np.meshgrid(np.linspace(-0.1, 0.1, 201), np.linspace(-0.1, 0.1, 201))
    P = np.dstack([xs, ys, np.zeros_like(xs)])
    leg = np.where(np.abs(xs) < 0.015, 0.05, 0.0)                           # a 3 cm leg standing, floor either side
    assert ws.fingers_land((0.0, 0.0), (1.0, 0.0), 0.03, P, leg)
    rim = np.where(xs > -0.015, 0.05, 0.0)                                  # a rim with the bowl on one side
    assert not ws.fingers_land((0.0, 0.0), (1.0, 0.0), 0.03, P, rim)


def test_an_item_never_seen_whole_is_too_big_when_even_its_seen_part_wont_fit():
    import pick
    cut_big = [{"whole_from_stereo": True, "length_cm": 34.0, "breadth_cm": 12.0, "xy": [0, 0]},
               {"whole_from_stereo": False, "length_cm": 28.0, "breadth_cm": 25.0, "xy": [0, 0]}]
    ref = pick.seen_part_too_big(cut_big)
    assert ref is not None and ref["length_cm"] == 34.0 and ref["breadth_cm"] == 25.0 and not pick.fits_bin(ref)
    small = [{"whole_from_stereo": True, "length_cm": 20.0, "breadth_cm": 9.0, "xy": [0, 0]}]
    assert pick.seen_part_too_big(small) is None                             # it could still fit: review, as before
    no_pair = [{"whole_from_stereo": None, "length_cm": 40.0, "breadth_cm": 30.0, "xy": [0, 0]}]
    assert pick.seen_part_too_big(no_pair) is None                           # without the pair, no lower bound is trusted


def test_heights_work_on_a_live_look_with_no_truth():
    import stereo_eval
    rng = np.random.default_rng(0); img = (rng.random((720, 1280, 3)) * 255).astype(np.uint8)
    f640 = 180 / np.tan(np.radians(25.0)); d640 = f640 * 0.05 / 0.24          # a surface 24 cm below a camera 30 cm up
    cv = {"img": img, "img_r": np.roll(img, -2 * int(round(d640)), axis=1), "robot": np.zeros((720, 1280), bool), "fovy": 50.0,
          "c": np.array([0.0, 0.0, 0.3]), "R": np.eye(3), "baseline": 0.05}  # looking straight down; a live look: no "truth"
    h = stereo_eval.heights(cv)                                               # (29 Sep: the binning wanted it, and failed)
    assert h.shape == (720, 1280) and abs(np.nanmedian(h) - 0.06) < 0.01


def test_close_looks_are_framed_for_the_camera_fitted():
    import pick
    from types import SimpleNamespace
    import mujoco
    spec = mujoco.MjSpec(); b = spec.worldbody.add_body(); c = b.add_camera(); c.name = "arm_wrist"
    for fovy, lo, hi in ((65.0, 0.99, 1.01), (50.3, 1.3, 1.45), (80.0, 0.99, 1.01)):
        c.fovy = fovy; k = pick.framing(SimpleNamespace(m=spec.compile()))
        assert lo <= k <= hi, (fovy, k)                                       # narrower: further back and up; never closer in
