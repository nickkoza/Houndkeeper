"""A find is placed and gripped as a whole item: the standing look hands on the item's middle, not its near end
(sim-slice [82p]), and a close look that sees only part of it, cut off by the image's edge, is not gripped from that
part (look-closer [TBu])."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))
import pick  # noqa: E402
import wrist_refine as wr  # noqa: E402

H, W, FOVY = 360, 640, 65.0
F = (H / 2) / np.tan(np.radians(FOVY) / 2)


def _cam(x=0.0, y=0.0, h=0.6):
    """a camera h up looking straight down, image up = +y on the floor"""
    return {"c": np.array([x, y, h]), "R": np.eye(3), "fovy": FOVY}


def _mask(cv, x0, x1, y0, y1, z=0.015):
    """the image pixels of a flat floor rectangle [x0, x1] x [y0, y1] at height z, as that camera sees it (clipped to the frame)"""
    d = cv["c"][2] - z; m = np.zeros((H, W), bool)
    u0, u1 = W / 2 + F * (x0 - cv["c"][0]) / d, W / 2 + F * (x1 - cv["c"][0]) / d
    v0, v1 = H / 2 - F * (y1 - cv["c"][1]) / d, H / 2 - F * (y0 - cv["c"][1]) / d
    m[max(int(v0), 0):min(int(v1), H), max(int(u0), 0):min(int(u1), W)] = True
    return m


def test_the_standing_look_hands_on_the_middle_of_a_lying_toy():
    # a toy 4 cm wide, 15 cm long, lying pointing away from the robot: its box's foot is its near end
    cv = _cam(); m = _mask(cv, -0.02, 0.02, 0.05, 0.20); vv, uu = np.nonzero(m)
    box = (uu.min(), vv.min(), uu.max(), vv.max())
    mid = wr.find_point(cv, box, m, middle=True); foot = wr.find_point(cv, box, m, middle=False)
    assert np.linalg.norm(mid["xy"] - [0.0, 0.125]) < 0.01            # the middle, within 1 cm
    assert np.linalg.norm(foot["xy"] - [0.0, 0.125]) > 0.04           # the old point: its near end
    assert abs(mid["length_cm"] - 15) < 1.5 and abs(abs(mid["long_axis"][1]) - 1) < 0.05


def test_a_mask_cut_off_by_the_frame_says_where():
    cv = _cam(h=0.33)                                                 # a close look: the frame spans +-0.20 m up and down
    whole = _mask(cv, -0.02, 0.02, 0.0, 0.15); cut = _mask(cv, -0.02, 0.02, 0.10, 0.40)
    assert wr.edge_contact(cv, whole) is None
    e = wr.edge_contact(cv, cut)
    assert e is not None and e[1] > 0.18 and abs(e[0]) < 0.03      # at the top edge, beyond what it saw


class _S:
    """just enough robot for look_closer: each look sees the item from a camera 33 cm over where it was aimed"""
    def __init__(self, item):
        self.item = item; self.aim = None; self.last_looks = []
    def base_xy(self): return np.array([0.0, -0.5])
    def state(self, s): pass
    def look_down(self, target, out, h_cam=0.3): self.aim = np.asarray(target, float) + out * pick.IN_VIEW; return np.zeros(6)
    def arm_to(self, q, secs, grip=None): pass
    def step(self, n): pass
    def capture(self): return {**_cam(self.aim[0], self.aim[1] - pick.IN_VIEW, 0.33), "robot": np.zeros((H, W), bool)}


def _detector(item):
    x0, x1, y0, y1 = item
    def det(cv, max_dist=None):
        m = _mask(cv, x0, x1, y0, y1)
        if m.sum() < 30: return None
        pts = wr.footprint_at(cv, m, 0.015); ctr = pts.mean(0)
        ray = np.r_[ctr, 0.015] - cv["c"]
        return {"xy": ctr, "cam": cv["c"].copy(), "ray": ray / np.linalg.norm(ray), "mask": m, "label": "a toy",
                "fits_jaws": True, "grip_width_cm": (x1 - x0) * 100, "edge_xy": wr.edge_contact(cv, m)}
    return det


def test_looks_that_all_see_a_fragment_look_again_at_the_cut(monkeypatch):
    monkeypatch.setattr(pick, "WHOLE_ITEM", True)
    item = (-0.02, 0.02, 0.10, 0.30)                                  # flagged at its near end (y 0.12)
    S = _S(item); ref, notes = pick.look_closer(S, np.array([0.0, 0.12]), detector=_detector(item))
    assert ref is not None and any("re-aimed" in n for n in notes)
    assert np.linalg.norm(ref["xy"] - [0.0, 0.20]) < 0.02             # fused on the whole toy's middle


def test_a_toy_never_seen_whole_goes_to_review(monkeypatch):
    monkeypatch.setattr(pick, "WHOLE_ITEM", True)
    item = (-0.02, 0.02, 0.10, 1.10)                                  # a metre long: no look holds it
    S = _S(item); ref, notes = pick.look_closer(S, np.array([0.0, 0.12]), detector=_detector(item))
    assert ref is None and "never seen whole" in notes


def test_off_the_rule_is_as_before(monkeypatch):
    monkeypatch.setattr(pick, "WHOLE_ITEM", False)
    item = (-0.02, 0.02, 0.10, 0.30)
    S = _S(item); ref, notes = pick.look_closer(S, np.array([0.0, 0.12]), detector=_detector(item))
    assert not any("re-aimed" in n for n in notes)


def test_a_finds_foot_with_no_mask_is_placed_in_the_looks_own_frame():
    """[82p] [7cU]: with no mask, a detection's foot is laid on the floor through the look's own image size, not the
    640 x 360 the arm's looks once had: the box centred in a 1280 x 720 look is straight below the camera"""
    for h, w in ((360, 640), (720, 1280)):
        cv = dict(_cam(1.0, 2.0), img=np.zeros((h, w, 3), np.uint8))
        p = wr.find_point(cv, (w / 2 - 10, h / 2 - 10, w / 2 + 10, h / 2 + 10), None, middle=False)
        f = (h / 2) / np.tan(np.radians(FOVY) / 2); vv = h / 2 + 10 - 0.1 * 20         # the box's foot, 8 px below the centre
        assert np.allclose(p["xy"], [1.0, 2.0 - (vv - h / 2) / f * 0.6], atol=1e-6)
