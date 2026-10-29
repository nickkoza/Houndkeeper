"""The grasp proposers behind one interface (sim-grasp [WuR]): each answers a look with DimOS's grasp message, its
widths and its allowances; ours gives the pick's own grip options unchanged; DimOS's come through the point-cloud
adapter."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))


def _bar_look(angle_deg):
    """a look at a 12 x 3 cm, 4 cm tall bar turned angle_deg at (0.5, 0.2), its grip options as the look measures them"""
    H, W = 60, 80; a = np.radians(angle_deg); u, v = np.meshgrid(np.linspace(-0.1, 0.1, W), np.linspace(-0.075, 0.075, H))
    along = u * np.cos(a) + v * np.sin(a); across = -u * np.sin(a) + v * np.cos(a)
    on = (np.abs(along) < 0.06) & (np.abs(across) < 0.015)
    h = np.where(on, 0.04, 0.0); P = np.stack([0.5 + u, 0.2 + v, h], -1)
    narrow = np.array([-np.sin(a), np.cos(a)])
    return {"xy": np.array([0.5, 0.2]), "grip_width_cm": 3.0, "short_axis": narrow,
            "grip_options": [(0.030, narrow, np.zeros(2)), (0.031, narrow, np.array([0.02, 0.0])), (0.12, -narrow[::-1] * [1, -1], np.zeros(2))],
            "_cv": {"stereo_P": P, "stereo_h": h, "robot": np.zeros((H, W), bool)}, "mask": on | np.roll(on, 3, 1)}


def test_ours_offers_the_looks_own_grip_options_unchanged():
    import grasp_proposers as gp, pick
    look = _bar_look(30.0); prop = gp.proposer("ours").propose(look)
    want = pick.real_grips(look["grip_options"])[:12]
    assert len(prop.grasps.candidates) == len(want) == len(prop.widths_m) == len(prop.allowances)
    scores = [c.score for c in prop.grasps.candidates]; assert scores == sorted(scores, reverse=True)
    for c, w_, al, (w0, jaw, off) in zip(prop.grasps.candidates, prop.widths_m, prop.allowances, want):
        p, R = gp.pose_arrays(c.pose)
        assert np.allclose(p[:2], look["xy"] + off) and w_ == w0
        assert np.allclose(R[:, 2], [0, 0, -1], atol=1e-6)                   # straight down, the lean left to the pick
        assert abs(abs(R[:2, 1] @ jaw) - 1) < 1e-6                          # closing along the option's jaw direction
        assert al.around == "vertical" and al.height == "tips" and al.lean_deg == pick.MAX_LEAN


def test_the_wrist_turns_to_the_grasp_frame():
    import grasp_proposers as gp
    R = gp._straight_down([0.0, 1.0]); L7 = gp.link7_from_tcp(R)
    assert np.allclose(L7[:, 2], [0, 0, 1]) and np.allclose(L7[:, 1], [0, 1, 0])    # wrist -z down the approach, y closing
    assert np.isclose(np.linalg.det(L7), 1.0)
    p, R2 = gp.pose_arrays(gp.tcp_pose([0.1, 0.2, 0.3], R)); assert np.allclose(p, [0.1, 0.2, 0.3]) and np.allclose(R2, R)


def test_dimos_heuristic_comes_through_the_cloud_adapter():
    import grasp_proposers as gp
    for deg in (0.0, 30.0, 90.0):
        prop = gp.proposer("dimos").propose(_bar_look(deg))
        (c,) = prop.grasps.candidates; p, R = gp.pose_arrays(c.pose)
        a = np.radians(deg); narrow = np.array([-np.sin(a), np.cos(a)])
        assert np.allclose(p[:2], [0.5, 0.2], atol=0.01) and abs(p[2] - 0.04) < 0.005   # the floor's points left out
        assert abs(abs(R[:2, 1] @ narrow) - 1) < 0.02 and R[2, 2] < -0.99              # across the narrow side, straight down
        assert prop.allowances[0].height == "raise"


def test_graspgenx_offers_only_grasps_from_above():
    import grasp_proposers as gp
    down, up = np.eye(4), np.eye(4); down[:3, :3] = gp._straight_down([0, 1]); up[:3, :3] = np.diag([1.0, -1.0, -1.0]) @ gp._straight_down([0, 1])
    class Fake:
        def ask(self, pts): return np.stack([up, down]), np.array([0.9, 0.8])
    gp.GraspGenXGrasps._client = Fake()
    try:
        prop = gp.proposer("graspgenx").propose(_bar_look(0.0))
    finally:
        gp.GraspGenXGrasps._client = None
    assert [c.score for c in prop.grasps.candidates] == [0.8] and prop.allowances[0].around == "pose"


def test_no_cloud_is_a_reason_not_a_grasp():
    import grasp_proposers as gp
    look = _bar_look(0.0); look["_cv"]["stereo_h"] = np.zeros_like(look["_cv"]["stereo_h"])   # nothing stands off the floor
    assert isinstance(gp.proposer("dimos").propose(look), str) and isinstance(gp.proposer("graspgenx").propose(look), str)
    assert gp.proposer("ours").propose(look).grasps.candidates                              # ours needs no cloud


def test_a_tall_item_is_gripped_higher_with_grip_lift(monkeypatch):
    import grasp_proposers as gp, pick
    monkeypatch.setattr(gp, "GRIP_LIFT", True)
    short = gp.proposer("ours").propose(_bar_look(0.0))                     # 4 cm tall: tips at the floor as before
    assert all(al.tip_m == pick.TIP_CLEAR for al in short.allowances)
    tall = _bar_look(0.0); h = tall["_cv"]["stereo_h"]; tall["_cv"]["stereo_h"] = np.where(h > 0, 0.10, 0.0)
    tall["_cv"]["stereo_P"] = tall["_cv"]["stereo_P"].copy(); tall["_cv"]["stereo_P"][..., 2] = tall["_cv"]["stereo_h"]
    got = gp.proposer("ours").propose(tall)                                 # 10 cm tall: tips at 3.5 cm
    assert all(abs(al.tip_m - 0.035) < 1e-6 for al, w in zip(got.allowances, got.widths_m) if w >= pick.THIN)
