"""[d8d]'s standing look: the wrist camera aimed so the find sits above the jaws in the image, where it isn't hidden
behind them (27 Sep: a toy on the jaws' tip was dropped as the robot's own part)."""
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "arm"))
import standing_look as sl  # noqa: E402
import vslice as vs  # noqa: E402


def test_the_find_sits_up_deg_above_the_image_centre():
    S = vs.Sim("sock", np.array([30.0, 30.0]), with_obj=False, watch=False)
    tr = S.d.xpos[S.m.body("trunk").id]; tgt = np.array([tr[0] + 0.75, tr[1], 0.0])
    q0 = S.d.qpos[S.aq].copy()
    q, miss, _ = sl.aim(S, tgt, q0, up_deg=15.0)
    assert miss < 2.0
    kd = S.kin; kd.qpos[:] = S.d.qpos; kd.qpos[S.aq] = q; mujoco.mj_kinematics(S.m, kd); mujoco.mj_camlight(S.m, kd)
    cid = S.m.camera("arm_wrist").id; X = kd.cam_xmat[cid].reshape(3, 3); p = X.T @ (tgt - kd.cam_xpos[cid])
    up = math.degrees(math.atan2(p[1], -p[2]))                                   # above the optical axis, in the image
    assert 13.0 < up < 17.0 and abs(math.degrees(math.atan2(p[0], -p[2]))) < 2.0
