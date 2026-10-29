"""[z7P]: the simulated arm is not kinder than the real one ([x3f]): the encoders can't see all of where the camera is,
the arm sags on its springs, and the model stays stable with the imperfections on (and starts at rest)."""
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "arm")); sys.path.insert(0, str(HK / "src"))
mujoco = pytest.importorskip("mujoco")


def _settled():
    import go1_arm as ga, payload_test as pt, legs
    m = ga.build(True, pt.STOW, pick=True, objects=[{"name": "box", "pos": [0.6, 0.2, 0.05], "half": [0.03, 0.03, 0.03], "mass": 0.1, "rgba": [1, 0, 0, 1]}])
    m.opt.timestep = 0.005; d = mujoco.MjData(m); legs.start_standing(m, d, 0.30)
    for _ in range(400): mujoco.mj_step(m, d)
    return m, d


def test_the_arm_starts_at_rest_and_stays_stable():
    m, d = _settled()
    assert d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number == 0
    assert np.allclose(d.xpos[m.body("box").id][:2], [0.6, 0.2], atol=0.01)       # objects start where placed
    flex = [m.jnt_qposadr[j] for j in range(m.njnt) if "_flex_" in m.joint(j).name or m.joint(j).name.endswith("_lash")]
    assert flex and np.abs(d.qpos[flex]).max() < np.radians(5)


def test_the_encoders_do_not_see_where_the_wrist_camera_is():
    import go1_arm as ga
    m, d = _settled(); cid = m.camera("arm_wrist").id
    kin = mujoco.MjData(m); kin.qpos[:] = d.qpos
    for j in range(m.njnt):                                             # the robot's belief: encoders, rigid frame
        if "_flex_" in m.joint(j).name or m.joint(j).name.endswith("_lash"): kin.qpos[m.jnt_qposadr[j]] = 0.0
    mujoco.mj_kinematics(m, kin); mujoco.mj_camlight(m, kin)
    Rt, Rb = d.cam_xmat[cid].reshape(3, 3), kin.cam_xmat[cid].reshape(3, 3)
    err = np.degrees(np.arccos(np.clip((np.trace(Rb.T @ Rt) - 1) / 2, -1, 1)))
    assert err > 0.2                                                    # imperfect, as the printed arm will be
    aq = [m.jnt_qposadr[m.joint("arm_" + j).id] for j in ga.ARM_JOINTS[:6]]
    assert np.abs(d.qpos[aq] - d.ctrl[12:18]).max() > 1e-3             # the joints sag off target (no perfect gravity comp)


def _walk_over_a_step(substeps, h=0.012):
    """the robot, with the imperfect arm, walks up onto a carpet-height step and back off it: its worst tilt"""
    import go1_arm as ga
    _orig = mujoco.MjSpec.from_file
    def from_file(*a, **k):
        sp = _orig(*a, **k)
        if "_flat_" in str(a[0]):
            g = sp.worldbody.add_geom(); g.type = mujoco.mjtGeom.mjGEOM_BOX; g.size = [1.0, 1.5, h / 2]; g.pos = [1.6, 0, h / 2]
            g.contype = 1; g.conaffinity = 0
        return sp
    mujoco.MjSpec.from_file = from_file
    old = ga.SUBSTEPS; ga.SUBSTEPS = substeps
    try:
        import vslice as vs
        S = vs.Sim("sock", np.array([30.0, 30.0]), with_obj=False); S.m.opt.timestep = ga.CTRL_DT / substeps
        S.walk_to(np.array([1.6, 0.0]), 0.0, max_s=25, settle=False); S.walk_to(np.array([-0.3, 0.0]), np.pi, max_s=25, settle=False)
        return S.max_tilt
    finally:
        mujoco.MjSpec.from_file = _orig; ga.SUBSTEPS = old


def test_the_robot_walks_over_the_carpet_edge_with_the_imperfect_arm():
    """at a 5 ms physics step the flexing arm shook and the robot fell stepping off the carpet (27 Sep); every sim
    now steps at params sim.physics_dt_s through go1_arm.step"""
    import go1_arm as ga
    assert _walk_over_a_step(ga.SUBSTEPS) < 10.0
