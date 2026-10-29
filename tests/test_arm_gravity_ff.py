"""[cqG]: the sim's arm carries its weight through the DimOS adapter's own gravity feed-forward ([Y2p]: gravity.py, loaded by
file path), on a model built from the SDK's URDF with a stated, seeded error, so it holds a share of the weight and no more
([x3f]). Static poses only: mj_forward, never a physics step."""
import ast
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "arm")); sys.path.insert(0, str(HK / "src"))
mujoco = pytest.importorskip("mujoco")

FLOOR_REACH_Q = [0.0, -2.95, 1.3, 1.2, 0.0, 0.0]   # lying, mount 0.22 m up: the jaws' grasp point 2 cm over the floor, 0.45 m out


@pytest.fixture(scope="module")
def sim():
    import go1_arm as ga
    import payload_test as pt
    if not ga.GRAVITY_PY.exists() or not ga.SDK_URDF.exists():
        pytest.skip("needs the maker-arm driver worktree and the SDK URDF")
    m = ga.build(True, pt.STOW, pick=True)
    d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, 0)
    st = ga.gravity_ff(m)
    qa = [m.jnt_qposadr[m.joint("arm_" + j).id] for j in ga.FF_JOINTS]
    dof = [m.jnt_dofadr[m.joint("arm_" + j).id] for j in ga.FF_JOINTS]
    return ga, m, d, st, qa, dof


def _pose(m, d, qa, q, tilt=(0.0, 0.0)):
    """the arm commanded and standing at q (encoders on target), the trunk tilted (roll, pitch) in radians"""
    r, p = tilt
    qr = np.array([np.cos(r / 2), np.sin(r / 2), 0, 0]); qp = np.array([np.cos(p / 2), 0, np.sin(p / 2), 0])
    qt = np.zeros(4); mujoco.mju_mulQuat(qt, qr, qp); d.qpos[3:7] = qt
    d.qpos[qa] = q; d.qvel[:] = 0
    mujoco.mj_forward(m, d)


def _base_gravity(ga, d, st):
    R = d.xmat[st["trunk"]].reshape(3, 3) @ st["R_mount"]
    return R.T @ ga.G_WORLD


POSES = [([0.0, -0.6, 1.0, 0.3, 0.0, 0.0], (0.0, 0.0)), ([0.5, -1.2, 0.8, 0.5, 0.3, 0.2], (0.25, 0.0)),
         ([-0.4, -2.0, 1.6, -0.8, -0.5, 1.0], (-0.1, 0.3)), (FLOOR_REACH_Q, (0.0, 0.0))]


def test_the_sims_feed_forward_is_the_perturbed_models_torques(sim):
    """[cqG] [Y2p]: what the arm actuators add is gravity.py's GravityFeedForward on the perturbed model, gravity turned
    into the arm base's frame by the trunk's attitude, capped at the adapter's rated torques; MuJoCo's gravcomp is off;
    it rides the bias constant, so the actuator's force is kp (ctrl - q) + ff within its force limit"""
    ga, m, d, st, qa, dof = sim
    assert m.body_gravcomp.max() == 0 and not m.jnt_actgravcomp.any()
    assert all(m.actuator_forcelimited[st["act"]])
    G = ga.gravity_module(); model = ga.gravity_model_perturbed()
    cap = np.array([G.RATED_TORQUE_NM[k] for k in ga.FF_MOTORS])
    true_model = G.GravityModel.from_urdf(ga.SDK_URDF, joints=ga.FF_JOINTS)
    for q, tilt in POSES:
        d.ctrl[st["act"]] = q; _pose(m, d, qa, q, tilt)
        tau = ga.update_gravity_ff(m, d)
        g = _base_gravity(ga, d, st)
        want = np.clip(model.holding_torques(q, g, 0.0, st["ff"].mass_scale), -cap, cap)
        assert np.allclose(tau, want, atol=1e-9) and np.allclose(m.actuator_biasprm[st["act"], 0], want, atol=1e-9)
        # the frame and signs are right: the URDF, scaled to the sim arm's mass, holds exactly MuJoCo's gravity
        held = true_model.holding_torques(q, g, 0.0, st["true_kg"] / true_model.mass)
        assert np.allclose(held, d.qfrc_bias[dof], atol=2e-3), (q, held, d.qfrc_bias[dof])
        mujoco.mj_forward(m, d)                                     # encoders on target: the motor sends ff alone
        assert np.allclose(d.actuator_force[st["act"]], tau, atol=1e-9)


def test_the_model_error_is_seeded_and_nonzero(sim):
    """[cqG] [x3f]: the model's error is stated (params maker_arm.gravity_ff_model_error), seeded (the same model each
    build), and real: its links' masses and centres of mass are off the URDF's; in all it carries gravity_ff_fraction of
    the sim arm's true moving mass"""
    ga, m, d, st, qa, dof = sim
    from houndkeeper import params as P
    err = P.get("maker_arm.gravity_ff_model_error")
    assert err["mass_sd"] > 0 and err["com_sd_m"] > 0
    G = ga.gravity_module(); urdf = G.GravityModel.from_urdf(ga.SDK_URDF, joints=ga.FF_JOINTS)
    a, b = ga.gravity_model_perturbed(), ga.gravity_model_perturbed()
    assert a.links == b.links
    other = ga.gravity_model_perturbed(dict(err, seed=int(err["seed"]) + 1))
    assert other.links != a.links
    dm = np.array([p.mass / u.mass - 1 for p, u in zip(a.links, urdf.links)])
    dc = np.array([np.subtract(p.com, u.com) for p, u in zip(a.links, urdf.links)])
    assert np.abs(dm).max() > 0.05 and np.abs(dc).max() > 0.003          # a model error worth the name
    assert np.abs(dm).max() < 5 * err["mass_sd"] and np.abs(dc).max() < 5 * err["com_sd_m"]
    share = P.get("maker_arm.gravity_ff_fraction")
    assert st["ff"].mass_scale * a.mass == pytest.approx(share * st["true_kg"], rel=1e-9)
    assert st["true_kg"] == pytest.approx(float(m.body_subtreemass[m.body("arm_link_002").id]))


def test_the_static_sag_at_floor_reach_is_within_the_expected_band(sim):
    """[cqG] [x3f]: at floor reach (lying, mount 0.22 m up, grasp point 2 cm over the floor 0.45 m out, nothing held, the
    frame's flex and play at rest), the joints' PD springs hold what the feed-forward misses: a static solve
    q = ctrl - (gravity(q) - ff) / kp by fixed-point iteration on mj_forward. PD alone sags ~0.044 rad at the shoulder,
    ~4 cm at the jaws; with the feed-forward (0.8 share, perturbed) ~0.010 rad, ~0.9 cm. Band: the shoulder keeps 10 to
    40 % of the PD-alone sag (the 20 % share missed, spread by the model's error), the jaws move 0.3 to 2 cm, never zero"""
    ga, m, d, st, qa, dof = sim
    import legs
    q = np.array(FLOOR_REACH_Q); kp = np.array(ga.KP[:6], float)
    mujoco.mj_resetDataKeyframe(m, d, 0); d.qpos[7:19] = legs.LIE
    base = m.body("arm_base_link").id; l7 = m.body("arm_link_007").id; tip = np.array(st["model"].tip)
    _pose(m, d, qa, q); d.qpos[2] += 0.22 - d.xpos[base][2]; _pose(m, d, qa, q)
    grasp = lambda: d.xpos[l7] + d.xmat[l7].reshape(3, 3) @ tip
    p0 = grasp().copy()
    assert 0.015 < p0[2] < 0.025 and abs(p0[0] - d.xpos[base][0] - 0.45) < 0.01      # the pose is what it says
    d.ctrl[st["act"]] = q
    out = {}
    for name, ff in (("pd", np.zeros(6)), ("ff", ga.update_gravity_ff(m, d))):
        qq = q.copy()
        for _ in range(40):
            _pose(m, d, qa, qq); qn = q - (d.qfrc_bias[dof] - ff) / kp
            done = np.abs(qn - qq).max() < 1e-7; qq = qn
            if done: break
        assert done
        _pose(m, d, qa, qq); out[name] = (q - qq, float(np.linalg.norm(grasp() - p0)))
    (sag_pd, move_pd), (sag_ff, move_ff) = out["pd"], out["ff"]
    print(f"floor reach: PD alone shoulder {sag_pd[1]:.4f} rad, jaws {move_pd * 100:.2f} cm; "
          f"with feed-forward {sag_ff[1]:.4f} rad, {move_ff * 100:.2f} cm")
    assert 0.03 < sag_pd[1] < 0.06 and 0.02 < move_pd < 0.06                 # the bare PD's sag, as [Y2p]'s solve found
    assert 0.10 < sag_ff[1] / sag_pd[1] < 0.40
    assert 0.003 < move_ff < 0.02


def test_gravity_py_imports_numpy_and_the_standard_library_only(sim):
    """[Y2p]: the adapter's gravity.py is numpy only (no DimOS import), so the sim can load it by file path"""
    ga = sim[0]
    tree = ast.parse(ga.GRAVITY_PY.read_text())
    mods = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    mods |= {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert not any(x.split(".")[0] == "dimos" for x in mods)
    assert {x.split(".")[0] for x in mods} <= {"__future__", "collections", "dataclasses", "math", "pathlib", "xml", "numpy"}
