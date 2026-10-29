"""[cqG] [qaS]: the held item's weighed mass rides the arm's gravity feed-forward from the weighing until the item is
released, and is cleared on every way out, in both sims (the standalone Sim directly, DimOS's through a per-launch
payload file the sim process reads). Static poses only: mj_forward, never a physics step."""
import sys
import types
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "arm")); sys.path.insert(0, str(HK / "src"))
mujoco = pytest.importorskip("mujoco")

WEIGHED_KG = 0.4
POSES = [([0.0, -0.6, 1.0, 0.3, 0.0, 0.0], (0.0, 0.0)), ([0.5, -1.2, 0.8, 0.5, 0.3, 0.2], (0.25, 0.0))]


OURS = ("go1_arm", "pick", "vslice", "env")      # the modules this change touches, as this checkout has them


@pytest.fixture(scope="module")
def ours():
    """this checkout's go1_arm, pick, vslice and sim/dimos env, whatever the suite imported before: the sim's modules
    put the main checkout's sim/arm first on the path, so an earlier test may have loaded its copies. Swapped in for
    this module's tests and back out after"""
    import importlib
    saved = {n: sys.modules.pop(n) for n in OURS if n in sys.modules}
    try:
        mods = {}
        for name, where in (("go1_arm", "sim/arm"), ("pick", "sim/arm"), ("vslice", "sim/arm"), ("env", "sim/dimos")):
            sys.path.insert(0, str(HK / where))         # again each time: importing one can put the main checkout first
            mods[name] = importlib.import_module(name)
            assert Path(mods[name].__file__).resolve().parent == HK / where, mods[name].__file__
        yield types.SimpleNamespace(**mods)
    finally:
        for n in OURS: sys.modules.pop(n, None)
        sys.modules.update(saved)


@pytest.fixture(scope="module")
def sim(ours):
    ga = ours.go1_arm
    import payload_test as pt
    if not ga.GRAVITY_PY.exists() or not ga.SDK_URDF.exists():
        pytest.skip("needs the maker-arm driver worktree and the SDK URDF")
    m = ga.build(True, pt.STOW, pick=True)
    d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, 0)
    st = ga.gravity_ff(m)
    qa = [m.jnt_qposadr[m.joint("arm_" + j).id] for j in ga.FF_JOINTS]
    dof = [m.jnt_dofadr[m.joint("arm_" + j).id] for j in ga.FF_JOINTS]
    yield ga, m, d, st, qa, dof
    ga.set_payload(m, 0.0)


def _pose(m, d, qa, st, q, tilt=(0.0, 0.0)):
    """the arm commanded and standing at q, the trunk tilted (roll, pitch) in radians"""
    r, p = tilt
    qr = np.array([np.cos(r / 2), np.sin(r / 2), 0, 0]); qp = np.array([np.cos(p / 2), 0, np.sin(p / 2), 0])
    qt = np.zeros(4); mujoco.mju_mulQuat(qt, qr, qp); d.qpos[3:7] = qt
    d.qpos[qa] = q; d.qvel[:] = 0; d.ctrl[st["act"]] = q
    mujoco.mj_forward(m, d)


def _held_torque(m, d, st, dof, kg):
    """a point mass kg at the feed-forward model's grasp point, held against gravity: J^T m g, MuJoCo's own Jacobian"""
    l7 = m.body("arm_link_007").id
    point = d.xpos[l7] + d.xmat[l7].reshape(3, 3) @ np.array(st["model"].tip)
    jacp = np.zeros((3, m.nv)); mujoco.mj_jac(m, d, jacp, None, point, l7)
    return jacp[:, dof].T @ (kg * -np.asarray([0.0, 0.0, -9.81]))


def test_the_estimate_error_is_stated_and_applied(ours):
    """[qaS] [x3f]: the payload is the weighed mass x (1 + maker_arm.payload_estimate_error), an estimate with its
    source, taken short; never below zero"""
    pick = ours.pick
    import yaml
    entry = yaml.safe_load((HK / "params" / "robot.yaml").read_text())["maker_arm"]["payload_estimate_error"]
    assert entry["source"] == "estimate" and entry["note"] and -0.5 < entry["value"] < 0
    assert pick.payload_kg(WEIGHED_KG) == pytest.approx(WEIGHED_KG * (1 + entry["value"]))
    assert pick.payload_kg(-0.05) == 0.0


def test_the_payload_raises_the_feed_forward_by_the_held_weight(sim, ours):
    """[cqG] [qaS]: with the weighed item set, each joint's feed-forward rises by the torque that holds its estimated
    mass at the grasp point (upright and tilted, no joint at its cap), and clearing it gives the unloaded torques back"""
    ga, m, d, st, qa, dof = sim
    pick, vslice = ours.pick, ours.vslice
    cap = np.array([ga.gravity_module().RATED_TORQUE_NM[k] for k in ga.FF_MOTORS])
    S = types.SimpleNamespace(m=m); S.set_payload = lambda kg: vslice.Sim.set_payload(S, kg)   # the standalone Sim's own
    kg = pick.payload_kg(WEIGHED_KG)
    for q, tilt in POSES:
        _pose(m, d, qa, st, q, tilt)
        pick.set_payload(S, 0.0); bare = ga.update_gravity_ff(m, d).copy()
        pick.set_payload(S, kg); loaded = ga.update_gravity_ff(m, d).copy()
        assert np.all(np.abs(loaded) < cap - 1e-6), (q, loaded)
        assert np.allclose(m.actuator_biasprm[st["act"], 0], loaded)
        want = _held_torque(m, d, st, dof, kg)
        assert np.abs(want).max() > 0.3                                        # the item weighs on the arm here
        assert np.allclose(loaded - bare, want, atol=2e-3), (q, loaded - bare, want)
        pick.set_payload(S, 0.0)
        assert np.allclose(ga.update_gravity_ff(m, d), bare, atol=1e-12)


def test_every_way_out_of_the_pick_clears_the_payload(sim, ours, monkeypatch):
    """[qaS]: once weighed the payload is set; however the carry ends (released, a failure returned, an error raised)
    the feed-forward is back to nothing held"""
    ga, m, d, st, qa, dof = sim
    pick = ours.pick
    monkeypatch.setenv("HK_PICK_RECORD", "0")
    S = types.SimpleNamespace(m=m, set_payload=lambda kg: ga.set_payload(m, kg))
    seen = []

    def lifted_then(outcome):
        def fake(S_, ref, q_ready, q_bin, rec=None):
            pick.set_payload(S_, pick.payload_kg(WEIGHED_KG)); seen.append(st["ff"].payload_kg)
            if isinstance(outcome, Exception): raise outcome
            return outcome, 0.01
        return fake

    for outcome in (None, "lost on the way to the bin (may be in it)", RuntimeError("the arm faulted")):
        monkeypatch.setattr(pick, "_grasp_to_bin", lifted_then(outcome))
        if isinstance(outcome, Exception):
            with pytest.raises(RuntimeError): pick.grasp_to_bin(S, {"label": "toy"}, None, None)
        else:
            assert pick.grasp_to_bin(S, {"label": "toy"}, None, None)[0] == outcome
        assert seen[-1] == pytest.approx(pick.payload_kg(WEIGHED_KG)) and st["ff"].payload_kg == 0.0

    def faulting_carry(*a, **k): raise RuntimeError("the arm faulted")      # the policy pick's carry to the bin
    S.gripper = lambda: 0.01; S.over_bin = lambda hang: None; S.state = lambda name: None; S.arm_to_safe = faulting_carry
    ga.set_payload(m, 0.3)
    with pytest.raises(RuntimeError): pick.carry_to_bin(S, None, None)
    assert st["ff"].payload_kg == 0.0


def test_the_payload_file_round_trip(sim, tmp_path, monkeypatch):
    """[qaS]: DimOS's sim reads the held mass from the pick's per-launch file: what DimosRobot.set_payload writes,
    sim_hook's per-frame read sets on the feed-forward; a cleared or missing file is nothing held"""
    ga, m, d, st, qa, dof = sim
    from houndkeeper import dimos_robot
    assert Path(dimos_robot.__file__).resolve() == HK / "src" / "houndkeeper" / "dimos_robot.py", dimos_robot.__file__
    DimosRobot = dimos_robot.DimosRobot
    path = tmp_path / "houndkeeper_payload_test.json"
    monkeypatch.setenv(ga.PAYLOAD_ENV, str(path))
    R = object.__new__(DimosRobot)                                            # no DimOS connection: set_payload only
    assert ga.payload_from_file(m) == 0.0                                    # no file yet: nothing held
    R.set_payload(0.3)
    assert ga.payload_from_file(m) == pytest.approx(0.3) and st["ff"].payload_kg == pytest.approx(0.3)
    assert ga.payload_from_file(m) == pytest.approx(0.3)                    # unchanged: read once
    R.set_payload(0.0)
    assert ga.payload_from_file(m) == 0.0 and st["ff"].payload_kg == 0.0
    path.write_text("{half a fil"); assert ga.payload_from_file(m) == 0.0   # never half a reading
    monkeypatch.delenv(ga.PAYLOAD_ENV)
    ga.set_payload(m, 0.2); assert ga.payload_from_file(m) is None and st["ff"].payload_kg == pytest.approx(0.2)
    ga.set_payload(m, 0.0)


def test_each_dimos_launch_names_its_own_payload_file(ours, monkeypatch):
    """[qaS]: the payload file is named per launch, as the clock is, and reaches every process apply() sets up"""
    import os
    env = ours.env
    saved = dict(os.environ)
    try:
        os.environ.pop("HK_SIM_PAYLOAD", None)
        monkeypatch.setattr(env, "display_awake", lambda: False)
        names = env.apply()
        assert "HK_SIM_PAYLOAD" in names
        assert os.environ["HK_SIM_PAYLOAD"] == f"/dev/shm/houndkeeper_payload_{os.getpid()}.json"
        assert "export HK_SIM_PAYLOAD=" in env.shell_exports()
    finally:
        os.environ.clear(); os.environ.update(saved)
