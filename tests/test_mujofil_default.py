"""Waymark 0007 [3my]: the build that draws every lamp is the default. It was opt-in (HK_MUJOFIL_DIR), so anything run
without it (the showcase's stills and video, night_tidy.sh, the arm benches) fell back to the installed build, which
draws only one lamp (found again 30 Sep 2026 through the showcase's ceiling fan)."""
import os, subprocess, sys
from pathlib import Path

HK = Path(__file__).resolve().parents[1]


def test_without_a_choice_the_lamp_fixed_build_is_used_and_stamped():
    env = {k: v for k, v in os.environ.items() if k != "HK_MUJOFIL_DIR"}
    code = ("import sys; sys.path.insert(0, 'src'); from houndkeeper import filament_cameras as fc, sim_hook as sh; "
            "fc.use_chosen_mujofil(); import mujofil, os; print(os.path.dirname(os.path.dirname(mujofil.__file__))); "
            "print(sh.render_settings().get('mujofil'))")
    out = subprocess.run([sys.executable, "-c", code], cwd=HK, env=env, capture_output=True, text=True).stdout.split()
    fixed = HK.parent / "build" / "mujofil-f177-site"
    assert out[-2] == str(fixed), out
    assert out[-1] == fixed.name


def test_a_build_that_drops_lamps_is_refused():
    """the check run before the first render: two lamps that each light half the frame must both show; on a build that
    draws only one, rendering stops with an error rather than lighting the house wrongly"""
    sys.path.insert(0, str(HK / "src"))
    from houndkeeper import filament_cameras as fc
    import pytest
    one = lambda halves: (halves[0], halves[0])                                 # what 1.56.3 gives: one lamp's light
    with pytest.raises(RuntimeError, match="3my"): fc._judge_lamps((131.4, 104.3), (104.3, 131.5), (131.5, 104.3), 103.3)
    fc._judge_lamps((131.4, 104.3), (104.3, 131.5), (131.8, 131.8), 103.3)       # both drawn: passes


def test_the_lamp_check_runs_under_a_control_callback_and_leaves_it_in_place():
    """waymark 0007 [3my]: inside DimOS's sim a walking policy is installed as MuJoCo's global control callback; the
    check's own bare scene must not run it (it raised, and every night of the 30 Sep bench died at its first frame)"""
    import pytest
    mujoco = pytest.importorskip("mujoco"); pytest.importorskip("mujofil")
    sys.path.insert(0, str(HK / "src"))
    from houndkeeper import filament_cameras as fc
    def policy(m, d): raise RuntimeError("a walking policy on a scene with no robot")
    mujoco.set_mjcb_control(policy); fc._LAMPS_OK = False
    try:
        fc.check_lamps()
        assert mujoco.get_mjcb_control() is policy
    finally:
        mujoco.set_mjcb_control(None)
