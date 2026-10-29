"""The arm is drawn as the real one looks: a black 3D-printed structure (waymark 0008 [ipj])."""
import sys
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))


def test_the_printed_arm_is_matte_black():
    import go1_arm
    arm = mujoco.MjSpec.from_file(str(go1_arm.HERE / "maker_arm.xml")); go1_arm.paint_arm(arm)
    m = arm.compile(); drawn = [g for g in range(m.ngeom) if m.geom_rgba[g, 3] > 0]
    assert drawn and all(m.geom_rgba[g, :3].max() < 0.1 for g in drawn)                 # black, not the default grey
    mats = {int(m.geom_matid[g]) for g in drawn}; assert -1 not in mats
    assert all(m.mat_metallic[k] == 0 and 0.4 <= m.mat_roughness[k] <= 0.8 for k in mats)  # printed plastic: matte, not metal


def test_the_jaws_are_black_too_and_by_default():
    import os, go1_arm
    assert go1_arm.ARM_BLACK or os.environ.get("HK_ARM_BLACK") == "0"         # every sim, not only the rounds (29 Sep)
    arm = mujoco.MjSpec.from_file(str(go1_arm.HERE / "maker_arm.xml")); go1_arm.add_jaws(arm); go1_arm.paint_arm(arm)
    m = arm.compile(); grip = {m.body(f"gripper_{s_}").id for s_ in ("left", "right")}
    jaw = [g for g in range(m.ngeom) if m.geom_bodyid[g] in grip and m.geom_rgba[g, 3] > 0]
    assert jaw and all(m.geom_rgba[g, :3].max() < 0.1 for g in jaw)           # the arm and jaw are black
