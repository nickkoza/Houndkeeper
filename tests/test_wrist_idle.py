"""The wrist camera is rendered at its full rate only while the arm is out of its fold (sim speed, sim-slice [hJ5]): it
cost about half the loop's camera time all night though the robot only looks through it with the arm out (28 Sep:
cameras 63% of the loop, the sim at 0.43x); folded, a frame a second keeps the viewer's inset alive."""
import sys
from pathlib import Path

from houndkeeper import workspace

import numpy as np

sys.path.insert(0, str(workspace.dimos()))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "dimos"))


def test_folded_the_wrist_camera_rests_and_out_it_runs():
    """DimOS's general arm-camera throttle, with our settings ([UvE]: sim/dimos/env.py)"""
    import env
    from dimos.simulation.mujoco.mujoco_process import ArmCameraThrottle
    o = env.SIM_OPTIONS
    wrist_frame_due = ArmCameraThrottle([float(v) for v in o["DIMOS_SIM_ARM_CAMERA_REST_POSE"].split(",")],
                                        float(o["DIMOS_SIM_ARM_CAMERA_IDLE_S"]), float(o["DIMOS_SIM_ARM_CAMERA_REST_TOL"])).due
    fold = np.zeros(6); out = np.array([0.0, -0.8, 1.2, 0.3, 0.0, 0.0]); sag = np.full(6, 0.05)
    assert wrist_frame_due(out, t=10.02, last_t=10.0)                    # out of the fold: every frame
    assert not wrist_frame_due(fold, t=10.5, last_t=10.0)                # folded: not yet a second since the last
    assert wrist_frame_due(fold, t=11.0, last_t=10.0)                    # folded: a frame a second
    assert not wrist_frame_due(sag, t=10.5, last_t=10.0)                 # sagging a little is still folded
