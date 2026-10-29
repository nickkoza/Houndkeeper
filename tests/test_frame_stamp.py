"""[FHA]: the sim stamps each camera frame with the time of the physics state it shows, so its pose is looked up for the
right moment. Stamped when DimOS polled it, a frame was up to a polling interval (50 ms at 20 fps) late: centimetres
of floor on a walking robot (26 Sep 2026)."""
import subprocess
from pathlib import Path

from houndkeeper import workspace

DIMOS = workspace.dimos()


def test_a_frame_carries_the_time_of_its_state():
    code = ("import numpy as np, dimos.simulation.mujoco.shared_memory as sm\n"
            "from dimos.simulation.mujoco.constants import VIDEO_WIDTH as W, VIDEO_HEIGHT as H\n"
            "w = sm.ShmWriter()\n"
            "try:\n"
            "    sim = sm.ShmReader(w.shm.to_names())   # the sim's side writes frames; DimOS's reads them\n"
            "    sim.write_video(np.full((H, W, 3), 7, np.uint8), 1234.25); sim.write_arm_video(np.zeros((H, W, 3), np.uint8), 99.5)\n"
            "    f, seq = w.read_video(); print(w.video_ts(), w.arm_video_ts(), int(f[0, 0, 0]))\n"
            "finally:\n"
            "    w.cleanup()\n")
    out = subprocess.run([str(DIMOS / ".venv" / "bin" / "python"), "-c", code], capture_output=True, text=True, cwd=DIMOS, timeout=120)
    assert out.stdout.strip().splitlines()[-1] == "1234.25 99.5 7", out.stdout + out.stderr[-800:]


def test_the_connection_turns_frames_into_images_stamped_with_their_state():
    """through DimOS's own code path: a helper wrapped by the wrong decorator once broke both video streams, and the night
    'succeeded' with no frames at all (26 Sep 2026)"""
    code = ("import numpy as np, dimos.simulation.mujoco.shared_memory as sm\n"
            "from dimos.robot.unitree.mujoco_connection import MujocoConnection as MC\n"
            "from dimos.simulation.mujoco.constants import VIDEO_WIDTH as W, VIDEO_HEIGHT as H\n"
            "w = sm.ShmWriter()\n"
            "try:\n"
            "    sm.ShmReader(w.shm.to_names()).write_video(np.full((H, W, 3), 9, np.uint8), 555.5)\n"
            "    f, _ = w.read_video(); print(MC._stamp(w.video_ts, f, w.read_video), int(f[0, 0, 0]))\n"
            "finally:\n"
            "    w.cleanup()\n")
    out = subprocess.run([str(DIMOS / ".venv" / "bin" / "python"), "-c", code], capture_output=True, text=True, cwd=DIMOS, timeout=120)
    assert out.stdout.strip().splitlines()[-1] == "555.5 9", out.stdout + out.stderr[-800:]
