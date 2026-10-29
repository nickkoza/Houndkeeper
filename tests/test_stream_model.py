"""[UMH]: frames reach the robot's software as they would over Wi-Fi ([x3f]): some lost, now and then a stall."""
import numpy as np

from dimos.simulation.mujoco.stream_model import StreamModel


def test_some_frames_are_lost_and_the_stream_stalls_now_and_then():
    s = StreamModel("drop=0.03,stall_per_min=2,stall_s=0.6,seed=5")
    t = np.arange(0, 600, 0.05); got = np.array([s.delivered(x) for x in t])   # 10 minutes at 20 frames a second
    gaps = np.diff(t[got]); stalls = (gaps > 0.3).sum()
    assert 0.94 < got.mean() < 0.98
    assert 10 < stalls < 35                                                    # ~20 in 10 minutes


def test_without_it_every_frame_arrives():
    s = StreamModel("seed=1")
    assert all(s.delivered(x) for x in np.arange(0, 60, 0.05))
