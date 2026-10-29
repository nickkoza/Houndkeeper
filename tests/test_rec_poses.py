"""[Feg], waymark [uKM] decision [929], decision [x5h]: a pose is looked up at a sample's own time in DimOS's transform
buffer (interpolated between the poses either side, never across a gap or past the ends), offline through
houndkeeper.rec_poses and live through tidy_module._head_camera; no tool keeps a pose interpolation of its own."""
import math
import re
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
pytest.importorskip("dimos.memory.tf")                                       # dimos venv
from dimos.memory.store.memory import MemoryStore  # noqa: E402
from dimos.msgs.geometry_msgs.Quaternion import Quaternion  # noqa: E402
from dimos.msgs.geometry_msgs.Transform import Transform  # noqa: E402
from dimos.msgs.geometry_msgs.Vector3 import Vector3  # noqa: E402
from dimos.msgs.tf2_msgs.TFMessage import TFMessage  # noqa: E402
from dimos.protocol.tf.tf import MultiTBuffer  # noqa: E402

from houndkeeper.rec_poses import POSE_GAP_S, RecordedPoses  # noqa: E402

T0 = 1_790_000_000.0


def _tf(parent, child, x, yaw, ts):
    return Transform(translation=Vector3(x, 0.5 * x, 0.3), rotation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
                     frame_id=parent, child_frame_id=child, ts=ts)


def _night(times, lag=lambda k: 0.05):
    """a recording's tf stream as the Go2 recorder writes it: the pose and the head camera's mount together, each
    message recorded lag(k) after the pose's own time (50 ms by default)"""
    store = MemoryStore(); s = store.stream("tf", TFMessage)
    for k, t in enumerate(times):
        s.append(TFMessage(_tf("world", "base_link", 0.1 * k, 0.05 * k, t), _tf("base_link", "camera_link", 0.25, 0.0, t),
                           _tf("camera_link", "camera_optical", 0.0, 0.0, t)), ts=t + lag(k), pose=None)
    return store


def _reference(times, t):
    """the retired lookup (floor_mosaic2, confirm_stream, approach_views before 4 Oct 2026), for the comparison"""
    from scipy.spatial.transform import Rotation, Slerp
    ts = np.asarray(times); Ts = np.array([_tf("world", "base_link", 0.1 * k, 0.05 * k, x).to_matrix() for k, x in enumerate(times)])
    if t <= ts[0] or t >= ts[-1]: return None
    i = np.searchsorted(ts, t)
    if ts[i] - ts[i - 1] > POSE_GAP_S: return None
    f = (t - ts[i - 1]) / (ts[i] - ts[i - 1]); M = np.eye(4); M[:3, :3] = Slerp(ts, Rotation.from_matrix(Ts[:, :3, :3]))([t]).as_matrix()[0]
    M[:3, 3] = Ts[i - 1][:3, 3] * (1 - f) + Ts[i][:3, 3] * f; return M


def test_pose_at_a_frames_own_time_matches_the_retired_lookup():
    times = list(T0 + np.cumsum(np.r_[0.0, np.full(30, 0.035), 0.5, np.full(30, 0.079)]))   # a 0.5 s dropout midway
    rp = RecordedPoses(_night(times))
    rng = np.random.default_rng(0)
    for t in np.r_[rng.uniform(times[0] - 0.1, times[-1] + 0.1, 300), times[5] + 0.0175]:
        old, new = _reference(times, t), rp(t)
        if old is None: assert new is None, t
        else: assert new is not None and np.abs(new - old).max() < 1e-9, t


def test_robot_clock_recording_stored_on_the_wall_clock():
    """[3Pv]: a sim slower than real time stamps on the robot's clock while an older recorder stored the wall clock,
    so each message is stored seconds, and more as the night goes on, after its own stamp (house 96: 4 s growing to
    413 s). Every pose is still found at its own time, as the retired lookup found it."""
    times = list(T0 + np.cumsum(np.r_[0.0, np.full(1715, 0.035)]))       # 60 s of the robot's clock at 0.13x real time
    rp = RecordedPoses(_night(times, lag=lambda k: 4.0 + (times[k] - T0) * (1 / 0.13 - 1)))
    frames = np.asarray(times[:-1]) + 0.0175
    assert all(rp(t) is not None for t in frames)
    for t in frames[::40]:                                                    # the retired lookup is slow: a sample
        assert np.abs(rp(t) - _reference(times, t)).max() < 1e-9, t


HOUSE96 = ROOT / "data/runs/dimos_tidy/20261003_200750/recording.txt"


def test_house96_walk_every_frame_has_its_pose():
    """[Feg], [3Pv]: house 96's walk (robot clock, stored on the wall clock): RecordedPoses places every frame the
    retired lookup placed (5076 of 5076; 1975 before the lookup went by the stamps)"""
    if not HOUSE96.exists() or not Path(HOUSE96.read_text().strip()).exists(): pytest.skip("house 96's walk is not here")
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.sensor_msgs.Image import Image
    st = SqliteStore(path=HOUSE96.read_text().strip()); st.start()
    try:
        rp = RecordedPoses(st); ts, _ = rp.samples()
        frames = [o.ts for o in st.stream("color_image", Image)]
        old = [t for t in frames if ts[0] < t < ts[-1] and ts[np.searchsorted(ts, t)] - ts[np.searchsorted(ts, t) - 1] <= POSE_GAP_S]
        new = [t for t in old if rp(t) is not None]
        print(f"house 96: {len(new)} of {len(old)} frames placed ({len(frames)} frames)")
        assert len(old) == len(frames) == 5076 and len(new) == len(old)
    finally:
        st.stop()


def test_never_across_a_gap_nor_past_the_ends():
    times = [T0, T0 + 0.1, T0 + 0.5, T0 + 0.6]
    rp = RecordedPoses(_night(times))
    assert rp(T0 + 0.05) is not None
    assert rp(T0 + 0.3) is None                                                  # inside the 0.4 s dropout
    assert rp(T0 - 0.01) is None and rp(T0 + 0.61) is None                       # no extrapolation
    assert RecordedPoses(_night(times), gap_s=0.5)(T0 + 0.3) is not None


def test_interpolated_halfway():
    rp = RecordedPoses(_night([T0, T0 + 0.1]))
    M = rp(T0 + 0.05)
    assert M[0, 3] == pytest.approx(0.05, abs=1e-6) and M[1, 3] == pytest.approx(0.025, abs=1e-6)
    assert math.atan2(M[1, 0], M[0, 0]) == pytest.approx(0.025, abs=1e-6)


def test_mount_and_samples():
    rp = RecordedPoses(_night([T0, T0 + 0.1, T0 + 0.2]))
    bc, co = rp.mount()
    assert bc[0, 3] == pytest.approx(0.25) and np.allclose(co[:3, :3], np.eye(3))
    ts, Ts = rp.samples()
    assert np.allclose(ts - T0, [0.0, 0.1, 0.2]) and Ts.shape == (3, 4, 4) and Ts[2][0, 3] == pytest.approx(0.2)


def test_no_tf_stream_refused():
    with pytest.raises(ValueError):
        RecordedPoses(MemoryStore())


def test_live_head_camera_pose_is_interpolated():
    """tidy_module._head_camera: a frame between two poses gets the pose at its own time; a frame newer than the last
    pose the nearest one, as before"""
    from houndkeeper.tidy_module import TidyArm
    buf = MultiTBuffer()
    for k in range(3):
        buf.receive_transform(_tf("world", "camera_optical", 0.1 * k, 0.0, T0 + 0.1 * k))
    me = SimpleNamespace(tfbuffer=buf, _K=SimpleNamespace(m=SimpleNamespace(camera=lambda name: SimpleNamespace(id=0)), d=None))
    assert TidyArm._head_camera(me, T0 + 0.05)[0, 3] == pytest.approx(0.05, abs=1e-6)
    assert TidyArm._head_camera(me, T0 + 0.25)[0, 3] == pytest.approx(0.2)       # past the last pose: the nearest


RETIRED = re.compile(r"\bSlerp\(|slerp\(\[")                                  # a pose interpolation of our own


def test_no_tool_keeps_its_own_pose_interpolation():
    """the class, not the instance ([x5h]): a new pose lookup goes through DimOS's buffer (houndkeeper.rec_poses
    offline), not another copy"""
    own = [str(p.relative_to(ROOT)) for d in ("src", "sim") for p in (ROOT / d).rglob("*.py")
           if "/runs/" not in str(p) and RETIRED.search(p.read_text(errors="ignore"))]
    assert own == [], own
