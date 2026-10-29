"""Where the robot was at any sample's own time in a recording ([Feg]): DimOS's recording-backed transform lookup
(dimos.memory.tf.StreamTF), interpolated between the poses either side of the time asked (linear in position, slerp in
rotation) and never across a gap wider than gap_s, nor past the first or last pose. The one place the offline tools
read the robot's and its head camera's poses from a memory.db; on the robot, DimOS's live buffer answers the same
question the same way (tidy_module._head_camera)."""
from __future__ import annotations

import numpy as np

POSE_GAP_S = 0.2   # the widest gap between two poses a time is interpolated across (s, the recording's clock): the
                   # odometry runs at 10-30 Hz, so a wider one is a dropout, not a step


class RecordedPoses:
    """pose(t): world <- base_link (4x4) at time t, or None outside the poses or across a gap; mount(): the head
    camera's base_link <- camera_link and camera_link <- camera_optical as first recorded"""

    def __init__(self, store, gap_s: float = POSE_GAP_S, world: str = "world", base: str = "base_link") -> None:
        from dimos.memory.tf import StreamTF
        self.tf = StreamTF.from_store(store)
        if self.tf is None: raise ValueError("the recording has no tf stream")
        self.gap_s, self.world, self.base = float(gap_s), world, base

    def at(self, parent: str, child: str, t: float) -> np.ndarray | None:
        """parent <- child at time t (4x4), interpolated; None outside the samples or across a gap"""
        tr = self.tf.get(parent, child, float(t), self.gap_s, interpolate=True, max_gap=self.gap_s, warn=False)
        return None if tr is None else tr.to_matrix()

    def __call__(self, t: float) -> np.ndarray | None:
        return self.at(self.world, self.base, t)

    def samples(self, parent: str | None = None, child: str | None = None) -> tuple[np.ndarray, np.ndarray]:
        """every recorded parent <- child (default world <- base) as (times, 4x4s), in time order, one per time: for
        rates and speeds over the night, never for a pose at a time (that is pose(t))"""
        parent, child = parent or self.world, child or self.base
        ts, Ts = [], []
        for o in self.tf.stream:
            for tr in o.data.transforms:
                if (tr.frame_id, tr.child_frame_id) == (parent, child): ts.append(tr.ts or o.ts); Ts.append(tr.to_matrix())
        if not ts: return np.zeros(0), np.zeros((0, 4, 4))
        o_ = np.argsort(ts, kind="stable"); t_ = np.asarray(ts)[o_]; T_ = np.asarray(Ts)[o_]; u_ = np.r_[True, np.diff(t_) > 1e-9]
        return t_[u_], T_[u_]

    def first(self, parent: str, child: str) -> np.ndarray | None:
        """parent <- child as first recorded (a mount: fixed on the robot)"""
        for o in self.tf.stream:
            for tr in o.data.transforms:
                if (tr.frame_id, tr.child_frame_id) == (parent, child): return tr.to_matrix()
        return None

    def mount(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        """the head camera on the body: (base_link <- camera_link, camera_link <- camera_optical)"""
        return self.first(self.base, "camera_link"), self.first("camera_link", "camera_optical")
