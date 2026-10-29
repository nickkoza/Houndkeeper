"""The robot placed in the house's frame while it walks ([KkQ]; [nAH] 2Ak): the floor it judges was pictured on earlier
nights, each placed against the house's lidar map (floor_mosaic2 --house), while tonight's odometry has drifted 3-15 cm
and 1-3 deg by a walk's end (27 Sep, bench real1). Each lidar scan is matched to the house map as it arrives (the same
Localizer, one scan at a time) and the floor comparison runs in the house's frame; the robot's own walking stays in its
odometry's frame, which is what the navigator plans in. HouseFramed is the seam: frames go in moved to the house's
frame, and what the comparison says comes out moved back."""
from __future__ import annotations

import math
import threading

import numpy as np

from houndkeeper.localize import Localizer


class LiveLocalizer:
    """the Localizer fed live: each scan matched in a worker thread (ICP takes a good part of a second); a scan that
    arrives while one is being matched waits, and an older waiting one is dropped (the newest is what matters)"""

    def __init__(self, house: np.ndarray):
        self.loc = Localizer(np.asarray(house)); self.travelled = 0.0; self._xy = None
        self._t: list = []; self._T: list = []; self._lock = threading.Lock(); self._cv = threading.Condition(); self._next = None
        self._stop = False; threading.Thread(target=self._run, daemon=True).start()

    def odom(self, xy, **_heading_and_time) -> None:
        """the odometry's position, for how far the robot has walked (how far its drift may have grown); its heading
        and time, given for the live graph (live_place.LivePlacer), are not needed here"""
        xy = np.asarray(xy, float)
        if self._xy is not None: self.travelled += float(np.hypot(*(xy - self._xy)))
        self._xy = xy

    def scan(self, t: float, pts: np.ndarray) -> None:
        from houndkeeper.localize import real_scan
        if not real_scan(len(pts), t, getattr(self, "_t_last", None)): return
        self._t_last = t
        with self._cv:
            self._next = (t, pts, self.travelled, None if self._xy is None else self._xy.copy()); self._cv.notify()

    def close(self) -> None:
        with self._cv: self._stop = True; self._cv.notify()

    def _run(self) -> None:
        while True:
            with self._cv:
                while self._next is None and not self._stop: self._cv.wait()
                if self._stop: return
                job, self._next = self._next, None
            T = self.loc.step(*job)
            with self._lock: self._t.append(job[0]); self._T.append(T.copy())

    def correction(self, t: float | None = None) -> np.ndarray:
        """the 3x3 SE(2) believed -> house at time t (None: the latest)"""
        with self._lock:
            if not self._t: return np.eye(3)
            return self._T[-1] if t is None else Localizer(times=list(self._t), Ts=list(self._T)).correction(t)


def _xy(C: np.ndarray, x: float, y: float) -> tuple[float, float]:
    p = C[:2, :2] @ np.array([x, y]) + C[:2, 2]; return float(p[0]), float(p[1])


class HouseFramed:
    """a Confirmer judging in the house's frame, used as if it judged in the odometry's: frames in, spots and discs out"""

    def __init__(self, cf, live, moves_frames: bool = True):
        """live: anything with correction(t=None) -> 3x3 (a LiveLocalizer, or FloorTrack); moves_frames: False when the
        Confirmer already places its frames by that correction itself (floor tracking, --track)"""
        object.__setattr__(self, "cf", cf); object.__setattr__(self, "live", live); object.__setattr__(self, "moves", moves_frames)

    def __getattr__(self, name): return getattr(self.cf, name)
    def __setattr__(self, name, value): setattr(self.cf, name, value)

    def frame(self, img, T, *args, ts: float | None = None, **kw):
        if not self.moves: return self.cf.frame(img, T, *args, **kw)
        C = self.live.correction(ts); M = np.eye(4); M[:2, :2] = C[:2, :2]; M[:2, 3] = C[:2, 2]
        if hasattr(self.live, "frame_context") and ts is not None: self.live.frame_context(ts, C)   # [x9X]: for its floor lock
        return self.cf.frame(img, M @ T, *args, **kw)

    def decide(self, t, robot_xy):
        C = self.live.correction(); Ci = np.linalg.inv(C); out = []
        for rec in self.cf.decide(t, _xy(C, *robot_xy)):
            rec["x_house"], rec["y_house"] = rec["x"], rec["y"]; rec["x"], rec["y"] = (round(v, 3) for v in _xy(Ci, rec["x"], rec["y"]))
            if rec.get("cells"): rec["cells"] = [[round(v, 2) for v in _xy(Ci, *c)] for c in rec["cells"]]
            out.append(rec)
        return out

    def keepout(self, *a, **kw):
        Ci = np.linalg.inv(self.live.correction())
        return [(*_xy(Ci, x, y), r) for x, y, r in self.cf.keepout(*a, **kw)]

    def to_house(self, xy) -> tuple[float, float]:
        return _xy(self.live.correction(), *xy)

    def house_points(self, pts: np.ndarray) -> np.ndarray:
        """points in the odometry's frame (tonight's live map), in the house's"""
        C = self.live.correction(); P = np.asarray(pts, float).copy(); P[:, :2] = P[:, :2] @ C[:2, :2].T + C[:2, 2]; return P


class FloorTrack:
    """the Confirmer's own floor tracking (confirm_stream --track, [fXD]) as a correction: believed -> the earlier
    nights' floor, kept up by each frame that locks onto it"""

    def __init__(self, cf): self.cf = cf
    def correction(self, t: float | None = None) -> np.ndarray: return self.cf.Cf
