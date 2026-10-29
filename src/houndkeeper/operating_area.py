"""The operating area ([JVr]): where he may go, drawn on the house's map and kept with it, in the map's frame (the house's
frame: the floor pictures' and the lidar map's). One or more allowed polygons and optional virtual walls (segments, e.g.
across a doorway). DimOS's costmap layer (dimos.mapping.keepout_zones, decision [u4W]) marks everything else lethal; this
module stores the area, places it in the frame the costmap is built in (the robot's odometry, "world") and answers the
tidy's own questions: is a point inside, where is the nearest way back in.

The file, <map dir>/operating_area.json:
    {"frame_id": "house", "allowed": [[[x, y], ...], ...], "walls": [[[x1, y1], [x2, y2]], ...],
     "changed": "<ISO time>", "by": "<who or what wrote it>"}
Every write is confirmed by its caller first and appended, whole, to operating_area.log.jsonl beside it (as a
calibration write is to calibration-runs.jsonl), so each change is kept with its time ([daF]).

It is not a safety barrier ([HZO]): it holds only while localization does. Stairs, drops and anything dangerous stay
behind a physical barrier."""
from __future__ import annotations

import json, os, time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from dimos.mapping.keepout_zones import KeepoutZones

HK = Path(__file__).resolve().parents[2]
FILE = "operating_area.json"
LOG = "operating_area.log.jsonl"
FRAME = "house"
REAL_MAP_DIR = HK / "real" / "local" / "house"   # the real house's map and its area: kept off the repo (gitignored)
INSET_M = 0.35          # a point the robot is sent to is this far inside the edge: the costmap's soft margin (0.3 m) and a
                        # little more, so the planner finds a goal it can stand on
WARNING = ("The operating area is not a safety barrier: it holds only while he knows where he is. Keep stairs, drops "
           "and anything dangerous behind a physical barrier, such as a gate.")


@dataclass
class Area:
    """the operating area in the house's frame"""
    allowed: list = field(default_factory=list)       # polygons, each [[x, y], ...]
    walls: list = field(default_factory=list)         # segments, each [[x1, y1], [x2, y2]]
    changed: str = ""
    by: str = ""

    def zones(self) -> KeepoutZones:                  # [rLQ]
        return KeepoutZones(FRAME, allowed=self.allowed, walls=self.walls)

    def to_dict(self) -> dict:
        z = self.zones()
        return {"frame_id": FRAME, "allowed": [np.round(p, 3).tolist() for p in z.allowed],
                "walls": [np.round(s, 3).tolist() for s in z.walls], "changed": self.changed, "by": self.by}


def map_dir(run_dir: str | Path | None = None) -> Path:
    """where the map and its area live: the sim house's run directory, else HK_MAP_DIR, else the real house's"""
    return Path(run_dir) if run_dir else Path(os.environ.get("HK_MAP_DIR") or REAL_MAP_DIR)


def validate(d: Any, dock: tuple[float, float] | None = None) -> Area:
    """an Area from a dict (a file's, or the cockpit's), or ValueError saying what is wrong: at least one polygon of 3+
    points with some area, finite numbers, and the dock inside when it is known ([OrO])"""
    if not isinstance(d, dict): raise ValueError("the area must be an object")
    if d.get("frame_id", FRAME) != FRAME: raise ValueError(f"the area must be in the house's frame ({FRAME!r})")
    allowed, walls = d.get("allowed") or [], d.get("walls") or []
    if not allowed: raise ValueError("the area needs at least one polygon")
    try: z = KeepoutZones(FRAME, allowed=allowed, walls=walls)
    except (ValueError, TypeError) as e: raise ValueError(str(e)) from None
    for p in [*z.allowed, *z.walls]:
        if not np.isfinite(p).all(): raise ValueError("the area has a coordinate that is not a number")
        if np.abs(p).max() > 1000: raise ValueError("the area has a coordinate over 1 km away")
    for p in z.allowed:
        x, y = p[:, 0], p[:, 1]
        if abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2 < 0.25:
            raise ValueError("a polygon covers less than 0.25 square metres")
    if dock is not None and not z.contains(*dock[:2]):
        raise ValueError("the dock is outside the area: he could not start or end a night inside it")
    return Area(allowed=[p.tolist() for p in z.allowed], walls=[s.tolist() for s in z.walls],
                changed=str(d.get("changed", "")), by=str(d.get("by", "")))


def load(path: str | Path | None) -> Area | None:
    """the area stored at path (a file, or a map directory holding one); None if there is none. A file that is there
    but unreadable raises: a broken area is not the same as no area"""
    if not path: return None
    p = Path(path); p = p / FILE if p.is_dir() else p
    if not p.exists(): return None
    return validate(json.loads(p.read_text()))


def save(path: str | Path, d: dict, by: str, dock: tuple[float, float] | None = None, now: float | None = None) -> Area:
    """validate, then write the area (atomically) and append it to the log with its time. The caller has had the user
    confirm it ([daF])"""
    p = Path(path); p = p / FILE if p.is_dir() or not p.suffix else p
    a = validate(d, dock)
    a.changed = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(now if now is not None else time.time())); a.by = by
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".tmp"); tmp.write_text(json.dumps(a.to_dict(), indent=1)); os.replace(tmp, p)
    with (p.parent / LOG).open("a") as fh: fh.write(json.dumps(a.to_dict()) + "\n")
    return a


def zones_json(area: Area, house_from_world: np.ndarray | None = None) -> str:
    """the area as the costmap layer's JSON, placed in the odometry's frame ("world") by the localization's correction
    (3x3, world -> house; None: the frames agree, as before any correction) ([rLQ], [mvn]: the area alone, no sensor
    data)"""
    C = np.eye(3) if house_from_world is None else np.asarray(house_from_world, float)
    return area.zones().transformed(np.linalg.inv(C), "world").to_json()


def inside(area: Area, xy, inset: float = 0.0) -> bool:
    """whether a point (house frame) is in the area, at least `inset` m from its edges and walls"""
    z = area.zones(); x, y = float(xy[0]), float(xy[1])
    if not z.contains(x, y): return False
    return inset <= 0 or _edge_distance(z, np.array([[x, y]]))[0] >= inset


def _edge_distance(z: KeepoutZones, P: np.ndarray) -> np.ndarray:
    from dimos.mapping.keepout_zones import _segment_distance
    segs = [np.array([p[i], p[(i + 1) % len(p)]]) for p in z.allowed for i in range(len(p))] + list(z.walls)
    d = np.full(len(P), np.inf)
    for s in segs: d = np.minimum(d, _segment_distance(P[:, 0], P[:, 1], s))
    return d


def nearest_inside(area: Area, xy, inset: float = INSET_M, res: float = 0.05) -> tuple[float, float] | None:
    """the nearest point to xy (house frame) that is inside the area and `inset` m clear of its edges and walls ([Qd5]:
    the shortest way back in); None if the area has no such point"""
    z = area.zones(); allp = np.vstack(z.allowed)
    lo, hi = allp.min(0), allp.max(0)
    xs, ys = np.arange(lo[0], hi[0] + res, res), np.arange(lo[1], hi[1] + res, res)
    X, Y = np.meshgrid(xs, ys); P = np.c_[X.ravel(), Y.ravel()]
    from dimos.mapping.keepout_zones import _inside
    ok = np.zeros(len(P), bool)
    for p in z.allowed: ok |= _inside(P, p)
    ok &= _edge_distance(z, P) >= inset
    if not ok.any(): return None
    P = P[ok]; i = int(np.argmin(np.hypot(P[:, 0] - xy[0], P[:, 1] - xy[1])))
    return float(P[i, 0]), float(P[i, 1])


def keep_inside(area: Area | None, pts, inset: float = 0.0) -> tuple[list, list]:
    """points (house frame) split into those inside the area and those not ([OrO]); with no area, all are inside"""
    if area is None: return list(pts), []
    ins, out = [], []
    for p in pts: (ins if inside(area, p, inset) else out).append(p)
    return ins, out
