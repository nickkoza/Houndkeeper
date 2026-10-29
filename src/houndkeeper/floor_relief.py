"""The floor's relief ([4t7], [JzW]): how far each patch of floor stands above the house's lowest floor (a carpet's
pile and pad, a rug on it), so a camera's floor pixels are laid on the floor where it is, not on one flat plane. A
camera 0.30 m up that takes a carpet 12 mm proud for the plane under it lays the carpet 4% too far out, 4 cm at 1 m:
the opposite way to the head camera's mount error, and as large (sim, 6 Oct 2026).

A relief is a grid on the floor grid (the confirmer's: RES m cells, north up, centred), metres, 0 where the floor is at
its level. It is solved from the robot's own views of the floor with the camera's geometry (sim/house/camgeo.py
selfcal: each 0.6 m tile's height from how its picture moves with range).

Since [yFg] (7 Oct 2026) the relief is part of each house's map: the stitch solves it in the house's frame and stores it
in the run directory (RELIEF_FILE, with RELIEF_META naming the frame: the pose corrections it was painted in). Every
painter takes it by default (--relief auto) when it paints in that frame; with none, or painting in another frame
(a night's own picture in odometry's frame), it lays the floor on the plane z = 0 and says so."""
from __future__ import annotations

import numpy as np

RES, BOX = 0.01, (8.4, 4.9)        # the floor grid (detect_changes2's): cell m, half extents (x, y) m


def at(relief, X, Y, res=RES, box=BOX):
    """the relief's height (m) at world points X, Y (arrays; 0 off the grid or with no relief)"""
    if relief is None: return np.zeros(np.shape(X), np.float32)
    HX, HY = box; r = np.floor((HY - np.asarray(Y)) / res).astype(int); c = np.floor((np.asarray(X) + HX) / res).astype(int)
    ok = (r >= 0) & (r < relief.shape[0]) & (c >= 0) & (c < relief.shape[1])
    out = np.zeros(np.shape(X), np.float32); out[ok] = relief[r[ok], c[ok]]
    return out


def load(path):
    """a saved relief (.npy), or None for no path"""
    return None if not path else np.load(path).astype(np.float32)


RELIEF_FILE, RELIEF_META = "relief.npy", "relief.json"


def _sha(path):
    import hashlib, os
    return hashlib.sha256(open(path, "rb").read()).hexdigest()[:12] if path and os.path.exists(path) else ""


def save(run_dir, grid, frame_files=(), **meta):
    """the house's relief into its run directory ([yFg]): the grid (RELIEF_FILE) and what it is (RELIEF_META): the
    frame it lies in (each pose-corrections file's hash, frame_files), and whatever else is given. Returns the path"""
    import json, time
    from pathlib import Path
    d = Path(run_dir); p = d / RELIEF_FILE
    np.save(p, np.asarray(grid, np.float32))
    m = dict(meta, made=time.strftime("%Y-%m-%d %H:%M"), frame={Path(f).name: _sha(f) for f in frame_files},
             relief_sha=_sha(p), raised_cells=int((np.asarray(grid) > 0).sum()))
    (d / RELIEF_META).write_text(json.dumps(m, indent=1))
    return p


def resolve(spec, run_dir=None, corrections=None, log=print):
    """the relief a painter lays the floor on ([yFg]): spec 'auto' (default) the run's own (RELIEF_FILE in run_dir)
    when the painter paints in its frame (corrections: the pose-corrections file it paints with, which must be one the
    relief names; a confirmer judging on the run's grid passes none and takes it), 'none' the plane, or a path.
    Returns (grid or None, its path or None, what was done); every fall back to the plane is said through log"""
    import json
    from pathlib import Path
    if spec in (None, "", "none"): return None, None, "the plane z = 0 (no relief, as asked)"
    if spec != "auto":
        return load(spec), str(spec), f"the relief given ({spec})"
    p = Path(run_dir) / RELIEF_FILE if run_dir else None
    if p is None or not p.exists():
        note = f"no floor relief in {run_dir}: the floor laid on the plane z = 0"; log(f"warning: {note}"); return None, None, note
    if corrections is not None:
        mp = Path(run_dir) / RELIEF_META
        frame = (json.loads(mp.read_text()).get("frame") or {}) if mp.exists() else {}
        if _sha(corrections) not in frame.values():
            note = (f"the house's relief ({p}) lies in another frame than {corrections}: the floor laid on the plane z = 0")
            log(f"warning: {note}"); return None, None, note
    return load(p), str(p), f"the house's relief ({p})"


def resolve_odometry(spec, run_dir=None, log=print):
    """a painter in odometry's own frame (no corrections): the house's relief lies in the house's frame, so 'auto' lays
    the floor on the plane (said once); a path given is taken"""
    if spec == "auto":
        note = "painting in odometry's frame: the house's relief (its frame) not used, the floor laid on the plane z = 0"
        log(note); return None, None, note
    return resolve(spec, run_dir, None, log)
