#!/usr/bin/env python3
"""Persistent 3D map, refined night after night, from DimOS's own per-night global_map.

Each night's map is voxelised (4 cm); a voxel is structure if it's been seen on at least half of the nights
so far (min 1). Per step it reports completeness against the sim's true walls and furniture (share of their
outline that's mapped) and the share of mapped structure that's real (not clutter or noise). Writes a height
map (max structure z per 2 cm cell) for the floor pipeline to use for line-of-sight masking.
Usage: map_model.py <recordings.txt> [--out runs/nights_s7/map_model]"""
import argparse, json
from pathlib import Path
import numpy as np
import make_house as mh
from dimos.memory.store.sqlite import SqliteStore

VOX, HRES, HX, HY = 0.04, 0.02, 8.4, 4.9


def night_voxels(db):
    st = SqliteStore(path=db); st.start()
    pts = list(st.stream("global_map"))[-1].data.as_numpy(); st.stop()
    pts = np.asarray(pts[0] if isinstance(pts, tuple) else pts)
    pts = pts[pts[:, 2] > 0.10]                                   # structure, not floor-level noise
    return {tuple(v) for v in np.floor(pts / VOX).astype(np.int32)}


def truth_boxes():
    g = mh.base_grid(); out = []
    for r in mh.wall_rects(g):
        (ax, ay), (bx, by) = mh.cell_to_world(r[0], r[1]), mh.cell_to_world(r[2], r[3]); out.append((min(ax, bx) - mh.CELL/2, max(ax, bx) + mh.CELL/2, min(ay, by) - mh.CELL/2, max(ay, by) + mh.CELL/2, mh.WALL_H))
    for f in mh.FURNITURE:
        (ax, ay), (bx, by) = mh.cell_to_world(f[0], f[1]), mh.cell_to_world(f[2], f[3]); out.append((min(ax, bx) - mh.CELL/2, max(ax, bx) + mh.CELL/2, min(ay, by) - mh.CELL/2, max(ay, by) + mh.CELL/2, f[4]))
    return out


def score(struct):
    """Completeness: share of the faces the robot could see (inside the house, not against a wall) that
    have a mapped voxel within 4 cm. Real: share of mapped voxels within 4 cm of a true wall or furniture box."""
    boxes = truth_boxes(); xy = {(x, y) for x, y, z in struct}
    near = lambda xx, yy: any((int(np.floor(xx / VOX)) + dx, int(np.floor(yy / VOX)) + dy) in xy for dx in (-1, 0, 1) for dy in (-1, 0, 1))
    inside = lambda xx, yy: abs(xx) < HX - 0.40 and abs(yy) < HY - 0.40
    solid = lambda xx, yy: any(x0 <= xx <= x1 and y0 <= yy <= y1 for x0, x1, y0, y1, _ in boxes)
    hit = tot = 0
    for x0, x1, y0, y1, h in boxes:
        faces = [(xx, y0 - .02) for xx in np.arange(x0, x1, VOX)] + [(xx, y1 + .02) for xx in np.arange(x0, x1, VOX)] + \
                [(x0 - .02, yy) for yy in np.arange(y0, y1, VOX)] + [(x1 + .02, yy) for yy in np.arange(y0, y1, VOX)]
        for xx, yy in faces:
            if not inside(xx, yy) or solid(xx, yy): continue        # face the robot can't see
            tot += 1; hit += near(xx, yy)
    v = np.array(list(struct), float); c = (v[:, :2] + .5) * VOX
    real = np.mean([any(x0 - VOX <= x <= x1 + VOX and y0 - VOX <= y <= y1 + VOX for x0, x1, y0, y1, _ in boxes) for x, y in c]) if len(c) else 0
    return hit / max(tot, 1), float(real)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("recordings"); ap.add_argument("--out", default="runs/nights_s7/map_model"); a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    recs = [l.split() for l in open(a.recordings) if l.strip() and len(l.split()) == 2]
    counts, curve = {}, []
    for i, (n, db) in enumerate(recs, 1):
        solo = night_voxels(db); sc_solo = score(solo)
        for v in solo: counts[v] = counts.get(v, 0) + 1
        need = 1 if i == 1 else 2                                 # confirmed on 2+ nights (noise rarely repeats)
        struct = {v for v, c in counts.items() if c >= need}; sc = score(struct)
        curve.append({"night": int(n), "nights_fused": i, "tonight_alone": {"completeness": round(sc_solo[0], 3), "real": round(sc_solo[1], 3)},
                      "fused": {"completeness": round(sc[0], 3), "real": round(sc[1], 3), "voxels": len(struct)}})
        print(f"after night {n}: tonight alone maps {sc_solo[0]:.0%} of true walls+furniture outline ({sc_solo[1]:.0%} of it real); "
              f"fused map {sc[0]:.0%} ({sc[1]:.0%} real)", flush=True)
        hw, hh = int(2 * HX / HRES), int(2 * HY / HRES); hm = np.zeros((hh, hw), np.float32)
        for x, y, z in struct:
            j, r = int(((x + .5) * VOX + HX) / HRES), int((HY - (y + .5) * VOX) / HRES)
            if 0 <= r < hh and 0 <= j < hw: hm[r, j] = max(hm[r, j], (z + 1) * VOX)
        np.save(out / f"heightmap_after_night{n}.npy", hm)
    (out / "curve.json").write_text(json.dumps(curve, indent=1))
