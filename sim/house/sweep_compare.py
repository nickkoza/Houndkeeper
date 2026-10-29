#!/usr/bin/env python3
"""The sweep route planned on the robot's own map against the one planned on the generator's plan ([HuM]), scored on the
plan (an analysis tool: it reads the truth, the route never does, [1B2]). Static: no sim, the tuning houses' recorded
baseline nights. Per house, with the night tidy's own arguments (5 m round the dock, two cells clear):
  - rows: the passes laid; waypoints; length;
  - coverage: the share of the true reachable free floor within 5 m of the true dock lying within 0.6 m (and 1.0 m) of
    the route's path, its legs straight between waypoints;
  - in an obstacle: waypoints in a true wall or furniture cell; too close: nearer one than the robot's half width;
    unreachable: off the floor the true dock reaches.
Usage: sweep_compare.py [--seeds 70 71 ...] [--json out.json] [--png <dir>]   (dimos venv; never the hold-out houses)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import argparse, json, math
from collections import deque
from pathlib import Path

import numpy as np

import make_house as mh
import sweep_route as sr

RUNS = Path(__file__).resolve().parent / "runs"
TUNING = [70, 71, 72, 73, 76, 77, *range(82, 94)]          # the tuning houses ([aj8]); 78 (validation), 80, 81 (held out)
HELD_OUT = {78, 80, 81}                                    # never opened here
RADIUS, CLEAR = 5.0, 2                                     # night_tidy.sh's ROUTE_ARGS


def truth_route(seed, near, radius=RADIUS, c=CLEAR, sp=3, max_leg=3.0, min_clear_m=0.7):
    """the route as it was planned until 3 Oct 2026, on the generator's plan (sweep_route.py at a958714)"""
    mh.use_layout(seed); g = mh.base_grid()
    reach = {tuple(mh.START)}; q = deque(reach)
    while q:
        x, y = q.popleft()
        for n in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= n[0] < mh.W and 0 <= n[1] < mh.H and n not in reach and not g[n[1]][n[0]]: reach.add(n); q.append(n)
    free = lambda x, y: (x, y) in reach and all(0 <= xx < mh.W and 0 <= yy < mh.H and not g[yy][xx]
                                                for yy in range(y - c, y + c + 1) for xx in range(x - c, x + c + 1))
    pts, flip = [], False; rows = list(range(mh.H - c - 2, c, -sp))
    if rows[-1] - (c + 1) > sp // 2: rows.append(c + 1)
    for y in rows:
        segs, x = [], 0
        while x < mh.W:
            if free(x, y):
                x0 = x
                while x < mh.W and free(x, y): x += 1
                segs.append((x0, x - 1))
            x += 1
        if flip: segs = [(b, a_) for a_, b in reversed(segs)]
        for s0, s1 in segs:
            (ax, ay), (bx, by) = mh.cell_to_world(s0, y), mh.cell_to_world(s1, y)
            n = max(1, int(abs(bx - ax) // max_leg) + 1)
            pts += [[round(ax + (bx - ax) * k / n, 2), round(ay, 2)] for k in range(n + 1)]
        flip = not flip
    blocked = [mh.cell_to_world(x, y) for y in range(mh.H) for x in range(mh.W) if g[y][x]]
    def clearance(px, py):
        return min(math.hypot(max(abs(px - cx) - mh.CELL / 2, 0), max(abs(py - cy) - mh.CELL / 2, 0)) for cx, cy in blocked)
    def open_up(p):
        if clearance(*p) >= min_clear_m: return p
        cands = [(p[0] + dx * 0.05, p[1] + dy * 0.05) for dx in range(-12, 13) for dy in range(-12, 13)]
        ok = [q for q in cands if clearance(*q) >= min_clear_m]
        q = min(ok, key=lambda q: math.hypot(q[0] - p[0], q[1] - p[1])) if ok else max(cands, key=lambda q: clearance(*q))
        return [round(q[0], 2), round(q[1], 2)]
    pts = [open_up(p) for p in pts]; pts = [p for i, p in enumerate(pts) if i == 0 or p != pts[i - 1]]
    return [p for p in pts if math.dist(p, near) <= radius], rows


def truth_floor(seed, res=0.05):
    """the plan's grid, its reachable open cells, and sample points (res apart) over the reachable floor"""
    mh.use_layout(seed); g = np.array(mh.base_grid())
    reach = np.zeros_like(g, bool); reach[mh.START[1], mh.START[0]] = True; q = deque([tuple(mh.START)])
    while q:
        x, y = q.popleft()
        for n in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= n[0] < mh.W and 0 <= n[1] < mh.H and not reach[n[1], n[0]] and not g[n[1], n[0]]: reach[n[1], n[0]] = True; q.append(n)
    xs = np.arange(-mh.W * mh.CELL / 2 + res / 2, mh.W * mh.CELL / 2, res); ys = np.arange(-mh.H * mh.CELL / 2 + res / 2, mh.H * mh.CELL / 2, res)
    X, Y = np.meshgrid(xs, ys); P = np.c_[X.ravel(), Y.ravel()]
    return g, reach, P[reach[cell_of(P)]]


def cell_of(P):
    """world points -> (row, column) index arrays of the plan's grid (clipped to it)"""
    c = np.clip(((P[:, 0] + mh.W * mh.CELL / 2) / mh.CELL).astype(int), 0, mh.W - 1)
    r = np.clip(((mh.H * mh.CELL / 2 - P[:, 1]) / mh.CELL).astype(int), 0, mh.H - 1)
    return r, c


def path_dist(P, pts):
    """each point's distance to the route's path (straight legs between waypoints)"""
    if not len(pts): return np.full(len(P), np.inf)
    W = np.asarray(pts, float); d = np.hypot(*(P - W[0]).T)
    for a, b in zip(W[:-1], W[1:]):
        ab = b - a; t = np.clip(((P - a) @ ab) / max(ab @ ab, 1e-12), 0, 1)
        d = np.minimum(d, np.hypot(*(P - a - t[:, None] * ab).T))
    return d


def score(seed, pts, g, reach, F, start):
    """coverage, obstacle and reach figures of one route on the plan"""
    F = F[np.hypot(*(F - start).T) <= RADIUS]; d = path_dist(F, pts); W = np.asarray(pts, float).reshape(-1, 2)
    r, c = cell_of(W); inside = g[r, c] > 0
    blocked = np.array([mh.cell_to_world(x, y) for y in range(mh.H) for x in range(mh.W) if g[y, x]])
    clr = np.array([np.min(np.hypot(np.maximum(abs(px - blocked[:, 0]) - mh.CELL / 2, 0), np.maximum(abs(py - blocked[:, 1]) - mh.CELL / 2, 0)))
                    for px, py in W]) if len(W) else np.zeros(0)
    rows = len({round(y / 0.35) for _, y in W})
    length = sum(math.dist(W[i], W[i - 1]) for i in range(1, len(W)))
    return {"waypoints": len(W), "rows": rows, "length_m": round(length, 1),
            "cover_0.6": round(float((d <= 0.6).mean()), 3), "cover_1.0": round(float((d <= 1.0).mean()), 3),
            "in_obstacle": int(inside.sum()), "under_0.25m": int((clr < sr.BODY_M).sum()),
            "unreachable": int((~reach[r, c] & ~inside).sum()), "min_clear_m": round(float(clr.min()), 2) if len(clr) else None}


def map_route(seed):
    """the route sweep_route.py plans for night 3 on the house's two baseline nights, and its map"""
    run = RUNS / f"nights_s{seed}"; ff = json.loads((run / "floor_frame.json").read_text())
    frame = dict(zip(ff["dbs"], ff["corrections"]))
    nights = [(n, db) for n, db in sr.nights_from(run / "seed_recordings.txt", 3) if db in frame]
    fl, st, tl, dk = sr.night_maps(nights, frame, run)
    m = sr.Map.from_points(fl, st, talls=tl); dock = dk[-1]
    return sr.plan(m, dock, near=dock[:2], radius=RADIUS), m, dock


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--seeds", type=int, nargs="*", default=TUNING)
    ap.add_argument("--json", default=None); ap.add_argument("--png", default=None); a = ap.parse_args()
    out = {}
    for seed in a.seeds:
        if seed in HELD_OUT: print(f"house {seed}: held out, not opened"); continue
        run = RUNS / f"nights_s{seed}"
        if not (run / "floor_frame.json").exists(): print(f"house {seed}: no stitched baselines"); continue
        start = np.array(json.loads((run / f"nights_s{seed}_n1.truth.json").read_text())["start_pos"], float)
        old, _ = truth_route(seed, start); new, m, dock = map_route(seed)
        g, reach, F = truth_floor(seed)
        out[seed] = {"truth_plan": score(seed, old, g, reach, F, start), "robot_map": score(seed, new, g, reach, F, start),
                     "dock_error_m": round(math.dist(dock[:2], start), 3)}
        o, n = out[seed]["truth_plan"], out[seed]["robot_map"]
        print(f"house {seed}: rows {o['rows']} -> {n['rows']}, waypoints {o['waypoints']} -> {n['waypoints']}, "
              f"length {o['length_m']} -> {n['length_m']} m, coverage(0.6 m) {o['cover_0.6']:.0%} -> {n['cover_0.6']:.0%}, "
              f"(1.0 m) {o['cover_1.0']:.0%} -> {n['cover_1.0']:.0%}, in an obstacle {o['in_obstacle']} -> {n['in_obstacle']}, "
              f"under {sr.BODY_M} m {o['under_0.25m']} -> {n['under_0.25m']}, unreachable {o['unreachable']} -> {n['unreachable']}, "
              f"dock seen {out[seed]['dock_error_m'] * 100:.1f} cm off", flush=True)
        if a.png: picture(m, old, new, dock, Path(a.png) / f"route_s{seed}.png")
    if out:
        keys = ["rows", "waypoints", "length_m", "cover_0.6", "cover_1.0", "in_obstacle", "under_0.25m", "unreachable"]
        mean = {s: {k: round(float(np.mean([v[s][k] for v in out.values()])), 3) for k in keys} for s in ("truth_plan", "robot_map")}
        print("mean: " + "; ".join(f"{k} {mean['truth_plan'][k]} -> {mean['robot_map'][k]}" for k in keys))
        out["mean"] = mean
    if a.json: Path(a.json).write_text(json.dumps(out, indent=1))


def picture(m, old, new, dock, path):
    """the robot's map (floor grey, obstacles red), the old route (blue) and the new (yellow)"""
    from PIL import Image
    img = np.zeros(m.floor.shape + (3,), np.uint8); img[m.floor] = (90, 90, 90); img[m.occ] = (200, 50, 50)
    for pts, col in ((old, (80, 140, 255)), (new, (255, 230, 0))):
        for x, y in pts:
            r, c = m.cell(x, y)
            if 2 <= r < img.shape[0] - 2 and 2 <= c < img.shape[1] - 2: img[r - 2:r + 3, c - 2:c + 3] = col
    r, c = m.cell(*dock[:2]); img[r - 3:r + 4, c - 3:c + 4] = (0, 220, 0)
    path.parent.mkdir(parents=True, exist_ok=True); Image.fromarray(img[::-1]).resize((img.shape[1] * 2, img.shape[0] * 2), 0).save(path)


if __name__ == "__main__":
    main()
