#!/usr/bin/env python3
"""Lawnmower (boustrophedon) sweep waypoints over the floor the robot has mapped ([HuM]): planned on its own lidar map of
its earlier nights, from its dock as it saw it, never on the generator's plan ([1B2]; until 3 Oct 2026 it read the
house's true walls, furniture and dock from make_house, sim-kindness audit finding 3).
  - the map: each earlier night's lidar scans, levelled on their own floor and placed in the house's frame by the
    stitch's corrections (floor_frame.json: house_shape.py's), as the floor pictures are; with none, in the night's
    odometry frame. A 5 cm cell is floor where a scan saw floor, an obstacle where returns stood 0.10-2.0 m up on any
    night used, or the night's tall layer (tall_layer.py) marks it. Not DimOS's global_map: it accumulates on
    odometry, whose drift doubled every wall (house 72: 7.7 deg by the end of night 1, 3 Oct 2026). The tall layer's
    marks block only where they repeat on --min-nights nights and on two when two or more are mapped ([pI7]): a one-night
    ridge on house 72's night 2 walled its dock into 2.4 m2 (13 Oct 2026);
  - the dock: where the night began (the robot starts docked, every night), in the same frame;
  - reachable: floor the robot's body fits through, joined to the dock (a waypoint behind a wall is a move_to that
    fails, and the robot stands through each failure: 2 Oct 2026, the demo room's night 1 stood ~1 min at 13 of them);
  - rows --spacing-cells apart from the dock's end, the last along the far wall, each only where the floor stands
    --clear cells clear of anything mapped or unseen; a waypoint short of --min-clear-m pushed out to where it is not.
Row spacing is a guess until we know the camera's floor footprint (see recorded frames).
A first night has no map: the caller explores first (DimOS's frontier exploration, as sweep_night.sh does), then plans on
the exploration's own recording (--db, read while DimOS records). With no map at all it plans nothing and exits 3; with a
map but no waypoint it says why on stderr and exits 5 ([pI7]).
Usage: sweep_route.py [--recordings <file of "night db" lines; default $SEED_RECS>] [--night N: nights before N]
       [--db <memory.db> ...] [--run <run dir: floor_frame.json, tall_n<k>.npz>] [--near dock|x,y] [--radius 4]
       [--spacing-cells 3] [--clear 2] [--area <operating_area.json or its map dir>] > route.json   (dimos venv)
With --area the route is planned inside the operating area only, its edge kept clear as a wall is ([JVr] [OrO]); a dock
outside the area is refused (exit 4)."""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))
import argparse, json, math, os, sys
from pathlib import Path

import numpy as np
from scipy import ndimage as ndi

CELL = 0.35                     # m: the unit of --spacing-cells and --clear (the planning cell the route has always used)
RES = 0.05                      # m: the map's cells (DimOS's costmap resolution)
FLOOR_Z, TOP_Z = 0.10, 2.0      # floor returns lie below FLOOR_Z; above it and under DimOS's ceiling cut, something stands
MIN_HITS, MIN_SHARE = 2, 0.3   # something stands in a cell when 2+ scans put a return up in it, and as many as 0.3 of the
                                # scans that saw the cell at all (a few stray returns over open floor: 3 Oct 2026, house 70)
BODY_M = 0.25                   # the robot's half width: a cell it can pass is this clear of anything mapped
DOCK_REACH_M = 1.0              # the dock's own cells may be unseen (the robot stood on them): joined from within this
TALL_HX, TALL_HY, TALL_RES = 8.4, 4.9, 0.01   # the tall layer's grid (tall_layer.py, floor_mosaic2's), row 0 at +y
TALL_NIGHTS = 2                 # nights a tall-layer mark must repeat on, when that many nights have a layer ([pI7])
TALL_SHARE = float(os.environ.get("HK_TALL_SHARE", 0.2))   # of a 5 cm cell's 1 cm pixels the tall layer must mark: a lone
                                # 4-6 cm pixel on carpet is its noise (house 70, 3 Oct 2026); a bowl covers its cells
FILL_M = 0.10                   # floor returns a few cm apart: gaps this wide in the seen floor are closed


def night_map(db, corrections=None):
    """one recording's map: (floor cells N x 2, standing cells M x 2, dock (x, y, heading)), each cell's centre, in the
    house's frame when its corrections (npz: times, Ts, house <- believed) are given, else the night's odometry frame"""
    from dimos.memory.store.sqlite import SqliteStore
    from houndkeeper.localize import Localizer, scans_from, level
    st = SqliteStore(path=str(db)); st.start()
    try:
        S = scans_from(st); dock = None
        for o in st.stream("tf"):
            for t in getattr(o.data, "transforms", [o.data]):
                if (t.frame_id, t.child_frame_id) == ("world", "base_link"):
                    M = t.to_matrix(); dock = (float(M[0, 3]), float(M[1, 3]), math.atan2(M[1, 0], M[0, 0]), o.ts); break
            if dock: break
    finally:
        st.stop()
    loc = None
    if corrections is not None: z = np.load(corrections); loc = Localizer(times=list(z["times"]), Ts=list(z["Ts"]))
    down, up = {}, {}                                     # per cell: the scans that saw floor there, and something up
    for t, p in S:
        P = level(np.asarray(p, float))
        if loc is not None: C = loc.correction(t); P = np.c_[P[:, :2] @ C[:2, :2].T + C[:2, 2], P[:, 2]]
        k = np.floor(P[:, :2] / RES).astype(np.int64); low = P[:, 2] <= FLOOR_Z; hi = ~low & (P[:, 2] < TOP_Z)
        for q in map(tuple, np.unique(k[low], axis=0)): down[q] = down.get(q, 0) + 1
        for q in map(tuple, np.unique(k[hi], axis=0)): up[q] = up.get(q, 0) + 1
    cells = lambda K: (np.array(sorted(K), float).reshape(-1, 2) + 0.5) * RES
    stands = [q for q, n in up.items() if n >= MIN_HITS and n >= MIN_SHARE * (n + down.get(q, 0))]
    if dock is not None and loc is not None:
        C = loc.correction(dock[3]); x, y = C[:2, :2] @ dock[:2] + C[:2, 2]
        dock = (float(x), float(y), dock[2] + math.atan2(C[1, 0], C[0, 0]), dock[3])
    return cells(down), cells(stands), dock and dock[:3]


def tall_points(npz, share=TALL_SHARE):
    """the tall layer's standing 5 cm cells as points (N x 2): those it marks over `share` of"""
    h = np.load(npz)["h"] > 0; k = round(RES / TALL_RES); H, W = (h.shape[0] // k) * k, (h.shape[1] // k) * k
    frac = h[:H, :W].reshape(H // k, k, W // k, k).mean((1, 3)); i, j = np.nonzero(frac >= share)
    return np.c_[-TALL_HX + (j + 0.5) * RES, TALL_HY - (i + 0.5) * RES]


class Map:
    """the robot's floor map on a 5 cm grid: floor (seen floor, nothing standing), occ (something standing), known;
    row r, column c is the cell at (x0 + (c + 0.5) RES, y0 + (r + 0.5) RES)"""
    def __init__(self, floor, occ, x0, y0):
        self.floor, self.occ, self.x0, self.y0 = floor, occ, x0, y0

    @classmethod
    def from_points(cls, floors, stands, min_nights=1, talls=(), tall_nights=TALL_NIGHTS):
        """floors, stands: per night, points N x 2 (one frame); a cell is an obstacle if something stood in it on
        min_nights nights, floor if any night saw floor there and it is no obstacle. talls: the nights' tall layers (points
        N x 2, one per night that has one): a cell they mark is an obstacle when marked on min_nights of them and on
        tall_nights when that many layers are given ([pI7]: with one layer its marks count)"""
        talls = [np.asarray(t, float).reshape(-1, 2) for t in talls]
        allp = np.vstack([p for p in [*floors, *stands, *talls] if len(p)] or [np.zeros((1, 2))])
        x0, y0 = allp.min(0) - 1.0; w, h = (np.ceil((allp.max(0) + 1.0 - (x0, y0)) / RES)).astype(int)
        def grid(P):
            g = np.zeros((h, w), bool)
            if len(P):                                    # (+1e-6: a cell's own centre lands in it, not the one below)
                c = np.floor((P[:, 0] - x0) / RES + 1e-6).astype(int); r = np.floor((P[:, 1] - y0) / RES + 1e-6).astype(int)
                g[r, c] = True
            return g
        seen = np.zeros((h, w), bool); count = np.zeros((h, w), int)
        for P in floors: seen |= grid(P)
        for P in stands: count += grid(P)
        occ = count >= min_nights
        if talls:
            tcount = np.zeros((h, w), int)
            for P in talls: tcount += grid(P)
            occ |= tcount >= max(min_nights, min(tall_nights, len(talls)))
        k = max(1, round(FILL_M / RES))
        seen = ndi.binary_closing(seen, iterations=k) | seen
        return cls(seen & ~occ, occ, x0, y0)

    def cell(self, x, y):
        return int((y - self.y0) / RES), int((x - self.x0) / RES)

    def xy(self, r, c):
        return self.x0 + (c + 0.5) * RES, self.y0 + (r + 0.5) * RES

    def clearance(self):
        """m from each cell to the nearest obstacle or unseen cell (what the lidar never saw may hold anything)"""
        return ndi.distance_transform_edt(self.floor) * RES


def reachable(m: Map, dock, clr):
    """the floor cells the robot's body reaches from the dock"""
    walk = m.floor & (clr >= BODY_M)
    lab, n = ndi.label(walk)
    if not n: return walk
    r, c = m.cell(*dock[:2]); k = int(DOCK_REACH_M / RES)
    r0, r1, c0, c1 = max(r - k, 0), min(r + k + 1, walk.shape[0]), max(c - k, 0), min(c + k + 1, walk.shape[1])
    win = lab[r0:r1, c0:c1]
    if not (win > 0).any(): return np.zeros_like(walk)
    rr, cc = np.nonzero(win > 0); d = (rr + r0 - r) ** 2 + (cc + c0 - c) ** 2
    return lab == win[rr[d.argmin()], cc[d.argmin()]]


def plan(m: Map, dock, spacing_m=3 * CELL, clear_m=2 * CELL, max_leg=3.0, near=None, radius=4.0, min_clear_m=0.7):
    """the sweep's waypoints [[x, y], ...] over the map's reachable floor, rows along x from the dock's end"""
    clr = m.clearance(); reach = reachable(m, dock, clr)
    ok = reach & (clr >= clear_m - 1e-9)
    if near is not None:
        R, C = np.mgrid[:ok.shape[0], :ok.shape[1]]
        ok &= np.hypot(m.x0 + (C + 0.5) * RES - near[0], m.y0 + (R + 0.5) * RES - near[1]) <= radius
    rows = np.nonzero(ok.any(1))[0]
    if not len(rows): return []
    step = max(1, round(spacing_m / RES)); dr = m.cell(*dock[:2])[0]
    a, b = (rows.min(), rows.max()) if abs(dr - rows.min()) <= abs(dr - rows.max()) else (rows.max(), rows.min())
    seq = list(range(a, b + (1 if b >= a else -1), step if b >= a else -step))   # from the dock's end
    if abs(b - seq[-1]) > step // 2: seq.append(b)       # and a last pass along the far wall, not a pass short of it
    pts, flip = [], False                                 # (2 Oct 2026: the demo room's far corner went unswept)
    for r in seq:
        row = ok[r]
        edges = np.flatnonzero(np.diff(np.r_[0, row.astype(np.int8), 0]))
        segs = list(zip(edges[::2], edges[1::2] - 1))
        if flip: segs = [(e, s) for s, e in reversed(segs)]
        for s0, s1 in segs:
            (ax, ay), (bx, by) = m.xy(r, s0), m.xy(r, s1)
            n = max(1, int(abs(bx - ax) // max_leg) + 1)
            pts += [[round(ax + (bx - ax) * k / n, 2), round(ay, 2)] for k in range(n + 1)]
        flip = not flip
    pts = [open_up(m, clr, reach, p, min_clear_m) for p in pts]   # a waypoint the robot stops at hard by a wall or a chair
                                                       # leaves it inside the planner's margin, where it finds no path out
                                                       # (2 Oct 2026: 5 of 15)
    return [p for i, p in enumerate(pts) if i == 0 or p != pts[i - 1]]   # a waypoint once, not twice in a row


def why_empty(m: Map, dock, clear_m, near=None, radius=4.0):
    """why plan() found no waypoint, in words: the floor the dock reaches, its best clearance against clear_m, and how far
    the walkable floor that is clear enough lies from the dock ([pI7])"""
    clr = m.clearance(); reach = reachable(m, dock, clr); walk = m.floor & (clr >= BODY_M)
    a = lambda g: float(g.sum()) * RES * RES
    R, C = np.mgrid[:m.floor.shape[0], :m.floor.shape[1]]
    d = np.hypot(m.x0 + (C + 0.5) * RES - dock[0], m.y0 + (R + 0.5) * RES - dock[1])
    open_ = walk & (clr >= clear_m - 1e-9)
    out = [f"the dock ({dock[0]:.2f}, {dock[1]:.2f}) reaches {a(reach):.1f} m2 of the {a(walk):.1f} m2 the body fits through",
           f"best clearance there {float(clr[reach].max()) if reach.any() else 0.0:.2f} m, {clear_m:.2f} m asked"]
    if near is not None:
        within = np.hypot(m.x0 + (C + 0.5) * RES - near[0], m.y0 + (R + 0.5) * RES - near[1]) <= radius
        out.append(f"{a(reach & open_ & within):.1f} m2 of it clear enough within {radius:g} m of {near[0]:.2f}, {near[1]:.2f}")
    if (open_ & ~reach).any():
        out.append(f"the dock is in a pocket: floor clear enough lies {float(d[open_ & ~reach].min()):.2f} m from it, "
                   f"not joined to it")
    return "; ".join(out)


def open_up(m: Map, clr, reach, p, min_clear_m, look_m=0.6):
    """the nearest reachable point with min_clear_m round it, within look_m (or the most open one there)"""
    r, c = m.cell(*p)
    if 0 <= r < clr.shape[0] and 0 <= c < clr.shape[1] and reach[r, c] and clr[r, c] >= min_clear_m: return p
    k = int(look_m / RES); r0, c0 = max(r - k, 0), max(c - k, 0)
    win = np.where(reach[r0:r + k + 1, c0:c + k + 1], clr[r0:r + k + 1, c0:c + k + 1], -1.0)
    if (win < 0).all(): return p
    rr, cc = np.mgrid[:win.shape[0], :win.shape[1]]; d = np.hypot(rr + r0 - r, cc + c0 - c)
    good = win >= min_clear_m
    i = np.argmin(np.where(good, d, np.inf)) if good.any() else np.argmax(win)
    x, y = m.xy(r0 + i // win.shape[1], c0 + i % win.shape[1]); return [round(x, 2), round(y, 2)]


def limit_to_area(m: Map, area):
    """the map's floor inside the operating area only: outside it is no floor to plan on ([OrO])"""
    from dimos.mapping.keepout_zones import _inside
    r, c = np.mgrid[:m.floor.shape[0], :m.floor.shape[1]]
    P = np.c_[m.x0 + (c.ravel() + 0.5) * RES, m.y0 + (r.ravel() + 0.5) * RES]
    ins = np.zeros(len(P), bool)
    for poly in area.zones().allowed: ins |= _inside(P, poly)
    ins = ins.reshape(m.floor.shape)
    if area.walls:                                        # a virtual wall is a wall: no floor under it
        from dimos.mapping.keepout_zones import _segment_distance
        X, Y = P[:, 0].reshape(m.floor.shape), P[:, 1].reshape(m.floor.shape)
        for s in area.zones().walls: ins &= _segment_distance(X, Y, s) > RES
    m.floor = m.floor & ins
    return m


def night_maps(nights, frame, run, dbs=()):
    """per night (nights [(n, db)], then each extra recording): floor points, lidar standing points, dock; and the tall
    layers (one per night that has one, in the house's frame: used only with the stitch's frame)"""
    floors, stands, talls, docks = [], [], [], []
    for n, db in nights:
        f, s, d = night_map(db, frame.get(db)); floors.append(f); stands.append(s); docks.append(d)
        t = run / f"tall_n{n}.npz" if run else None      # the tall layer, in the frame the stitch made it in
        if t is not None and t.exists() and frame: talls.append(tall_points(t))
    for db in dbs:
        f, s, d = night_map(db); floors.append(f); stands.append(s); docks.append(d)
    return floors, stands, talls, docks


def nights_from(path, night=None):
    """[(night, db)] from a file of "night db" lines, the nights before `night` only"""
    out = []
    for line in Path(path).read_text().splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0].isdigit() and (night is None or int(parts[0]) < night) and Path(parts[1]).exists():
            out.append((int(parts[0]), parts[1]))
    return sorted(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recordings", default=os.environ.get("SEED_RECS"), help="earlier nights: a file of 'night memory.db' lines (night_tidy.sh's seed_recordings.txt)")
    ap.add_argument("--night", type=int, default=None, help="tonight's number: only the nights before it are used")
    ap.add_argument("--db", action="append", default=[], help="a recording to map from as well, e.g. tonight's exploration's, read as it records (a first night's map)")
    ap.add_argument("--run", default=None, help="the run dir holding each night's tall layer (tall_n<k>.npz); default: the recordings file's dir")
    ap.add_argument("--min-nights", type=int, default=1, help="nights something must stand in a cell to block it")
    ap.add_argument("--spacing-cells", type=int, default=3)   # 3 x 0.35 m = 1.05 m between passes (estimate)
    ap.add_argument("--clear", type=int, default=2)           # cells kept clear of walls/furniture (~0.7 m)
    ap.add_argument("--max-leg", type=float, default=3.0)     # split long passes so move_to legs stay short
    ap.add_argument("--near", default=None, help="'dock', or x,y: keep only floor within --radius of it (a part of the house)")
    ap.add_argument("--radius", type=float, default=4.0)
    ap.add_argument("--min-clear-m", type=float, default=0.7, help="a waypoint this far from anything mapped, where it can be")
    ap.add_argument("--area", default=os.environ.get("HK_AREA"), help="the operating area (file or map dir): plan only inside it")
    ap.add_argument("--seed", type=int, default=None, help="ignored: the robot plans on its own map, not the house's seed")
    a = ap.parse_args()
    if a.seed is not None: print("sweep_route: --seed ignored, the route is planned on the robot's own map", file=sys.stderr)
    nights = nights_from(a.recordings, a.night) if a.recordings and Path(a.recordings).exists() else []
    run = Path(a.run) if a.run else Path(a.recordings).parent if a.recordings else None
    frame = {}                                            # db -> its corrections into the house's frame (the stitch's)
    if run is not None and (run / "floor_frame.json").exists():
        ff = json.loads((run / "floor_frame.json").read_text()); frame = dict(zip(ff["dbs"], ff["corrections"]))
    if any(db in frame for _, db in nights):          # the stitched nights alone: one frame for the whole map
        left = [n for n, db in nights if db not in frame]; nights = [(n, db) for n, db in nights if db in frame]
        if left: print(f"sweep_route: nights {left} left out: not in the house's frame (floor_frame.json)", file=sys.stderr)
    else: frame = {}
    floors, stands, talls, docks = night_maps(nights, frame, run, a.db)
    if not any(len(f) for f in floors) or not any(docks):
        print("sweep_route: no map of this house yet (a first night): explore first, then plan on the exploration's "
              "recording (--db)", file=sys.stderr); sys.exit(3)
    dock = [d for d in docks if d][-1]                    # tonight's own, when its recording is given; else the latest night's
    m = Map.from_points(floors, stands, a.min_nights, talls)
    from houndkeeper import operating_area as oa
    area = oa.load(a.area) if a.area else None
    if a.area and area is None: print(f"sweep_route: no operating area at {a.area}", file=sys.stderr); sys.exit(4)
    if area is not None:
        if not oa.inside(area, dock[:2]):
            print("sweep_route: the dock is outside the operating area: refusing to plan", file=sys.stderr); sys.exit(4)
        m = limit_to_area(m, area)
    near = dock[:2] if a.near == "dock" else tuple(map(float, a.near.split(","))) if a.near else None
    pts = plan(m, dock, a.spacing_cells * CELL, a.clear * CELL, a.max_leg, near, a.radius, a.min_clear_m)
    length = sum(math.dist(pts[i], pts[i - 1]) for i in range(1, len(pts)))
    out = json.dumps({"spacing_m": round(a.spacing_cells * CELL, 2), "clear_m": round(a.clear * CELL, 2),
                      "length_m": round(length, 1), "waypoints": pts, "dock": [round(v, 3) for v in dock],
                      "map": {"nights": [n for n, _ in nights], "recordings": [*(db for _, db in nights), *a.db],
                              "frame": "house" if frame else "odometry", "area": bool(area is not None),
                              "floor_m2": round(float(m.floor.sum()) * RES * RES, 1)}})
    print(out)
    if not pts:                                           # a night with nothing to sweep stops on the reason ([pI7])
        print("sweep_route: no waypoints: " + why_empty(m, dock, a.clear * CELL, near, a.radius), file=sys.stderr); sys.exit(5)


if __name__ == "__main__":
    main()
