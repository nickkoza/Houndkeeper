#!/usr/bin/env python3
"""Furniture that gets moved every night but is never a pick-up target: dining chairs, a stool, a kid's chair.
Each has a home pose. Per night each is TUCKED (near home), moved A BIT (5-25 cm out, some turn) or moved
A LOT (0.5-2 m to another free spot). Static in the sim (the robot bumps into them like real chairs)."""
import math
import make_house as mh

# id, home x, home y, home yaw (rad; chair faces +x at yaw 0), seat w, seat d, seat h, back h (0 = none), rgba
CHAIRS = [
    ("chair_n1", -0.20, -0.72, -math.pi / 2, .44, .44, .45, .90, "0.36 0.27 0.20 1"),
    ("chair_n2",  0.90, -0.72, -math.pi / 2, .44, .44, .45, .90, "0.36 0.27 0.20 1"),
    ("chair_s1", -0.20, -2.43,  math.pi / 2, .44, .44, .45, .90, "0.36 0.27 0.20 1"),
    ("chair_s2",  0.90, -2.43,  math.pi / 2, .44, .44, .45, .90, "0.36 0.27 0.20 1"),
    ("chair_w",  -1.03, -1.58,  0.0,         .44, .44, .45, .90, "0.36 0.27 0.20 1"),
    ("chair_e",   1.73, -1.58,  math.pi,     .44, .44, .45, .90, "0.36 0.27 0.20 1"),
    ("stool",    -5.60,  1.60,  0.0,         .40, .40, .42, 0.0, "0.55 0.52 0.48 1"),
    ("kid_chair", 5.60,  1.00,  0.3,         .30, .30, .28, .55, "0.25 0.55 0.80 1"),
]
P_TUCK, P_BIT = 0.25, 0.60          # rest = moved a lot
LOT_BOX = {"stool": (-7.8, -1.5, 3.0, 3.5), "kid_chair": (3.8, -1.5, 8.0, 4.0)}   # x0, y0, x1, y1 for big moves
LOT_BOX_DEFAULT = (-3.5, -4.3, 3.8, 1.5)


def footprint_cells(x, y, w, d, margin=1):
    r = math.hypot(w, d) / 2
    x0, y1 = int((x - r + mh.W * mh.CELL / 2) / mh.CELL), int((mh.H * mh.CELL / 2 - (y - r)) / mh.CELL)
    x1, y0 = int((x + r + mh.W * mh.CELL / 2) / mh.CELL), int((mh.H * mh.CELL / 2 - (y + r)) / mh.CELL)
    return {(cx, cy) for cx in range(x0 - margin, x1 + margin + 1) for cy in range(y0 - margin, y1 + margin + 1)}


def chairs():
    """the chairs with tonight's plan's homes: a hold-out's own plan puts them round its own table, by its own play shelf
    and in its own kitchen (make_house.CHAIR_HOMES, sim-house-nights [ejh]); every other plan keeps CHAIRS as they are"""
    H = mh.CHAIR_HOMES
    if not H: return list(CHAIRS)
    return [(c[0], *H[c[0]][:3], *c[4:]) if c[0] in H else c for c in CHAIRS]


def lot_box(cid):
    """where a chair moved a lot may land: its room in a hold-out's own plan, else LOT_BOX's"""
    H = mh.CHAIR_HOMES
    return H[cid][3] if H and cid in H else LOT_BOX.get(cid, LOT_BOX_DEFAULT)


def place(rng, g, prev=None):
    """Tonight's poses. prev: last night's list (for 'moved from'). Never on walls/furniture, each other, or the dock."""
    out, used = [], set()
    sx, sy = mh.cell_to_world(*mh.START)
    CH = chairs()
    homes = {c[0]: (c[1], c[2], max(c[4], c[5])) for c in CH}   # every chair's home kept for it: a chair moved a lot
    clear = lambda cid, x, y, w: (all(math.hypot(x - q["x"], y - q["y"]) >= max(w, q["w"]) for q in out)   # onto another's
                                  and all(math.hypot(x - hx_, y - hy_) >= max(w, hw_) for o, (hx_, hy_, hw_) in homes.items()   # home
                                          if o != cid and o not in {q["id"] for q in out}))   # left that one stacked on it when it
    for (cid, hx, hy, hyaw, w, d, h, bh, rgba) in CH:                                         # stayed home (house 78, 1 Oct)
        r = rng.random(); drawn = "tucked" if r < P_TUCK else "a bit" if r < P_TUCK + P_BIT else "a lot"
        for attempt in range(500):                      # the move is drawn once; only its position is retried (re-drawing
            mode = drawn if attempt < 400 else "tucked"  # the move too made big moves 3% of chairs, not 15%: 23 Sep, 27 Sep)
            if mode == "tucked":
                x, y, yaw = hx + rng.uniform(-.03, .03), hy + rng.uniform(-.03, .03), hyaw + rng.uniform(-.05, .05)
            elif mode == "a bit":
                dist, side = rng.uniform(.05, .25), rng.uniform(-.1, .1)
                ox, oy = -math.cos(hyaw), -math.sin(hyaw)          # pulled out = away from what it faces
                x, y = hx + ox * dist - oy * side, hy + oy * dist + ox * side; yaw = hyaw + rng.uniform(-.35, .35)
            else:
                bx0, by0, bx1, by1 = lot_box(cid)
                x, y, yaw = rng.uniform(bx0, bx1), rng.uniform(by0, by1), rng.uniform(-math.pi, math.pi)
                if not (.5 <= math.hypot(x - hx, y - hy) <= 2.0): continue
            cells = footprint_cells(x, y, w, d, margin=0)
            if any(not (0 <= cx < mh.W and 0 <= cy < mh.H) for cx, cy in cells): continue
            if mode != "tucked" and any(g[cy][cx] for cx, cy in cells): continue
            if math.hypot(x - sx, y - sy) < 1.2: continue    # (chairs apart by their size, not by 35 cm cells: the cells
            if not clear(cid, x, y, max(w, d)): continue      # had chairs at home "touching" neighbours a metre off)
            used |= footprint_cells(x, y, w, d, margin=0); break
        else:                                            # no clear spot: a chair never lands on another, so say so
            raise RuntimeError(f"{cid}: no clear spot tonight (chairs placed: {[q['id'] for q in out]})")
        p = next((q for q in (prev or []) if q["id"] == cid), None)
        moved = round(math.hypot(x - p["x"], y - p["y"]), 3) if p else None
        out.append({"id": cid, "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3), "w": w, "d": d, "h": h, "back_h": bh,
                    "rgba": rgba, "mode": mode, "moved_m": moved})
    return out


def taken_cells(chairs):
    s = set()
    for c in chairs: s |= footprint_cells(c["x"], c["y"], c["w"], c["d"], margin=1)
    return s


def xml(c):
    """a chair or stool as a body; wooden ones in the house's wood on every face ([jmf]: meshes carrying their own texture
    coordinates, declared with the scene's meshes), the kid's plastic chair in its colour"""
    w, d, h, bh, L = c["w"] / 2, c["d"] / 2, c["h"], c["back_h"], 0.015
    if not c["id"].startswith("kid"):
        import cabinets
        before = len(cabinets.GEOMS); i = c["id"]
        for k, (sx, sy) in enumerate((sx, sy) for sx in (-1, 1) for sy in (-1, 1)):
            x, y = sx * (w - L), sy * (d - L)
            cabinets.piece(f"{i}_leg{k}", x - L, y - L, 0.0, x + L, y + L, h - .04, "wood_m", True, 2.0)
        cabinets.piece(f"{i}_seat", -w, -d, h - .04, w, d, h, "wood_m", True, 2.0)
        if bh > h: cabinets.piece(f"{i}_back", -w, -d, h, -w + .03, d, bh, "wood_m", True, 2.0)
        geoms = "".join(g.strip() for g in cabinets.GEOMS[before:]); del cabinets.GEOMS[before:]
        return f'    <body name="{i}" pos="{c["x"]} {c["y"]} 0" euler="0 0 {c["yaw"]}">{geoms}</body>'
    legs = "".join(f'<geom type="box" pos="{sx * (w - L):.3f} {sy * (d - L):.3f} {h / 2 - .02:.3f}" size="{L} {L} {h / 2 - .02:.3f}" rgba="{c["rgba"]}" contype="1" conaffinity="0"/>'
                   for sx in (-1, 1) for sy in (-1, 1))
    seat = f'<geom type="box" pos="0 0 {h - .02:.3f}" size="{w:.3f} {d:.3f} 0.02" rgba="{c["rgba"]}" contype="1" conaffinity="0"/>'
    back = (f'<geom type="box" pos="{-w + .015:.3f} 0 {(h + bh) / 2:.3f}" size="0.015 {d:.3f} {(bh - h) / 2:.3f}" rgba="{c["rgba"]}" contype="1" conaffinity="0"/>'
            if bh > h else "")
    return f'    <body name="{c["id"]}" pos="{c["x"]} {c["y"]} 0" euler="0 0 {c["yaw"]}">{legs}{seat}{back}</body>'
