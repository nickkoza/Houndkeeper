"""Cabinets as the head camera sees real ones (sim-house-nights [QMt]), textured, not
flat: door and drawer fronts standing 18 mm proud of the carcass with 3 mm gaps, a handle on each, applied panels
on the island's seating side and on exposed ends, and a finish per house (painted, over a faint wood grain, or stained
wood). Carcasses and fronts are meshes with their own texture coordinates, so the grain runs up each door: MuJoCo 3.10
stretches a 2D texture across a box's vertical faces (it read as one flat colour), and the renderers keep a mesh's
coordinates as they do the scanned toys'.

Usage (make_nights): reset(), then carcass()/fronts()/panels() while building furniture, then material_xml(seed) and
ASSETS into the scene's <asset>, GEOMS into its worldbody."""
import random
from pathlib import Path

import numpy as np
from PIL import Image

PROUD, GAP = 0.018, 0.003        # fronts stand off the carcass (a 3/4 inch door) with a 3 mm reveal (est. typical)
DOOR_W, DRAWER_H = 0.50, 0.16    # the widest a single door runs; a drawer row's height (est. typical)
REP = 2.0                        # texture repeats a metre (the wood photos cover ~50 cm, as make_nights.SURFACES)
FINISHES = [("white", (0.91, 0.90, 0.87)), ("warm white", (0.93, 0.89, 0.81)), ("grey", (0.60, 0.61, 0.60)),
            ("navy", (0.20, 0.25, 0.34)), ("sage", (0.58, 0.64, 0.54)), ("stained", None)]   # the finishes kitchens
                                 # most often have (est.); painted ones keep a faint grain under the paint
HANDLE = (0.018, 0.12)           # a bar handle's stand-off and length (est. a 96 mm bar pull)

ASSETS: list = []; GEOMS: list = []


def reset():
    ASSETS.clear(); GEOMS.clear()


def _mesh(name, x0, y0, z0, x1, y1, z1, rep=REP):
    """a box as a mesh whose every face carries texture coordinates in metres x rep (the texture's repeats a metre); on
    the faces that stand upright the texture's v runs up, so a door's grain is vertical. Written about its own centre,
    placed by the geom. Any furniture box can be one: MuJoCo 3.10 stretches a 2D texture across a box's upright faces."""
    cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
    hx, hy, hz = (x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2
    V, T, F = [], [], []
    def quad(corners, uv):
        k = len(V); V.extend(corners); T.extend(uv); F.extend([(k, k + 1, k + 2), (k, k + 2, k + 3)])
    for s in (-1, 1):                                                        # the +-y faces (fronts face -y in the house)
        c = [(-hx, s * hy, -hz), (hx, s * hy, -hz), (hx, s * hy, hz), (-hx, s * hy, hz)]
        c = c if s < 0 else c[::-1]
        quad(c, [((x + cx) * rep, (z + cz) * rep) for x, _, z in c])
    for s in (-1, 1):                                                        # the +-x faces (the ends)
        c = [(s * hx, hy, -hz), (s * hx, -hy, -hz), (s * hx, -hy, hz), (s * hx, hy, hz)]
        c = c if s < 0 else c[::-1]
        quad(c, [((y + cy) * rep, (z + cz) * rep) for _, y, z in c])
    for s in (-1, 1):                                                        # top and bottom
        c = [(-hx, -hy, s * hz), (-hx, hy, s * hz), (hx, hy, s * hz), (hx, -hy, s * hz)]
        c = c if s < 0 else c[::-1]
        quad(c, [((x + cx) * rep, (y + cy) * rep) for x, y, _ in c])
    f = lambda a: " ".join(f"{v:.4f}" for p in a for v in p)
    ASSETS.append(f'    <mesh name="m_{name}" vertex="{f(V)}" texcoord="{f(T)}" face="{" ".join(str(i) for t in F for i in t)}"/>')
    return (cx, cy, cz)


def piece(name, x0, y0, z0, x1, y1, z1, mat="cabinet", solid=False, rep=REP):
    """one textured box as a mesh geom; mat must be a material with texuniform false (the mesh carries its coordinates)"""
    cx, cy, cz = _mesh(name, x0, y0, z0, x1, y1, z1, rep)
    GEOMS.append(f'    <geom name="{name}" type="mesh" mesh="m_{name}" pos="{cx:.4f} {cy:.4f} {cz:.4f}" material="{mat}" '
                 f'contype="{int(solid)}" conaffinity="0"/>')


def cylinder(name, cx, cy, z0, z1, r, mat="wood_m", solid=False, rep=REP, n=40):
    """a round piece (a table top, a column) as a mesh: the texture across its top and bottom as a plane, round its side
    at real scale (texture u along the circumference, v up), each face its own vertices"""
    import math
    V, T, F = [], [], []
    for i in range(n):                                                     # the side, a quad per segment
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n; k = len(V)
        for a, z in ((a0, z0), (a1, z0), (a1, z1), (a0, z1)):
            V.append((r * math.cos(a), r * math.sin(a), z - (z0 + z1) / 2)); T.append((a * r * rep, z * rep))
        F.extend([(k, k + 1, k + 2), (k, k + 2, k + 3)])
    for z, up in ((z1, True), (z0, False)):                                # the caps, fans about their centres
        k = len(V); V.append((0.0, 0.0, z - (z0 + z1) / 2)); T.append((cx * rep, cy * rep))
        for i in range(n):
            a = 2 * math.pi * i / n; V.append((r * math.cos(a), r * math.sin(a), z - (z0 + z1) / 2))
            T.append(((cx + r * math.cos(a)) * rep, (cy + r * math.sin(a)) * rep))
        for i in range(n):
            b, c = k + 1 + i, k + 1 + (i + 1) % n
            F.append((k, b, c) if up else (k, c, b))
    f = lambda a: " ".join(f"{v:.4f}" for p in a for v in p)
    ASSETS.append(f'    <mesh name="m_{name}" vertex="{f(V)}" texcoord="{f(T)}" face="{" ".join(str(i) for t in F for i in t)}"/>')
    GEOMS.append(f'    <geom name="{name}" type="mesh" mesh="m_{name}" pos="{cx:.4f} {cy:.4f} {(z0 + z1) / 2:.4f}" material="{mat}" '
                 f'contype="{int(solid)}" conaffinity="0"/>')


def carcass(name, x0, y0, x1, y1, z0, z1):
    """the cabinet's box, solid (what the robot bumps), in the cabinet finish"""
    piece(name, x0, y0, z0, x1, y1, z1, solid=True)


def _widths(w, most):
    n = max(1, int(np.ceil((w - 1e-6) / most))); return [w / n] * n


def fronts(name, x0, x1, y_face, side, z0, z1, drawers=False, handles=True):
    """doors (and a drawer row above them when drawers) across a face at y_face, standing out toward side (-1: -y);
    a bar handle on each: upright near a door's meeting edge, level on a drawer"""
    y_out = y_face + side * PROUD; ya, yb = sorted((y_face, y_out))           # the front's face is y_out
    rows = [(z1 - DRAWER_H, z1, "drawer"), (z0, z1 - DRAWER_H, "door")] if drawers else [(z0, z1, "door")]
    k = 0
    for r0, r1, kind in rows:
        xs = np.cumsum([x0] + _widths(x1 - x0, DOOR_W if kind == "door" else 2 * DOOR_W))
        for i, (a, b) in enumerate(zip(xs[:-1], xs[1:])):
            piece(f"{name}_front{k}", a + GAP / 2, ya, r0 + GAP / 2, b - GAP / 2, yb, r1 - GAP / 2); k += 1
            if not handles: continue
            hy0, hy1 = sorted((y_out, y_out + side * HANDLE[0])); t = 0.006
            if kind == "drawer":
                m, zc = (a + b) / 2, (r0 + r1) / 2
                piece(f"{name}_handle{k}", m - HANDLE[1] / 2, hy0, zc - t, m + HANDLE[1] / 2, hy1, zc + t, "handle")
            else:                                                            # pairs open from the middle: the handle
                hx = b - 0.05 if i % 2 == 0 else a + 0.05                    # by the edge that meets the next door
                zt = r1 - 0.06 if drawers else r1 - 0.08                     # (base units: near the top)
                piece(f"{name}_handle{k}", hx - t, hy0, zt - HANDLE[1], hx + t, hy1, zt, "handle")


def panels(name, x0, x1, y_face, side, z0, z1):
    """applied panels without handles: an island's seating side"""
    fronts(name, x0, x1, y_face, side, z0, z1, handles=False)


def ends(name, x0, x1, y0, y1, z0, z1):
    """a panel on each exposed end, flush with the fronts"""
    for s, x in ((-1, x0), (1, x1)):
        a, b = sorted((x, x + s * PROUD))
        piece(f"{name}_end{int(s > 0)}", a, y0, z0, b, y1, z1)


def finish(seed):
    """the house's cabinet finish, from its own draw (so the house's other draws stay as they were)"""
    return random.Random(seed * 7919 + 37).choice(FINISHES)


def material_xml(seed, wood_png, tex_dir):
    """the cabinet and handle materials: the house's wood turned so its grain runs up (texture v), under paint or not"""
    name, paint = finish(seed)
    src = Path(wood_png); dst = Path(tex_dir) / f"{src.stem}_cab_{'paint' if paint else 'stain'}.png"
    if not dst.exists():
        g = np.asarray(Image.open(src).convert("RGB"), np.float32)
        lum = g.mean(-1)
        if np.abs(np.diff(lum, axis=0)).mean() > np.abs(np.diff(lum, axis=1)).mean():   # grain across: turn it upright
            g = np.rot90(g).copy(); lum = g.mean(-1)
        if paint:                                                            # paint: the grain faint, in grey
            l = lum.mean() + (lum - lum.mean()) * 0.18
            g = np.repeat(np.clip(l * (0.95 * 255 / max(l.mean(), 1)), 0, 255)[..., None], 3, -1)
        Image.fromarray(g.astype(np.uint8)).save(dst)
    rgba = f"{paint[0]:.3f} {paint[1]:.3f} {paint[2]:.3f} 1" if paint else "1 1 1 1"
    sheen = ' specular="0.35" shininess="0.5"' if paint else ' specular="0.25" shininess="0.4"'
    return (f'    <texture type="2d" name="tx_cabinet" file="{dst.name}"/>\n'
            f'    <material name="cabinet" texture="tx_cabinet" texuniform="false" texrepeat="1 1" rgba="{rgba}"{sheen}/>\n'
            f'    <material name="handle" rgba="0.62 0.62 0.60 1" specular="0.8" shininess="0.8" reflectance="0.1"/>'
            f'  <!-- cabinets: {name} -->')
