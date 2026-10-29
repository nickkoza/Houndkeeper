#!/usr/bin/env python3
"""TPU jaw pads for the Maker Arm's gripper (grasp spec [GU5], [v8J]; waymark [wjx] holds the numbers and sources):
a slip-on boot over each finger's
wedge-shaped tip, with a pad on the gripping face. Designed from the printable TPUs' datasheet values, and each design
writes the contact values the sim uses for it (patch size, twist resistance, give), so the sim grips with what the
printed part would.

Why pads: a rigid jaw meets an item at a point, and a point resists almost no twist; the twist a contact resists grows
with its patch (Xydas & Kao 1999, IJRR 18(9): contact radius ~ N^gamma, 0 <= gamma <= 1/3; the friction limit
surface is close to an ellipse whose moment axis scales with the patch).

Why a collapsible pad: a thin solid rubber layer can't bulge sideways where it's squeezed, so it acts stiffer than its
modulus (the "shape factor" of bonded rubber), and a solid 3 mm TPU face makes a patch only ~3 mm in radius on a
curved item at the gripper's squeeze. Thin ribs at +-45 deg under a thin skin bend instead (a honeycomb's
bending-dominated collapse, Gibson & Ashby, Cellular Solids: E*/E ~ (t/l)^3), so the face gives ~10-30x more and the
patch roughly doubles, bounded by the face's width.

Contact model (per jaw, est.; replace with the bench measurements in real/calibration/jaw_pads.md): the pad face is an
elastic layer (Winkler foundation, stiffness k = E_eff / thickness per unit area) pressed by an item surface of radius
R with force N: patch radius a = (4 R N / (pi k))^(1/4), capped at the face's half-width; the pressure is paraboloidal,
so the twist resisted is M = (8/15) mu N a, which is MuJoCo's torsional friction coefficient mu_t = M / N (a length).

The boot is built from the finger's own CAD (Maker Arm SDK mesh, the right finger; the left is its mirror image to
1.2 mm, and each finger is symmetric side to side, so one pad fits both): the finger is sliced every SLICE mm, each
slice's convex outline grown by the wall (the pad's thickness on the gripping face) and shrunk by the interference
fit, and neighbouring slices lofted. The open end goes up the finger to just above the 2.4 mm step behind the flat
face, so the boot's top lip hooks over that step and the item's weight can't pull it off the tip.
Printed open end down, tip up: every wall stands vertical or leans inward less than 45 deg; small bridges only.
Usage: build_pads.py [out_dir]   (dimos venv: manifold3d, trimesh, shapely)"""
import re
import sys
from pathlib import Path
import numpy as np
import trimesh
import yaml
from manifold3d import CrossSection, Manifold, OpType
from scipy.spatial.transform import Rotation
from shapely.affinity import translate
from shapely.geometry import LineString, Polygon, box as sbox
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from printcheck import printable   # noqa: E402

HK = Path(__file__).resolve().parents[2]
SDK_MESHES = HK / "models" / "maker_arm" / "meshes"          # the SDK's meshes, vendored (models/README.md)
ARM_XML = HK / "sim/arm/maker_arm.xml"
OUT = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parent)

WALL = 1.2         # mm, sides and back: a stretch sleeve, not a gripping surface
CAP = 1.5          # mm, over the fingertip
INTERF = 0.15      # mm per side smaller than the finger: a snug slip fit in TPU (est.; revise after a test print)
TOP = -141.0       # mm (jaw frame, z down the finger): just above the step behind the flat face at about -145
SLICE = 0.5        # mm between slices

# Printable TPUs, Young's modulus in the print plane (the pad's thickness runs in the print plane, as printed tip-up)
TPU = {
    "bambu_95a_hf": {"E_MPa": 9.8, "src": "Bambu TPU 95A HF datasheet v1.0, ISO 527: 9.8 +- 0.7 MPa X-Y (7.4 Z); overhang ~55 deg"},
    "bambu_85a":    {"E_MPa": 6.8, "src": "Bambu TPU 85A datasheet v1.0, ISO 527: 6.8 +- 0.7 MPa X-Y (5.2 Z); overhang ~40 deg"},
    "cheetah_95a":  {"E_MPa": 26.2, "src": "NinjaTek Cheetah datasheet, ASTM D638 tensile modulus 3800 psi"},
    "ninjaflex_85a": {"E_MPa": 12.4, "src": "NinjaTek NinjaFlex datasheet, ASTM D638 tensile modulus 1800 psi"},
}
PARAMS = yaml.safe_load((HK / "params/robot.yaml").read_text())["maker_arm"]   # the robot parameters ([J6A])
MU_TPU = PARAMS["pad_tpu_mu"]["value"]        # sliding friction, TPU on household items (est.; measure it)
SQUEEZE_N = PARAMS["sim_squeeze_N"]["value"]  # each jaw's squeeze in the sim (the real one is unknown)
ITEM_R_MM = 20.0   # a typical curved item surface under the jaw (est.: a shoe's side, a toy's body)

DESIGNS = {        # pad_t: face thickness (mm); core: None = solid, else (rib thickness mm, skin mm, rib angle deg)
    "v1": {"pad_t": 3.0, "core": None, "tpu": "bambu_95a_hf", "note": "solid face"},
    "v2": {"pad_t": 4.0, "core": (0.8, 0.8, 45.0), "tpu": "bambu_95a_hf", "note": "zigzag ribs under a skin"},
    "v3": {"pad_t": 3.0, "core": (0.8, 0.8, 45.0), "tpu": "bambu_95a_hf", "note": "zigzag ribs, 3 mm: the one to print ([NKb])"},
}
PRINT = "v3"       # pencilled in 25 Sep 2026: most of v2's grip for v1's cost in opening; revisit with the bench test


def contact(d, half_width_mm):
    """the design's expected contact on a curved item at the sim's squeeze: E_eff (MPa), patch radius a (mm), how far
    the face gives (mm), and MuJoCo's torsional coefficient (m)"""
    E = TPU[d["tpu"]]["E_MPa"]; t = d["pad_t"]
    if d["core"] is None:                                    # solid, bonded to the sleeve: stiffened by its shape factor
        E_eff, h = E, t                                      # (S = a / 2t for a patch of radius a), solved with a below
    else:
        rib, skin, ang = d["core"]; h = t - skin - 0.8       # the ribs span the gap between the sleeve's face wall and the skin
        l = h / np.sin(np.radians(ang)); E_eff = E * (rib / l) ** 3   # bending-dominated (Gibson & Ashby)
    a = 3.0
    for _ in range(30):
        Ec = E_eff * (1 + 2 * (a / (2 * h)) ** 2) if d["core"] is None else E_eff
        k = Ec * 1e6 / (h / 1000)                            # N/m^3
        a = min((4 * ITEM_R_MM / 1000 * SQUEEZE_N / (np.pi * k)) ** 0.25 * 1000, half_width_mm)
    give = min(a ** 2 / (2 * ITEM_R_MM), h * 0.8)
    return {"E_eff_MPa": E_eff, "a_mm": a, "give_mm": give, "mu_t_m": 8 / 15 * MU_TPU * a / 1000}


def finger():
    """the right finger's long part in the jaw body's frame, mm (as the MJCF places the SDK's STL)"""
    x = ARM_XML.read_text(); b = x[x.index('<body name="gripper_right">'):]; b = b[:b.index("</body>")]
    g = re.search(r'<geom pos="([^"]+)" quat="([^"]+)"', b); t = np.array(g.group(1).split(), float) * 1000
    w, qx, qy, qz = map(float, g.group(2).split()); R = Rotation.from_quat([qx, qy, qz, w]).as_matrix()
    for n in sorted(set(re.findall(r'mesh="(part_\d+_solid_\d+)"', b))):
        m = trimesh.load(SDK_MESHES / f"{n}.stl"); v = m.vertices @ R.T + t
        if np.ptp(v[:, 2]) > 70: return trimesh.Trimesh(v, m.faces)           # the 80 mm finger, not the carriage parts
    raise SystemExit("finger mesh not found")


def section(F, z):
    s = F.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if s is None: return None
    polys = [Polygon(np.asarray(e)[:, :2]) for e in s.discrete]
    return unary_union([p.buffer(0) for p in polys if p.area > 1e-3])


def rings(d):
    """per slice: (z, outside, cavity, core) -- the boot's outline, the finger's (less the fit), and for a ribbed pad the
    space in front of the gripping face that the ribs fill (inside the sleeve's face wall and the skin)"""
    F = finger(); zt = F.bounds[0][2]; t = d["pad_t"]; out = []
    for z in list(np.arange(TOP, zt + 0.3, -SLICE)) + [zt + 0.3]:
        P = section(F, z)
        if P is None or P.is_empty: continue
        H = P.convex_hull; S = H.buffer(WALL, join_style=2)
        O = unary_union([S, translate(S, yoff=t - WALL)]).convex_hull          # the sleeve pushed forward: t thick in front
        core = None
        if d["core"]:
            rib, skin, _ = d["core"]; x0, _, x1, _ = O.bounds
            core = sbox(x0 + skin, 0.8, x1 - skin, t - skin)                   # y = 0 is the finger's gripping face
        out.append((z, O, H.buffer(-INTERF, join_style=2), core))
    return out, zt


def loft(rs):
    pts = lambda z, g: np.c_[np.asarray(g.exterior.coords)[:-1], np.full(len(g.exterior.coords) - 1, z)]
    return Manifold.batch_boolean([Manifold.hull_points(np.vstack([pts(*a), pts(*b)])) for a, b in zip(rs, rs[1:])], OpType.Add)


def zigzag(d, x0, x1):
    """the ribs' outline: a zigzag across the face at +-angle, rib-thick, spanning the core's depth"""
    rib, skin, ang = d["core"]; y0, y1 = 0.8, d["pad_t"] - skin; run = (y1 - y0) / np.tan(np.radians(ang))
    xs = np.arange(x0 - run, x1 + 2 * run, run); ys = [y0 if k % 2 == 0 else y1 for k in range(len(xs))]
    Z = LineString(list(zip(xs, ys))).buffer(rib / 2, cap_style=2, join_style=2)
    g = orient(Z.intersection(sbox(x0 - 1, y0 - 0.2, x1 + 1, y1 + 0.2)), 1.0)
    geoms = getattr(g, "geoms", [g])
    return CrossSection([np.asarray(p.exterior.coords)[:-1] for p in geoms if p.area > 0.01])


def pad(d):
    rs, zt = rings(d)
    outs = [(z, O) for z, O, _, _ in rs] + [(zt - CAP, rs[-1][1])]           # the cap over the tip
    ins = [(TOP + 1.0, rs[0][2])] + [(z, C) for z, _, C, _ in rs]           # the cavity opens through the top
    m = loft(outs) - loft(ins)
    if d["core"]:                                                            # hollow the face and fill it with ribs, which
        cores = [(z, c) for z, _, _, c in rs if c is not None and z < TOP - 2]   # stay closed at the tip (under the cap) and
        core = loft(cores)                                                   # open 2 mm short of the top (a closed rim)
        x0, x1 = min(c.bounds[0] for _, c in cores), max(c.bounds[2] for _, c in cores)
        ribs = Manifold.extrude(zigzag(d, x0, x1), TOP - zt + 10).translate([0, 0, zt - 5])
        m = (m - core) + (ribs ^ core)
    return m.mirror([0, 0, 1])                                               # open end down, tip up, for printing


def sim_file(name, d, band=8.0):
    """the pad's outside as convex pieces for the sim (MuJoCo collides meshes by their convex hulls), in the right jaw
    body's frame, metres, with its contact values; written as <name>_sim.npz, read by sim/arm/go1_arm.py"""
    rs, zt = rings(d)
    pts = [np.c_[np.asarray(O.exterior.coords)[:-1], np.full(len(O.exterior.coords) - 1, z)] for z, O, _, _ in rs]
    pts.append(np.c_[pts[-1][:, :2], np.full(len(pts[-1]), zt - CAP)])
    zs = np.array([p[0, 2] for p in pts]); pieces = []
    for z0 in np.arange(TOP, zt - CAP - 1e-6, -band):                         # overlapping bands, each the hull of its slices
        pick = [p for p, z in zip(pts, zs) if z0 - band - SLICE <= z <= z0 + 1e-6]
        if len(pick) >= 2: pieces.append(np.vstack(pick) / 1000.0)
    half_w = min((O.bounds[2] - O.bounds[0]) / 2 for z, O, _, _ in rs if -185 < z < -165)   # around the grasp centre
    c = contact(d, half_w)
    np.savez_compressed(OUT / f"{name}_sim.npz", pad_t=d["pad_t"] / 1000.0, mu=MU_TPU, mu_t=c["mu_t_m"],
                        give=c["give_mm"] / 1000.0, **{f"hull_{k}": v for k, v in enumerate(pieces)})
    return pieces, c


def export(m, name):
    mesh = m.to_mesh()                                      # as manifold made it: merging its coincident vertices would pinch edges
    t = trimesh.Trimesh(vertices=mesh.vert_properties[:, :3], faces=mesh.tri_verts, process=False)
    assert t.is_watertight, name
    t.apply_translation(-t.bounds[0]); br, sl = printable(t, name)
    t.export(OUT / f"{name}.stl"); e = t.extents
    print(f"{name}.stl  {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm, {t.volume / 1000:.1f} cm3; bridges {br:.2f} cm2, steep overhang {sl:.2f} cm2; fits, prints flat")
    return t


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    for v, d in DESIGNS.items():
        name = f"jaw_pad_{v}"; export(pad(d), name); pieces, c = sim_file(name, d)
        print(f"  {d['note']}, {d['tpu']}: face gives {c['give_mm']:.1f} mm, patch radius {c['a_mm']:.1f} mm, "
              f"twist resisted {c['mu_t_m'] * SQUEEZE_N * 1000:.1f} N mm per jaw at {SQUEEZE_N} N (sim: HK_JAWS=pads_{v})")
    print("\nThe same designs in the other common TPUs (patch radius mm / twist N mm per jaw):")
    for v, d in DESIGNS.items():
        hw = 10.0
        print(f"  {v}: " + ", ".join(f"{k} {contact({**d, 'tpu': k}, hw)['a_mm']:.1f}/{contact({**d, 'tpu': k}, hw)['mu_t_m'] * SQUEEZE_N * 1000:.1f}" for k in TPU))
    print(f"\nTo print: jaw_pad_{PRINT}.stl, two of them, in {DESIGNS[PRINT]['tpu']}, open end down, 100% infill, 0.4 mm nozzle. "
          "In the slicer's preview check the small triangular voids under the gripping face stay open (turn off gap fill if "
          "they're filled): they are what lets the face collapse. Slide one onto each fingertip until the top lip clicks "
          "over the step behind the flat face. Each jaw's opening shrinks by the pad's thickness.")
