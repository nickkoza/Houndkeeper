"""Estimates for the printed mount, before anything is printed: mass ([mgb]), the adapter's stiffness and stress by a beam
model ([lUO] [uqL]), the load cases for the finite-element analysis ([vqz] [tY3]), the magnets ([Frm] [4JP]) and the
static clearances against the real models ([Uaq]). Geometry only: nothing is stepped."""
import math
from pathlib import Path
import numpy as np
import config as C

R = C.R
G = 9.81


# ---------------------------------------------------------------- meshes
def to_mesh(solid, tol=0.2):
    import trimesh
    v, t = solid.tessellate(tol, 0.3)
    return trimesh.Trimesh(np.array([[p.X, p.Y, p.Z] for p in v]), np.array(t), process=True)


# ---------------------------------------------------------------- mass ([mgb])
def mass_g(solid, density, infill=C.INFILL):
    """printed mass: every face carries a skin (walls on the sides, top/bottom layers on the level faces, each capped so
    thin ribs count as solid), the rest is sparse infill at `infill`"""
    from build123d import Vector
    skin = 0.0
    for f in solid.faces():
        n = f.normal_at(f.center())
        t = C.SKIN_TB if abs(n.Z) > 0.7 else C.SKIN_SIDE
        skin += f.area * t
    V = solid.volume; skin = min(skin * 0.92, V)              # 0.92: the skins overlap at edges and around holes (est.)
    return density * (skin + infill * (V - skin)) / 1000.0, skin / V


# ---------------------------------------------------------------- arm, from the sim's model, in the robot frame
def _arm():
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(R / "sim/arm/maker_arm.xml")); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    Rup = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])
    zmin = 1e9
    for i in range(m.ngeom):
        if m.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH: continue
        mid = m.geom_dataid[i]; V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        W = (V @ d.geom_xmat[i].reshape(3, 3).T + d.geom_xpos[i]) @ Rup.T
        zmin = min(zmin, W[:, 2].min())
    return m, d, Rup, zmin


def arm_to_robot(P, Rup, zmin):
    """sim arm frame (m) -> robot frame (mm): as tools/export_models.py places the folded arm, turned 180 deg, base at
    [Pvh] on the module's top"""
    Q = (np.asarray(P) @ Rup.T); Q = Q - [0, 0, zmin]
    Q = Q * [-1, -1, 1]
    return Q * 1000.0 + [C.MOUNT_X, 0, C.Z_BASE]


def arm_numbers(samples=6000, seed=0):
    """the stowed arm's centre of mass and its motors (encoders) in the robot frame; the largest horizontal lever of
    the arm's own mass over its joint ranges (the full-reach case)"""
    import mujoco
    m, d, Rup, zmin = _arm()
    bodies = [b for b in range(m.nbody) if m.body_mass[b] > 0 and mujoco.mj_id2name(m, 1, b) != "world"]
    moving = [b for b in bodies if mujoco.mj_id2name(m, 1, b) != "base_link"]
    def com(ids):
        M = sum(m.body_mass[b] for b in ids)
        return sum(m.body_mass[b] * d.xipos[b] for b in ids) / M, M
    c_all, M_all = com(bodies)
    stow_com = arm_to_robot(c_all, Rup, zmin)
    joints = {mujoco.mj_id2name(m, 3, j): arm_to_robot(d.xanchor[j], Rup, zmin) for j in range(m.njnt) if m.jnt_type[j] == 3}
    base_axis = arm_to_robot(d.xpos[m.body("base_link").id], Rup, zmin)
    rng = np.random.default_rng(seed); best = 0.0
    hinge = [j for j in range(m.njnt) if m.jnt_type[j] == 3]
    for _ in range(samples):
        for j in hinge:
            lo, hi = m.jnt_range[j]; d.qpos[m.jnt_qposadr[j]] = rng.uniform(lo, hi)
        mujoco.mj_kinematics(m, d)
        c, Mm = com(moving); P = arm_to_robot(c, Rup, zmin)
        best = max(best, Mm * math.hypot(P[0] - C.MOUNT_X, P[1]) / 1000.0)
    mujoco.mj_resetData(m, d); mujoco.mj_forward(m, d)
    model_mass = float(sum(m.body_mass[b] for b in bodies))
    scale = C.ARM_MASS / model_mass                                         # the URDF is 9% light: carry the real 3.7 kg
    return dict(model_mass_kg=round(model_mass, 2), stow_com_mm=[round(float(v), 1) for v in stow_com],
                max_self_moment_Nm=round(best * scale * G, 2), joints_mm={k: [round(float(v), 1) for v in p] for k, p in joints.items()},
                base_axis_mm=[round(float(v), 1) for v in base_axis])


# ---------------------------------------------------------------- load cases ([vqz], for [tY3])
def load_cases(arm):
    m = C.ARM_MASS; Wa = m * G; Wp = C.PAYLOAD * G
    M_reach = arm["max_self_moment_Nm"] + Wp * C.REACH / 1000.0           # the arm stretched out with 1.5 kg at the tip
    com = arm["stow_com_mm"]
    base = {"point_mm": [C.MOUNT_X, 0.0, C.Z_BASE], "what": "the arm's base: its four M5 on the module's top (79 mm square)"}
    cases = {
        "sustained_stowed": dict(factor=1.0, F_N=[0, 0, -Wa], at_mm=com, kind="sustained (creep)",
                                 note="the folded arm's weight, always on the mount"),
        "working_full_reach": dict(factor=1.5, F_N=[0, 0, -(Wa + Wp)], M_Nm_about_Y=M_reach, M_Nm_about_X=M_reach, at_mm=base["point_mm"],
                                   kind="working peak", note="robot lying (the arm moves only stopped and down); arm stretched out at its "
                                   "measured 0.67 m reach with the rated 1.5 kg at the tip, forward/back (about Y) or to the side (about X); "
                                   "x1.5 for the arm's own accelerations (est.)"),
        "walking_stowed": dict(factor_vertical_peak=2.0, factor_vertical_range=2.0, factor_lateral=0.5, factor_fore_aft=0.5, F_N_per_g=[0, 0, -Wa],
                               at_mm=com, kind="fatigue (range) and peak", note="each step: vertical 0 to 2 g on the stowed arm (range 2 g), "
                               "0.5 g sideways and fore-aft (est., trot); 1e7 cycles design life ([Xtj])"),
        "lying_and_standing": dict(factor_vertical=1.5, tilt_deg=20.0, F_N_per_g=[0, 0, -Wa], at_mm=com, kind="working peak",
                                   note="lying down and standing up with the arm stowed: 1.5 g and a 20 deg pitch (est.)"),
        "fall_onto_back": dict(factor=8.0, F_N_per_g=[0, 0, Wa], at_mm=com, kind="one-off (fall)",
                               note="the robot rolls onto its back: the stowed arm's mass at 8 g, pushing the arm toward the body "
                                    "(est.: ~0.3 m drop onto a rug or hard floor with ~4 mm of give at the arm); the arm may scuff, "
                                    "the mount must not let go of it"),
    }
    bcs = {
        "adapter": {"fixed": [dict(what=f"{C.REAR_BOLT} bolt through the cover hole (assumed thread)", xy_mm=xy, z_mm=C.COVER_Z) for xy in C.BACK["rear_holes_xy"]],
                    "contact_compression_only": dict(what="underside on the top cover (Z 57)", region_x_mm=[C.ADP_X[0], 75.0], half_width_mm=C.ADP_HALF_W_FOOT),
                    "loads_from": "the module, through the interface (bolts + dowels + bearing on the deck); the bin on its magnets"},
        "module": {"joined_to": "adapter or bench_plate at the interface", "loads_at": base, "electronics_kg": round(sum(C.HARDWARE_G.values()) / 1000, 3)},
        "bench_plate": {"fixed": [dict(what="M8 to the bench (or C-clamp on the flange)", xy_mm=xy) for xy in C.BENCH_HOLES],
                        "loads_from": "the module, as on the robot"},
    }
    joints = [dict(name="module-adapter (and module-bench_plate)", plane_z_mm=C.Z_IF,
                   bolts=[dict(size="M5", into="M5 heat-set insert (Ruthex, 6.4 x 9.5)", xy_mm=xy) for xy in C.IF_BOLTS],
                   dowels=[dict(d_mm=C.DOWEL_D, xy_mm=C.IF_DOWELS[0], fit="pressed in the adapter; round hole in the module"),
                           dict(d_mm=C.DOWEL_D, xy_mm=C.IF_DOWELS[1], fit="pressed in the adapter; slot along Y in the module")],
                   bearing="the module's underside on the deck, compression only, X 0..112"),
              dict(name="arm base-module", plane_z_mm=C.Z_BASE, bolts=[dict(size="M5", into="the arm base's nut traps", xy_mm=[C.MOUNT_X + x, y])
                                                                        for x, y in C.ARM_BASE["m5_nut_traps_xy"]]),
              dict(name="adapter-robot", bolts=[dict(size=C.REAR_BOLT + " (assumed)", xy_mm=xy) for xy in C.BACK["rear_holes_xy"]]),
              dict(name="bin-adapter (breakaway)", magnets=[dict(xy_mm=xy, magnet=C.MAGNET["src"]) for xy in C.MAGNETS_XY],
                   locators=[dict(xy_mm=xy, ball_d_mm=C.CONE["ball_d"], socket_half_angle_deg=C.CONE["half_angle_deg"]) for xy in C.CONES_XY])]
    return dict(frame="robot frame, mm: X forward, Y left, Z up, origin at the Go2's body centre", g=G, arm_mass_kg=m,
                payload_kg=C.PAYLOAD, reach_mm=C.REACH, arm=arm, cases=cases, boundary_conditions=bcs, joints=joints,
                pieces="no piece needs splitting: each fits the X1's 256 mm cube with margin (see report), so no printed joints "
                       "inside a part; the joints above are the assembly's",
                layers={"adapter": "printed upside down, deck on the bed: layers are XY planes, so bending along X (the arm's moment) "
                                   "and the twist both run in the layers; the bolt clamp and the deck's bearing press across them",
                        "module": "printed top-down: the same; the arm base's bolts pull across the layers (6 mm of top skin)",
                        "bench_plate": "printed top-down: as the module",
                        "bin": "printed upright: walls along Z; a knock bends the walls across the layers (PETG or PC, low stress)"},
                status="estimates, 4 Oct 2026; the printed parts' measured stiffness replaces them ([lUO])")


# ---------------------------------------------------------------- section stiffness of the adapter ([lUO], [uqL])
def section(mesh, x, pix=0.25):
    """the skin-only section of a part (or several, bonded) at X = x (printed solid walls and skins carry load; sparse
    infill is not credited, [Xtj]): area, centroid, I about Y (vertical bending), I about Z, J (closed thin-walled
    cell), extreme fibre"""
    from shapely.geometry import Polygon as SP
    from shapely.ops import unary_union
    from scipy import ndimage
    from PIL import Image, ImageDraw
    M = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, -x], [0, 0, 0, 1.0]])   # world -> (Y, Z) on the plane
    shapes = []
    for m in (mesh if isinstance(mesh, (list, tuple)) else [mesh]):
        sec = m.section(plane_origin=[x, 0, 0], plane_normal=[1, 0, 0])
        if sec is None: continue
        planar, _ = sec.to_2D(to_2D=M)
        shapes += [p.buffer(0) for p in planar.polygons_full]
    if not shapes: return None
    shape = unary_union(shapes)
    if shape.area < 1: return None
    y0, z0, y1, z1 = shape.bounds; W = int((y1 - y0) / pix) + 4; H = int((z1 - z0) / pix) + 4
    img = Image.new("L", (W, H), 0); dr = ImageDraw.Draw(img)
    geoms = getattr(shape, "geoms", [shape])
    for g in geoms:
        dr.polygon([((yy - y0) / pix + 2, (zz - z0) / pix + 2) for yy, zz in g.exterior.coords], fill=1)
        for hole in g.interiors:
            dr.polygon([((yy - y0) / pix + 2, (zz - z0) / pix + 2) for yy, zz in hole.coords], fill=0)
    A = np.array(img, bool)                                    # rows = z, cols = y
    dist = ndimage.distance_transform_edt(A, sampling=(pix / C.SKIN_TB, pix / C.SKIN_SIDE))
    skin = A & (dist <= 1.0)
    zz, yy = np.nonzero(skin); Y = y0 + (yy - 2) * pix; Z = z0 + (zz - 2) * pix; dA = pix * pix
    area = skin.sum() * dA; zc = Z.mean(); yc = Y.mean()
    Iy = ((Z - zc) ** 2).sum() * dA; Iz = ((Y - yc) ** 2).sum() * dA
    c = max(z1 - zc, zc - z0)
    # torsion: the outer skin as one closed cell (Bredt): J = 4 Am^2 / sum(ds / t)
    outer = max(geoms, key=lambda g: g.area); Am = SP(outer.exterior).buffer(-min(C.SKIN_TB, C.SKIN_SIDE) / 2).area
    pts = np.asarray(outer.exterior.coords); seg = np.diff(pts, axis=0); L = np.hypot(seg[:, 0], seg[:, 1])
    vert = np.abs(seg[:, 1]) > np.abs(seg[:, 0])
    s_over_t = (L[vert] / C.SKIN_SIDE).sum() + (L[~vert] / C.SKIN_TB).sum()
    J = 4 * Am ** 2 / s_over_t
    return dict(x=x, area=area, zc=zc, Iy=Iy, Iz=Iz, J=J, c=c, depth=z1 - z0, width=y1 - y0)


def stiffness(adapter_mesh, materials, module_mesh=None, x_fix=None, x_load=None, step=4.0):
    """[uqL] The adapter as a cantilever fixed at the front row of the rear bolts and loaded at the arm's base:
    rotation per unit moment = sum dx / (E I) (bending about Y: the arm reaching fore/aft) and sum dx / (G J) (twist
    about X: reaching sideways). Two bounds: the adapter alone (low: the module above, which stiffens X 14..112, and
    the cover's bearing under the deck are left out) and the adapter and module as one section from the module's rear
    bolts forward (high: as if their joint did not slip). Neither includes the give of the rear bolts and the cover
    themselves (unknown: measured on the robot)."""
    import trimesh
    x_fix = max(x for x, _ in C.BACK["rear_holes_xy"]) if x_fix is None else x_fix
    x_load = C.MOUNT_X if x_load is None else x_load
    xs = np.arange(x_fix + step / 2, x_load, step)
    secs = [s for s in (section(adapter_mesh, x) for x in xs) if s]
    bounds = {"adapter_alone": secs}
    if module_mesh is not None:
        both = [adapter_mesh, module_mesh]; x_join = min(x for x, _ in C.IF_BOLTS)
        bounds["adapter_and_module_bonded"] = [s if s["x"] < x_join else (section(both, s["x"]) or s) for s in secs]
    out = {"x_fix_mm": x_fix, "x_load_mm": x_load, "materials": {}}
    for name, ss in bounds.items():
        inv_I = sum(step / s["Iy"] for s in ss); inv_J = sum(step / s["J"] for s in ss)
        for k, mt in materials.items():
            E = mt["E_xy_design"]; Gm = E / (2 * (1 + C.POISSON)); E_st = mt["E_xy_short_40C"]
            kb = 1.0 / (inv_I / E) / 1000.0; kt = 1.0 / (inv_J / Gm) / 1000.0                  # N m / rad
            out["materials"].setdefault(k, {"E_design_MPa": E})[name] = dict(
                k_bend_Nm_per_rad=round(kb), k_twist_Nm_per_rad=round(kt),
                k_bend_short_term_Nm_per_rad=round(kb * E_st / E), k_twist_short_term_Nm_per_rad=round(kt * E_st / E))
    out["sections"] = [{k: round(float(v), 1) for k, v in s.items()} for s in secs]
    return out


def jaw_error(stiff, arm):
    """how far the jaws move at full reach under the working static moment, from the mount's flex alone"""
    M = arm["max_self_moment_Nm"] + C.PAYLOAD * G * C.REACH / 1000.0
    out = {"moment_Nm": round(M, 1)}
    for k, v in stiff["materials"].items():
        for b in ("adapter_alone", "adapter_and_module_bonded"):
            if b not in v: continue
            th = M / v[b]["k_bend_Nm_per_rad"]
            out.setdefault(k, {})[b] = dict(deg=round(math.degrees(th), 2), jaw_mm_at_reach=round(th * C.REACH, 1))
    th = M / C.SIM_FLEX; out["sim_now"] = dict(deg=round(math.degrees(th), 2), jaw_mm_at_reach=round(th * C.REACH, 1))
    return out


def stresses(stiff, cases, arm, materials):
    """bending stress in the adapter's skins, the largest along it (M(x) c / I at each section between the bolts and the
    arm), per case, against [Xtj]'s allowables (bend_xy: the load runs along the layers)"""
    secs = stiff["sections"]; com = arm["stow_com_mm"]; Wa = C.ARM_MASS * G
    def peak(F_N, at, M_extra_Nmm=0.0):
        return max(abs(F_N * (at[0] - s["x"]) + M_extra_Nmm) * s["c"] / s["Iy"] for s in secs if s["x"] < at[0] or M_extra_Nmm)
    work = cases["working_full_reach"]
    base = [C.MOUNT_X, 0, C.Z_BASE]
    sigma = dict(sustained=peak(Wa, com),
                 working_peak=work["factor"] * peak((C.ARM_MASS + C.PAYLOAD) * G, base, work["M_Nm_about_Y"] * 1000),
                 walking_range=peak(Wa * cases["walking_stowed"]["factor_vertical_range"], com),
                 walking_peak=peak(Wa * cases["walking_stowed"]["factor_vertical_peak"], com),
                 fall=peak(Wa * cases["fall_onto_back"]["factor"], com))
    out = {"sigma_MPa": {k: round(v, 2) for k, v in sigma.items()}, "against": {}}
    for k, mt in materials.items():
        a = mt["allow"]["bend_xy"]
        out["against"][k] = dict(sustained=f'{sigma["sustained"]:.2f} of {a["sustained_allow_MPa"]}',
                                 working_peak=f'{sigma["working_peak"]:.2f} of {a["working_peak_allow_MPa"]}',
                                 walking_range_fatigue=f'{sigma["walking_range"]:.2f} of {a["fatigue_range_allow_1e7_MPa"]}',
                                 fall=f'{sigma["fall"]:.2f} of {a["fall_allow_MPa"]}',
                                 ok=bool(sigma["sustained"] <= a["sustained_allow_MPa"] and sigma["working_peak"] <= a["working_peak_allow_MPa"]
                                         and sigma["walking_range"] <= a["fatigue_range_allow_1e7_MPa"] and sigma["fall"] <= a["fall_allow_MPa"]))
    out["note"] = ("beam estimate in the adapter only, away from holes and corners; the FEA ([tY3]) finds the peaks at "
                   "the bolts, inserts and the deck's corners, where these will be several times higher")
    return out


# ---------------------------------------------------------------- magnets ([Frm], [4JP])
def disc_field_mT(z_mm, d=C.MAGNET["d"], h=C.MAGNET["h"], Br=1.3):
    """on-axis field of a disc magnet (N42, Br ~1.3 T) at z from its face: the worst direction; a pair stacked counts as
    one magnet twice as tall"""
    Rm = d / 2; z = z_mm
    return 1000 * Br / 2 * ((z + h) / math.hypot(z + h, Rm) - z / math.hypot(z, Rm))


def magnets(arm):
    m = C.MAGNET; held = m["pull_N"] * C.MAGNET_GAP_DERATE
    pull = held * len(C.MAGNETS_XY)
    mu = C.CONE["mu"]; a = math.radians(C.CONE["half_angle_deg"])
    bin_kg = 0.15; toys_kg = 0.6                                          # the bin (est. from its print) and a load of toys (est.)
    W = (bin_kg + toys_kg) * G
    lateral = (pull + W) * math.tan(a + math.atan(mu))                    # riding up a 45 deg cone, with friction
    walk_lift = (bin_kg + toys_kg) * G * (2.0 - 1.0)                      # at 2 g, the bin's own inertia pulls it up by 1 g
    pts = [("bin", (x, y, C.Z_BIN)) for x, y in C.MAGNETS_XY] + [("hat", (*C.HAT["centre"], C.Z_BASE + C.HAT["flange_h"] + C.HAT["boss_h"]))]
    enc = arm["joints_mm"]; imu = (-25.57, 0.0, 42.32)                     # Menagerie Go2's imu site (models/go2/go2.xml)
    dists = {}
    for name, p in pts:
        dmin = min((math.dist(p, q), j) for j, q in enc.items())
        dists.setdefault(name, []).append(dict(at=p, nearest_encoder=dmin[1], encoder_mm=round(dmin[0], 1),
                                               imu_mm=round(math.dist(p, imu), 1)))
    def worst(name):
        e = min(d["encoder_mm"] for d in dists[name]); i = min(d["imu_mm"] for d in dists[name])
        return dict(encoder_mm=e, field_at_encoder_mT=round(disc_field_mT(e, h=2 * m["h"]), 3), imu_mm=i,
                    field_at_imu_mT=round(disc_field_mT(i, h=2 * m["h"]), 3))
    hat_W = C.HAT["mass_g"] / 1000 * G
    return dict(magnet=m["src"], gap_derate=C.MAGNET_GAP_DERATE, held_per_pair_N=round(held, 1),
                bin=dict(pairs=len(C.MAGNETS_XY), pull_off_N=round(pull + W, 1), lateral_release_N=round(lateral, 1),
                         walking_lift_N=round(walk_lift, 1), margin_walking=round(pull / walk_lift, 1),
                         repeat="two balls in the deck, a 45 deg cone socket and a V-slot along X in the bin: back within ~0.3 mm (est., print tolerance)",
                         **worst("bin")),
                hat=dict(pairs=1, held_N=round(held, 1), weight_N=round(hat_W, 2), margin_at_2g=round(held / (hat_W * 2.0), 1),
                         note="hat mass est. 40 g; lateral: the boss locates it", **worst("hat")),
                earth_field_mT=0.05, detail=dists)


# ---------------------------------------------------------------- clearances ([Uaq])
def go2_legs(pose):
    """the Menagerie Go2's legs (hips, thighs, calves; the same robot, articulated) in the body frame, mm"""
    import mujoco, trimesh
    m = mujoco.MjModel.from_xml_path(str(R / "models/go2/go2.xml")); d = mujoco.MjData(m)
    q = {"hip": 0.0, "thigh": 0.9, "calf": -1.8}; q.update(pose)
    for leg in ("FL", "FR", "RL", "RR"):
        d.qpos[m.jnt_qposadr[m.joint(f"{leg}_hip_joint").id]] = q["hip"] * (1 if leg[1] == "L" else -1)
        d.qpos[m.jnt_qposadr[m.joint(f"{leg}_thigh_joint").id]] = q["thigh"]
        d.qpos[m.jnt_qposadr[m.joint(f"{leg}_calf_joint").id]] = q["calf"]
    mujoco.mj_kinematics(m, d)
    base = m.body("base").id; bo = d.xpos[base]; bR = d.xmat[base].reshape(3, 3)
    parts = []
    for i in range(m.ngeom):
        if m.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH or m.geom_bodyid[i] == base: continue
        if m.geom_contype[i] or m.geom_conaffinity[i]: continue
        mid = m.geom_dataid[i]; V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        W = ((V @ d.geom_xmat[i].reshape(3, 3).T + d.geom_xpos[i]) - bo) @ bR
        parts.append(trimesh.Trimesh(W * 1000, m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid] + m.mesh_facenum[mid]], process=False))
    return trimesh.util.concatenate(parts)


def _surface_points(m, spacing):
    n = int(min(400000, max(2000, m.area / spacing ** 2)))
    return np.vstack([m.vertices, m.sample(n)])


def min_dist(a, b, n=None, spacing=1.0, reach=60.0):
    """smallest distance between the surfaces of meshes a and b, mm, from dense surface samples (resolution ~spacing);
    only the parts of each within `reach` of the other's box are compared"""
    from scipy.spatial import cKDTree
    if len(a.faces) == 0 or len(b.faces) == 0: return float("inf")
    lo = np.maximum(a.bounds[0], b.bounds[0]) - reach; hi = np.minimum(a.bounds[1], b.bounds[1]) + reach
    if np.any(lo > hi): return float("inf")
    def crop(m):
        P = _surface_points(m, spacing); keep = np.all((P >= lo) & (P <= hi), axis=1); return P[keep]
    pa, pb = crop(a), crop(b)
    if not len(pa) or not len(pb): return float("inf")
    d, _ = cKDTree(pb).query(pa, k=1)
    return float(d.min())


def arm_stow_mesh():
    import trimesh
    f = R / "models/export/maker_arm_folded_mm.stl"
    if not f.exists(): return None
    a = trimesh.load(f); a.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [0, 0, 1]))
    a.apply_translation([C.MOUNT_X, 0, C.Z_BASE]); return a


POSES = {"standing": {}, "lying": {"thigh": 1.25, "calf": -2.72},
         "walking_thigh_back": {"thigh": -0.3, "calf": -1.2, "hip": 0.3}, "walking_thigh_fwd": {"thigh": 1.6, "calf": -2.4, "hip": -0.3}}


def clearances(meshes, arm_mesh):
    """minimum gaps (mm) from each part to the Go2's body (Unitree's CAD, dock removed), its legs in each pose, the
    folded arm; the adapter's underside against the cover; the head camera's and lidar's view; the battery and port"""
    import trimesh
    import go2_ref as g
    out = {}
    body = g.mesh(exclude=g.DOCK + tuple(i["id"] for i in g.index() if _is_leg(i)))
    out["body_source"] = "Unitree Go2 CAD 0911 (dock removed) for body/head/battery; Menagerie Go2 legs, posed"
    # adapter underside vs the cover: rays up from below inside its footprint
    adp = meshes["adapter"]; xs = np.arange(C.ADP_X[0] + 2, C.ADP_X[1], 4.0); ys = np.arange(-C.ADP_HALF_W + 2, C.ADP_HALF_W, 4.0)
    O = np.array([[x, y, 0.0] for x in xs for y in ys]); D = np.tile([0, 0, 1.0], (len(O), 1))
    loc, ir, _ = adp.ray.intersects_location(O, D, multiple_hits=False)
    under = {i: z for i, z in zip(ir, loc[:, 2])}
    top = g.top_z(body, xs, ys).ravel()
    gaps = [under[i] - top[i] for i in under if not np.isnan(top[i])]
    out["adapter_on_cover"] = dict(min_gap_mm=round(float(min(gaps)), 2), share_in_contact=round(float(np.mean(np.array(gaps) < 0.6)), 2),
                                   note="negative = the part would sit into the cover; contact is the flat at Z 57")
    others = {k: v for k, v in meshes.items() if k in ("module", "bin")}
    out["parts_to_body"] = {k: round(min_dist(v, body), 1) for k, v in others.items()}
    out["legs"] = {}
    for pose, q in POSES.items():
        L = go2_legs(q)
        out["legs"][pose] = {k: round(min_dist(L, v), 1) for k, v in meshes.items() if k in ("adapter", "module", "bin")}
    # how far a thigh can swing up/back before it touches a part (recovery moves), calf folded
    lim = None
    for th in np.arange(1.6, 4.6, 0.1):
        L = go2_legs({"thigh": float(th), "calf": -2.72})
        if min(min_dist(L, meshes[k], spacing=2.0) for k in ("adapter", "module", "bin")) < 1.0: lim = round(float(th), 1); break
    out["thigh_contact_rad"] = lim
    if arm_mesh is not None:
        out["folded_arm"] = {k: round(min_dist(arm_mesh, v), 1) for k, v in meshes.items() if k in ("bin", "adapter")}
        out["folded_arm"]["module_top_to_arm_bottom_mm"] = round(float(arm_mesh.bounds[0][2] - C.Z_BASE), 1)
        hat = _hat_mesh(); out["hat"] = dict(to_folded_arm=round(min_dist(hat, arm_mesh), 1), to_bin=round(min_dist(hat, meshes["bin"]), 1))
    # battery (left side) and its port, and the pack's way out (+Y)
    bat = g.mesh(include=[g.BATTERY]); port = g.mesh(include=[g.BATTERY_PORT])
    out["battery"] = {k: round(min_dist(v, bat), 1) for k, v in meshes.items() if k in ("adapter", "module", "bin")}
    out["battery_port"] = {k: round(min_dist(v, port), 1) for k, v in meshes.items() if k in ("adapter", "module", "bin")}
    b0, b1 = bat.bounds; path = trimesh.creation.box(bounds=[[b0[0], b0[1], b0[2]], [b1[0], b1[1] + 150, b1[2]]])
    out["battery_way_out_clear"] = all(min_dist(v, path) > 1.0 for k, v in meshes.items() if k in ("adapter", "module", "bin"))
    out["views"] = views(body, meshes, arm_mesh)
    return out


def _is_leg(i):
    b = i["bbox"]
    if abs(b[0][0]) > 1e4: return True
    X0, X1 = C_STEP(b[1][1]), C_STEP(b[0][1])
    return abs(b[0][0]) > 99 or abs(b[1][0]) > 99 or b[1][2] < -100 or X0 < -145 or (X1 > 135 and b[1][2] < 30 and abs(b[0][0]) > 60)


def C_STEP(y):
    import go2_ref as g
    return g.BODY_CENTRE_STEP_Y - y


def _hat_mesh():
    import trimesh
    h = C.HAT; c = trimesh.creation.cylinder(radius=h["brim_d"] / 2, height=h["height"], sections=48)
    c.apply_translation([h["centre"][0], h["centre"][1], C.Z_BASE + h["flange_h"] + h["height"] / 2 + 1.0]); return c


def views(body, meshes, arm_mesh):
    """share of the head camera's view (92 x 52 deg, forward) and of the lidar's upper hemisphere whose first hit is a
    mount part (or the folded arm)"""
    import trimesh
    import go2_ref as g
    cam = g.mesh(include=[g.HEAD_CAMERA]).bounds.mean(0) + [6, 0, 0]
    lid = g.mesh(include=list(g.LIDAR)).bounds.mean(0)
    scene = {"robot": body, **{k: v for k, v in meshes.items() if k in ("adapter", "module", "bin")}}
    if arm_mesh is not None: scene["arm"] = arm_mesh
    scene["hat"] = _hat_mesh()
    names = list(scene); allm = trimesh.util.concatenate([scene[n] for n in names])
    owner = np.concatenate([np.full(len(scene[n].faces), i) for i, n in enumerate(names)])
    def shares(origin, dirs):
        loc, ir, it = allm.ray.intersects_location(np.tile(origin, (len(dirs), 1)), dirs, multiple_hits=False)
        first = {}
        for r, t, p in zip(ir, it, loc):
            dd = np.linalg.norm(p - origin)
            if r not in first or dd < first[r][0]: first[r] = (dd, owner[t])
        res = {}
        for n_i, n in enumerate(names):
            if n == "robot": continue
            res[n] = round(100.0 * sum(1 for v in first.values() if v[1] == n_i) / len(dirs), 2)
        return res
    az = np.radians(np.linspace(-46, 46, 47)); el = np.radians(np.linspace(-26, 26, 27))
    dirs = np.array([[math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)] for a in az for e in el])
    az2 = np.radians(np.arange(-180, 180, 3)); el2 = np.radians(np.arange(1, 90, 3))
    dirs2 = np.array([[math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)] for a in az2 for e in el2])
    return {"head_camera_pct_blocked": shares(cam, dirs), "lidar_upper_hemisphere_pct_blocked": shares(lid, dirs2),
            "camera_mm": [round(float(v)) for v in cam], "lidar_mm": [round(float(v)) for v in lid]}
