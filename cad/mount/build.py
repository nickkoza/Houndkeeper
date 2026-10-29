#!/usr/bin/env python3
"""Build the printed mount ([3JA]): every part as STEP (robot frame, for Fusion and the FEA) and 3MF (on the bed, in its
print orientation), the load cases for the FEA ([tY3]), the estimates report ([mgb] [lUO] [Uaq] [Frm] [4JP]), the
reference assembly ([I9y]) and the review renders.

    python cad/mount/build.py            (DimOS venv with the `cad` extra; the Go2 cache from tools/go2_step_cache.py)
    python cad/mount/build.py --no-renders --no-clearances     (parts, load cases and estimates only)

Committed: cad/mount/print/ (our parts only), load_cases.json, report.json. Not committed (Unitree's geometry, licence
unstated): models/go2/_src/go2_with_mount_reference_mm.stl and cad/mount/renders/."""
import argparse, json, math, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import config as C            # noqa: E402
import parts                  # noqa: E402
import analysis as A          # noqa: E402

PRINT = HERE / "print"; RENDERS = HERE / "renders"
ORIENT = {"adapter": "flip", "module": "flip", "bench_plate": "flip", "bin": "up", "hat_seat": "up", "coupon": "up"}   # [ngX]


def on_bed(solid, how):
    """the part placed for printing: flipped parts turn about X, then everything sits on Z = 0 at the bed's middle"""
    from build123d import Pos, Rot
    s = Rot(180, 0, 0) * solid if how == "flip" else solid
    bb = s.bounding_box()
    return Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z) * s


def export(P):
    from build123d import export_step, Mesher
    PRINT.mkdir(exist_ok=True); out = {}
    for k, s in P.items():
        export_step(s, str(PRINT / f"{k}.step"))
        bed = on_bed(s, ORIENT[k]); m = Mesher(); m.add_shape(bed, part_number=k); m.write(str(PRINT / f"{k}.3mf"))
        bb = bed.bounding_box(); out[k] = [round(bb.size.X, 1), round(bb.size.Y, 1), round(bb.size.Z, 1)]
    return out


def reference(P, meshes):
    """[I9y] the Go2 (Unitree's CAD, dock removed), the folded arm at [Pvh] and the parts, in one mesh, robot frame, mm"""
    import trimesh
    import go2_ref as g
    if not g.available(): return None
    arm = A.arm_stow_mesh()
    go2 = g.mesh(exclude=g.DOCK)
    ref = trimesh.util.concatenate([go2] + ([arm] if arm is not None else []) + [meshes[k] for k in parts.ON_ROBOT])
    f = g.CACHE.parent / "go2_with_mount_reference_mm.stl"; ref.export(f)
    return str(f.relative_to(C.R))


# ---------------------------------------------------------------- renders (painter's algorithm, orthographic)
COL = {"go2": (0.80, 0.80, 0.78), "arm": (0.30, 0.31, 0.34), "adapter": (0.93, 0.55, 0.20), "module": (0.22, 0.50, 0.85),
       "bin": (0.30, 0.68, 0.40), "bench_plate": (0.78, 0.66, 0.48), "hat": (0.10, 0.10, 0.10), "hat_seat": (0.93, 0.80, 0.25),
       "bench": (0.55, 0.42, 0.30)}


def render(items, view, path, title, size=(10.8, 10.8), pad=30):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    az, el = map(math.radians, view)
    d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])     # towards the viewer
    right = np.cross([0, 0, 1.0], d); right = right / np.linalg.norm(right) if np.linalg.norm(right) > 1e-6 else np.array([0, -1.0, 0])
    up = np.cross(d, right)
    light = d * 0.6 + up * 0.5 + right * 0.2; light /= np.linalg.norm(light)
    import trimesh
    polys, cols, depth = [], [], []
    for name, m in items:
        if name != "go2":                                                  # long thin triangles break a depth sort
            v, f = trimesh.remesh.subdivide_to_size(m.vertices, m.faces, max_edge=12.0); m = trimesh.Trimesh(v, f, process=False)
        T = m.vertices[m.faces]; n = m.face_normals; vis = n @ d > 0
        T, n = T[vis], n[vis]
        P2 = np.stack([T @ right, T @ up], axis=-1); dep = (T.mean(1) @ d)
        shade = 0.45 + 0.55 * np.clip(n @ light, 0, 1)
        c = np.clip(np.outer(shade, COL[name]), 0, 1)
        polys.append(P2); cols.append(c); depth.append(dep)
    P2 = np.concatenate(polys); c = np.concatenate(cols); dep = np.concatenate(depth); o = np.argsort(dep)
    fig = plt.figure(figsize=size, dpi=100); ax = fig.add_axes([0, 0, 1, 0.90]); ax.set_axis_off()
    ax.add_collection(PolyCollection(P2[o], facecolors=c[o], edgecolors=c[o], linewidths=0.1))
    lo, hi = P2.reshape(-1, 2).min(0) - pad, P2.reshape(-1, 2).max(0) + pad
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect("equal")
    fig.text(0.03, 0.965, title, fontsize=20, va="center")
    keys = [k for k in COL if any(k == n for n, _ in items)]
    for i, k in enumerate(keys):
        fig.text(0.03 + 0.24 * (i % 4), 0.93 - 0.03 * (i // 4) * (10.8 / size[1]), "■ " + k.replace("_", " "), color=COL[k], fontsize=14, va="center")
    fig.savefig(path, facecolor="white"); plt.close(fig)


def renders(meshes):
    import trimesh
    import go2_ref as g
    RENDERS.mkdir(exist_ok=True)
    arm = A.arm_stow_mesh()
    h = C.HAT; hx, hy = h["centre"]; z = C.Z_BASE + h["flange_h"] + h["boss_h"] - 2.0
    brim = trimesh.creation.cylinder(radius=h["brim_d"] / 2, height=2.5, sections=48); brim.apply_translation([hx, hy, z + 1.25])
    crown = trimesh.creation.capsule(height=h["height"] - h["crown_open_d"] / 2 - 2, radius=h["crown_open_d"] / 2, count=[24, 24])
    crown.apply_translation([hx, hy, z + 2 + (h["height"] - h["crown_open_d"] / 2 - 2) / 2])
    hat = trimesh.util.concatenate([brim, crown])
    go2 = g.mesh(exclude=g.DOCK) if g.available() else None
    robot = [("go2", go2)] if go2 is not None else []
    on = [(k, meshes[k]) for k in ("adapter", "module", "bin", "hat_seat")] + ([("arm", arm)] if arm is not None else []) + [("hat", hat)]
    out = []
    f = RENDERS / "1_side.png"; render(robot + on, (-90, 0), f, "Barrymore with the mount: side", size=(10.8, 7.2)); out.append(f)
    f = RENDERS / "2_top.png"
    render(robot + on, (0, 90), f, "From above (head at the top)", size=(10.8, 13.5)); out.append(f)
    lift = np.eye(4); lift[2, 3] = 200.0
    mod = [("module", meshes["module"].copy().apply_transform(lift))] + ([("arm", arm.copy().apply_transform(lift))] if arm is not None else [])
    f = RENDERS / "3_module_off.png"
    render(robot + [(k, meshes[k]) for k in ("adapter", "bin")] + mod, (-135, 28), f, "The module lifted off: adapter and bin stay", size=(10.8, 10.8))
    out.append(f)
    bench = trimesh.creation.box(bounds=[[-150, -260, C.Z_IF - C.BENCH_T - 30], [300, 260, C.Z_IF - C.BENCH_T]])
    f = RENDERS / "4_module_on_bench.png"
    render([("bench", bench), ("bench_plate", meshes["bench_plate"]), ("module", meshes["module"])] + ([("arm", arm)] if arm is not None else []),
           (-50, 25), f, "The module on the bench plate", size=(10.8, 10.8)); out.append(f)
    return [str(p.relative_to(C.R)) for p in out]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--no-renders", action="store_true"); ap.add_argument("--no-clearances", action="store_true")
    a = ap.parse_args(); t0 = time.time()
    P = parts.build(); print(f"parts built in {time.time() - t0:.0f} s", flush=True)
    bed = export(P); print("exported", bed, flush=True)
    meshes = {k: A.to_mesh(v, 0.3) for k, v in P.items()}
    mats = C.materials(); arm = A.arm_numbers()
    cases = A.load_cases(arm); (HERE / "load_cases.json").write_text(json.dumps(cases, indent=1, ensure_ascii=False), encoding="utf-8")
    mass = {k: {m: round(A.mass_g(P[k], d["density_g_cm3"])[0]) for m, d in mats.items()} for k in P}
    hw = sum(C.HARDWARE_G.values())
    totals = {m: dict(printed_g=sum(mass[k][m] for k in parts.ON_ROBOT), hardware_g=hw,
                      total_kg=round((sum(mass[k][m] for k in parts.ON_ROBOT) + hw) / 1000, 2), budget_kg=C.KIT_BUDGET) for m in mats}
    st = A.stiffness(meshes["adapter"], mats, meshes["module"])
    rep = dict(date="2026-10-04", frame=cases["frame"], bed_mm=C.BED, on_bed_size_mm=bed, mass_g=mass, mass_on_robot=totals,
               print=dict(walls=C.WALLS, line_width=C.LINE_W, top_bottom_layers=C.TOPBOT_LAYERS, layer=C.LAYER, infill=C.INFILL),
               z_base_mm=C.Z_BASE, z_interface_mm=C.Z_IF, arm=arm,
               stiffness={k: v for k, v in st.items() if k != "sections"}, sim_flex_Nm_per_rad=C.SIM_FLEX,
               jaw_error=A.jaw_error(st, arm), stress=A.stresses(st, cases["cases"], arm, mats), magnets=A.magnets(arm),
               bin=dict(inner_mm=[round(C.BIN_X[1] - C.BIN_X[0] - 2 * C.BIN_WALL), round(2 * (C.BIN_HALF_W - C.BIN_WALL)), round(C.BIN_TOP_Z - C.Z_BIN - C.BIN_FLOOR)],
                        litres=round((C.BIN_X[1] - C.BIN_X[0] - 2 * C.BIN_WALL) * 2 * (C.BIN_HALF_W - C.BIN_WALL) * (C.BIN_TOP_Z - C.Z_BIN - C.BIN_FLOOR) / 1e6, 2)))
    if not a.no_clearances:
        import go2_ref as g
        if g.available():
            rep["clearances"] = A.clearances(meshes, A.arm_stow_mesh()); print("clearances done", flush=True)
            rep["reference_assembly"] = reference(P, meshes)
    if not a.no_renders:
        rep["renders"] = renders(meshes)
    (HERE / "report.json").write_text(json.dumps(rep, indent=1, ensure_ascii=False, default=float), encoding="utf-8")
    print(f"done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
