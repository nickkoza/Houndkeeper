#!/usr/bin/env python3
"""Export assembled, single-file STL references in millimetres for CAD (e.g. Fusion: Insert > Insert Mesh, units mm).
Sources: models/go2 (MuJoCo Menagerie unitree_go2) and models/maker_arm (MakerMods maker-arm-sdk URDF).
Frames: X forward, Y left, Z up. Visual meshes only. Poses are taken with MuJoCo forward kinematics."""
import sys, argparse
from pathlib import Path
import numpy as np, mujoco, trimesh
R = Path(__file__).resolve().parents[1]; OUT = R / "models/export"; OUT.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(R / "sim/arm")); sys.path.insert(0, str(R / "src"))
from houndkeeper import params as P
ap = argparse.ArgumentParser(); ap.add_argument("--mount-x", type=float, default=None, help="metres forward of body centre; default params/robot.yaml")
MOUNT_X = ap.parse_args().mount_x
MOUNT_X = P.get("maker_arm.mount_x_m") if MOUNT_X is None else MOUNT_X
def gather(m, d, pred, origin=np.zeros(3), Rw=np.eye(3)):
    parts = []
    for i in range(m.ngeom):
        if m.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH or not pred(i): continue
        mid = m.geom_dataid[i]; va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]; fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
        V = m.mesh_vert[va:va + vn] @ d.geom_xmat[i].reshape(3, 3).T + d.geom_xpos[i]; V = (V - origin) @ Rw
        parts.append(trimesh.Trimesh(V * 1000.0, m.mesh_face[fa:fa + fn], process=False))
    return trimesh.util.concatenate(parts)
def save(mesh, name, note):
    mesh.export(OUT / name); ext = mesh.bounds[1] - mesh.bounds[0]
    print(f"{name:34s} {ext[0]:6.0f} x {ext[1]:5.0f} x {ext[2]:5.0f} mm  {note}"); return mesh
# --- Go2 (standing, home keyframe) ---
go2 = mujoco.MjModel.from_xml_path(str(R / "models/go2/go2.xml")); gd = mujoco.MjData(go2)
mujoco.mj_resetDataKeyframe(go2, gd, 0) if go2.nkey else None; mujoco.mj_forward(go2, gd)
vis = lambda m: (lambda i: m.geom_contype[i] == 0 and m.geom_conaffinity[i] == 0)
base = go2.body("base").id; bo = gd.xpos[base].copy(); bR = gd.xmat[base].reshape(3, 3).copy()
g_all = save(gather(go2, gd, vis(go2)), "go2_standing_mm.stl", "whole robot, standing; origin on the floor under the body centre")
trunk = gather(go2, gd, lambda i: vis(go2)(i) and go2.geom_bodyid[i] == base, bo, bR)
save(trunk, "go2_body_mm.stl", "body shell only; origin = body centre (base frame)")
top = trunk.bounds[1][2]; print(f"   Go2 body top surface: {top:.1f} mm above body centre; body {trunk.bounds[0][0]:.0f}..{trunk.bounds[1][0]:.0f} mm along X")
# --- Maker Arm (factory fold = URDF zero), base on the XY plane ---
arm = mujoco.MjModel.from_xml_path(str(R / "sim/arm/maker_arm.xml")); ad = mujoco.MjData(arm); mujoco.mj_forward(arm, ad)
Rup = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])            # URDF +Y is up -> +Z up
a_all = gather(arm, ad, lambda i: True, np.zeros(3), Rup.T); zmin = a_all.bounds[0][2]
a_all.apply_translation([0, 0, -zmin]); save(a_all, "maker_arm_folded_mm.stl", "factory fold (stow pose); base underside on Z=0, base axis at X=Y=0")
b = arm.body("base_link").id; a_base = gather(arm, ad, lambda i: arm.geom_bodyid[i] == b, np.zeros(3), Rup.T); a_base.apply_translation([0, 0, -zmin])
save(a_base, "maker_arm_base_mm.stl", "base only (for the bolt pattern); same frame as the folded arm")
# --- Reference: arm on the Go2 at the chosen mount position (turned 180 deg) ---
MX = MOUNT_X * 1000; ref = a_all.copy(); ref.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [0, 0, 1]))
fp = a_base.bounds; hx, hy = (fp[1][0] - fp[0][0]) / 2, (fp[1][1] - fp[0][1]) / 2               # arm base footprint
TV = trunk.vertices; under = TV[(np.abs(TV[:, 0] - MX) < hx) & (np.abs(TV[:, 1]) < hy)]
seat = float(under[:, 2].max()) if len(under) else top                                             # body surface under the arm base
print(f"   body surface under the arm base at X = {MX:.0f} mm: {seat:.1f} mm above body centre (highest point of the body: {top:.1f} mm)")
ref.apply_translation([MX + bo[0] * 1000, bo[1] * 1000, bo[2] * 1000 + seat])
save(trimesh.util.concatenate([g_all, ref]), "go2_with_arm_reference_mm.stl", f"Go2 standing + folded arm on the body top, {MX:.0f} mm forward of centre (no plate)")
