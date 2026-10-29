#!/usr/bin/env python3
"""Cache Unitree's Go2 STEP (simplified, with the expansion dock) as light per-solid meshes and a hole index ([3JA] [I9y]).

The STEP is 204 MB and takes minutes and several GB to read; everything the mount design needs from it is cached once:
  models/go2/_src/go2_cache/solids.npz  - every solid tessellated (mm, the STEP's own frame), triangle -> solid id
  models/go2/_src/go2_cache/index.json  - per solid: name, colour, bounding box, volume, its cylindrical faces (holes/bosses)
Unitree's licence is not stated, so the cache stays in models/go2/_src/ (gitignored), like the STEP itself.

Run (DimOS venv, `cad` extra): python tools/go2_step_cache.py   [--tol 0.4]
"""
import argparse, json, sys, time
from pathlib import Path
import numpy as np

R = Path(__file__).resolve().parents[1]
SRC = R / "models/go2/_src/go2_simplified_with_dock_0911"
OUT = R / "models/go2/_src/go2_cache"


def mesh_faces(shape, tol):
    """Triangulate face by face, skipping any face OCC cannot mesh (some of Unitree's simplified faces are degenerate;
    build123d's tessellate gives up on the whole solid at the first one)."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRep import BRep_Tool
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    BRepMesh_IncrementalMesh(shape, tol, False, 0.5, True)
    V, F, n = [], [], 0
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        f = TopoDS.Face(ex.Current()); ex.Next(); loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is None: continue
        tr = loc.Transformation()
        v = np.array([[p.X(), p.Y(), p.Z()] for p in (tri.Node(i).Transformed(tr) for i in range(1, tri.NbNodes() + 1))], np.float32)
        t = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)], np.int32) - 1
        if f.Orientation() == TopAbs_REVERSED: t = t[:, ::-1]
        V.append(v); F.append(t + n); n += len(v)
    if not V: return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.int32)
    return np.concatenate(V), np.concatenate(F)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--tol", type=float, default=0.4, help="mesh linear deflection, mm")
    a = ap.parse_args()
    from build123d import import_step, Solid, Shell
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    from OCP.TopAbs import TopAbs_REVERSED
    step = next(SRC.glob("*.stp")); t0 = time.time()
    comp = import_step(step); print(f"read {step.name} in {time.time() - t0:.0f} s", flush=True)
    leaves = []
    def walk(s, path):
        kids = list(s.children) if hasattr(s, "children") else []
        if kids:
            for k in kids: walk(k, path + [getattr(s, "label", "")])
        else: leaves.append((s, "/".join(p for p in path + [getattr(s, "label", "")] if p)))
    walk(comp, [])
    solids = []
    for s, name in leaves:
        col = getattr(s, "color", None); col = list(col.to_tuple()) if col is not None else None
        sub = s.solids() or s.shells() or [s]
        for i, so in enumerate(sub): solids.append((so, name if len(sub) == 1 else f"{name}#{i}", col))
    print(f"{len(solids)} solids/shells", flush=True)
    V, F, SID, index = [], [], [], []; nv = 0
    for sid, (so, name, col) in enumerate(solids):
        bb = so.bounding_box()
        try: vol = float(so.volume)
        except Exception: vol = 0.0
        cyl = []
        for f in so.faces():
            ad = BRepAdaptor_Surface(f.wrapped)
            if ad.GetType() != GeomAbs_Cylinder: continue
            c = ad.Cylinder(); ax = c.Axis(); p, d = ax.Location(), ax.Direction()
            fb = f.bounding_box()
            cyl.append({"r": round(c.Radius(), 3), "p": [round(p.X(), 2), round(p.Y(), 2), round(p.Z(), 2)],
                        "d": [round(d.X(), 4), round(d.Y(), 4), round(d.Z(), 4)],
                        "c": [round((fb.min.X + fb.max.X) / 2, 2), round((fb.min.Y + fb.max.Y) / 2, 2), round((fb.min.Z + fb.max.Z) / 2, 2)],
                        "ext": [round(fb.size.X, 2), round(fb.size.Y, 2), round(fb.size.Z, 2)],
                        "inner": bool(f.wrapped.Orientation() == TopAbs_REVERSED)})   # reversed face on a cylinder = a hole
        v, t = mesh_faces(so.wrapped, a.tol)
        if not len(t): print("no triangles", sid, name)
        V.append(v); F.append(t + nv); SID.append(np.full(len(t), sid, np.int32)); nv += len(v)
        index.append({"id": sid, "name": name, "color": col, "volume_mm3": round(vol, 1), "n_tri": int(len(t)),
                      "bbox": [[round(bb.min.X, 2), round(bb.min.Y, 2), round(bb.min.Z, 2)], [round(bb.max.X, 2), round(bb.max.Y, 2), round(bb.max.Z, 2)]],
                      "cylinders": cyl})
        if sid % 20 == 0: print(f"  {sid}/{len(solids)}  {time.time() - t0:.0f} s", flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUT / "solids.npz", V=np.concatenate(V), F=np.concatenate(F), SID=np.concatenate(SID))
    (OUT / "index.json").write_text(json.dumps({"source": step.name, "units": "mm", "tol_mm": a.tol, "solids": index}, ensure_ascii=False, indent=0))
    print(f"wrote {OUT} ({nv} vertices) in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    sys.exit(main())
