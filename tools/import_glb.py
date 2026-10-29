#!/usr/bin/env python3
"""A downloaded model (.glb) made ready for the sims ([CaM]): one OBJ (Z up, metres, origin at the centre of its
footprint on the floor, its front facing +x) and its PBR textures as MuJoCo layers (colour, normal, ORM: occlusion,
roughness, metallic), resized; and meta.json with where it came from and its licence, for the credits
(in any video that shows it). A model in several parts keeps them (name_p0.obj, ...: each its own material), placed
together; a part without textures keeps its colour (meta.json rgba).
Usage: import_glb.py model.glb --name wing_chair --height 1.10 (or --length 0.20) --front -z --source URL --author NAME --licence CC-BY-4.0
       [--tex 2048] [--out models/furniture]   (dimos venv)"""
import argparse, json
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image

AXES = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("glb"); ap.add_argument("--name", required=True)
    ap.add_argument("--height", type=float, default=None, help="the real thing's height (m): the model is scaled to it")
    ap.add_argument("--length", type=float, default=None, help="or its longest side (m), for things that lie flat (fruit)")
    ap.add_argument("--up", default="+y", choices=AXES, help="the model's up axis (glTF: +y)")
    ap.add_argument("--front", default="+z", choices=AXES, help="the model's front, in its own axes (turned to face +x)")
    ap.add_argument("--source", required=True); ap.add_argument("--author", required=True); ap.add_argument("--licence", required=True)
    ap.add_argument("--tex", type=int, default=2048, help="texture size (px); the full-size originals are kept beside them")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "models" / "furniture"))
    a = ap.parse_args()
    scene = trimesh.load(a.glb)
    if hasattr(scene, "graph"):                                  # every part where the file places it (its node transforms)
        parts = []
        for node in scene.graph.nodes_geometry:
            T, gname = scene.graph[node]; parts.append(scene.geometry[gname].copy().apply_transform(T))
    else: parts = [scene]
    up, fr = np.array(AXES[a.up], float), np.array(AXES[a.front], float)
    R = np.stack([fr, np.cross(up, fr), up])                  # rows: the model's axes that become world x (front), y, z (up)
    Vs = [p.vertices @ R.T for p in parts]; V = np.vstack(Vs)
    if (a.height is None) == (a.length is None): raise SystemExit("give --height or --length")
    k_ = a.height / np.ptp(V[:, 2]) if a.height else a.length / np.ptp(V, 0).max()
    off = np.r_[(V[:, :2].min(0) + V[:, :2].max(0)) / 2, V[:, 2].min()]
    Vs = [(v - off) * k_ for v in Vs]; V = np.vstack(Vs)       # one scale and origin for all the parts
    out = Path(a.out) / a.name; out.mkdir(parents=True, exist_ok=True); size = (a.tex, a.tex); meta_parts = []
    for i, (part, Vp) in enumerate(zip(parts, Vs)):
        stem = a.name if len(parts) == 1 else f"{a.name}_p{i}"; m = part.visual.material if hasattr(part.visual, "material") else None
        uv = getattr(part.visual, "uv", None)
        with open(out / f"{stem}.obj", "w") as f:                # MuJoCo reads v / vt / f; glTF's v runs down, OBJ's up
            f.write(f"# {stem}: from {a.source} by {a.author}, {a.licence} (rescaled, re-oriented)\n")
            f.writelines(f"v {x:.5f} {y:.5f} {z:.5f}\n" for x, y, z in Vp)
            if uv is not None:
                f.writelines(f"vt {u:.5f} {1 - v:.5f}\n" for u, v in uv)
                f.writelines(f"f {i_ + 1}/{i_ + 1} {j + 1}/{j + 1} {k + 1}/{k + 1}\n" for i_, j, k in part.faces)
            else: f.writelines(f"f {i_ + 1} {j + 1} {k + 1}\n" for i_, j, k in part.faces)
        def save(img, tag):
            if img is None or uv is None: return None
            img = img.convert("RGB"); img.save(out / f"{stem}_{tag}_full.png"); img.resize(size, Image.LANCZOS).save(out / f"{stem}_{tag}.png")
            return f"{stem}_{tag}.png"
        rgb = save(getattr(m, "baseColorTexture", None), "rgb"); nrm = save(getattr(m, "normalTexture", None), "n")
        mr, occ = getattr(m, "metallicRoughnessTexture", None), getattr(m, "occlusionTexture", None)
        orm = None
        if mr is not None:                                       # glTF: G roughness, B metallic; occlusion in its own R
            mr = np.asarray(mr.convert("RGB")); o = np.asarray(occ.convert("RGB").resize(mr.shape[1::-1]))[..., 0] if occ is not None else np.full(mr.shape[:2], 255, np.uint8)
            orm = save(Image.fromarray(np.dstack([o, mr[..., 1], mr[..., 2]]).astype(np.uint8)), "orm")
        fac = getattr(m, "baseColorFactor", None)
        rgba = [round(float(c) / (255.0 if np.max(fac) > 1 else 1.0), 3) for c in fac] if fac is not None else [0.8, 0.8, 0.8, 1.0]
        meta_parts.append(dict(obj=f"{stem}.obj", textures=dict(rgb=rgb, normal=nrm, orm=orm), rgba=rgba,
                               roughness=float(getattr(m, "roughnessFactor", None) or 0.6), metallic=float(getattr(m, "metallicFactor", None) or 0.0)))
    meta = dict(name=a.name, source=a.source, author=a.author, licence=a.licence, glb=Path(a.glb).name, height_m=round(float(np.ptp(V[:, 2])), 3),
                footprint_m=[round(float(v), 3) for v in np.ptp(V[:, :2], 0)], faces=int(sum(len(p.faces) for p in parts)), parts=meta_parts)
    (out / "meta.json").write_text(json.dumps(meta, indent=1))
    print(f"{a.name}: {len(parts)} part(s), {meta['faces']} faces, {meta['footprint_m'][0]:.2f} x {meta['footprint_m'][1]:.2f} m footprint, "
          f"{np.ptp(V[:, 2]):.2f} m tall -> {out}")

if __name__ == "__main__":
    main()
