#!/usr/bin/env python3
"""A twin against the procedural house it was built from ([JLx], [POG]): the scorer, the only part that reads the sim's
truth (house_twin.py never does, [UdA]). Everything is laid in the sim's frame by the night's corrected path against its
true path (floor_fidelity.frame_from_path) and judged where the robot saw, within REACH of its seen floor:
  - walls and furniture: the twin's boxes' footprint against the house plan's (floor_fidelity.truth_structures), the
    things the plan doesn't hold left out on both sides (the chairs, the pet's bowls, the floor lamp: they move or aren't
    in the plan). IoU over the judged region; the faces' IoU (structure within FACE of the seen floor: what the lidar
    could see of it, as walls' far sides never are); the distance from each true face (structure touching the seen floor)
    to the twin's nearest structure, and from each of the twin's faces to the plan's (cm, median and p90);
  - the floor: the texture as written, scored as a floor picture is: floor_truth2.py's patch registration against the
    true texture (mm off it, and after one rigid offset), and the floor map error ([nv5], floor_fidelity.error_of: the
    share of open floor the detector's own test calls different from the true floor), beside the same for night 1's
    picture the twin was built from ("no worse than the floor picture's own error").
Usage: twin_score.py <twin dir> --seed 70 --run sim/house/runs/nights_s70 [--picture n1.npz ...] [--corrections c.npz]
       [--json out.json]   (dimos venv)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))
import argparse, json, math, tempfile
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage as ndi

import floor_agreement as fa
import floor_fidelity as ff
import floor_truth2 as ft2

HX, HY, RES = 8.4, 4.9, 0.01
REACH, FACE, LAMP_R = 0.5, 0.10, 0.3          # m: judged within REACH of the seen floor; faces within FACE of it; the lamp


def footprint(xml, shape):
    """the twin's boxes, as MuJoCo parses its scene, on the pictures' 1 cm grid (row 0 at +y), in the twin's frame"""
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(xml)); A = np.zeros(shape, bool)
    for g in range(m.ngeom):
        if not m.geom(g).name.startswith("map_box"): continue
        (x, y, _), (sx, sy, _) = m.geom_pos[g], m.geom_size[g]
        r0, r1 = int(round((HY - y - sy) / RES)), int(round((HY - y + sy) / RES))
        c0, c1 = int(round((x - sx + HX) / RES)), int(round((x + sx + HX) / RES)); A[max(r0, 0):r1, max(c0, 0):c1] = True
    return A


def not_in_plan(run, seed, shape):
    """the things the plan doesn't hold, on both nights: chairs (+10 cm), the pet's bowls (r 20 cm), the floor lamp"""
    import make_house as mh
    T = [json.loads((Path(run) / f"nights_s{seed}_n{k}.truth.json").read_text()) for k in (1, 2)]
    m = np.zeros(shape, bool)
    for t in T:
        for c in t.get("movables") or []: m |= fa.footprint(shape, fa.box(c["x"], c["y"], c["yaw"], c["w"] + 0.2, c["d"] + 0.2))
        for f in t.get("fixtures") or []: m |= fa.footprint(shape, fa.box(f["x"], f["y"], 0.0, 0.4, 0.4))
    mh.use_layout(seed); m |= fa.footprint(shape, fa.box(mh.LAMP[0], mh.LAMP[1], 0.0, 2 * LAMP_R, 2 * LAMP_R))
    return m


def geometry(xml, seen, frame, seed, run):
    """{iou, face_iou, true_face_to_twin_cm (median, p90), twin_face_to_true_cm, the shares of each within 5 cm, judged_m2}"""
    A = ff.to_true_frame(footprint(xml, seen.shape).astype(np.float32), *frame, order=0) > 0.5
    S = ff.to_true_frame(seen.astype(np.float32), *frame, order=0) > 0.5
    T = ff.truth_structures(seed, seen.shape); X = not_in_plan(run, seed, seen.shape)
    dS = ndi.distance_transform_edt(~S) * RES
    J = (dS <= REACH) & ~X; F = (dS <= FACE) & ~X
    iou = lambda a, b, r: float((a & b & r).sum() / max(((a | b) & r).sum(), 1))
    touch = ndi.binary_dilation(S, iterations=2)                        # structure touching the seen floor: a face
    dA, dT = ndi.distance_transform_edt(~A) * RES * 100, ndi.distance_transform_edt(~T) * RES * 100
    eT, eA = T & touch & ~X, A & touch & ~X
    q = lambda v: [round(float(np.median(v)), 1), round(float(np.percentile(v, 90)), 1)] if len(v) else [None, None]
    return {"iou": round(iou(A, T, J), 3), "face_iou": round(iou(A, T, F), 3), "true_face_to_twin_cm": q(dA[eT]),
            "twin_face_to_true_cm": q(dT[eA]), "true_faces_within_5cm": round(float((dA[eT] <= 5).mean()), 3) if eT.any() else None,
            "twin_faces_within_5cm": round(float((dT[eA] <= 5).mean()), 3) if eA.any() else None, "judged_m2": round(float(J.sum() * RES * RES), 1),
            "true_structure_m2": round(float((T & J).sum() * RES * RES), 2), "twin_structure_m2": round(float((A & J).sum() * RES * RES), 2)}


def as_picture(rgb, seen, path):
    np.savez(path, rgb=rgb, dist=np.where(seen, 1.0, np.nan).astype(np.float16)); return path


def floor(rgb, seen, frame, seed, run, td, corrections):
    """a floor (rgb, seen; the house's frame, 1 cm) against night 1's true floor: floor_truth2's patches laid on the true
    texture (laid in the sim's frame first, as floor_fidelity scores: patches, mm off it median and p90, and after one
    rigid offset) and the floor map error ([nv5]) over the open floor"""
    t_rgb = np.clip(ff.to_true_frame(rgb.astype(np.float32), *frame) + 0.5, 0, 255).astype(np.uint8)
    t_seen = ff.to_true_frame(seen.astype(np.float32), *frame, order=0) > 0.5
    n, m, p90, rm, rp90 = ft2.score(str(as_picture(t_rgb, t_seen, Path(td) / "true_frame.npz")), seed, 1)
    run = Path(run); pic = as_picture(rgb, seen, Path(td) / "house_frame.npz")
    e, m2 = ff.error_of(str(pic), seed, 1, run / f"nights_s{seed}_n1.traj.csv", corrections, run)
    r = lambda v: None if v is None or not np.isfinite(v) else round(float(v), 1)
    return {"patches": n, "off_mm": [r(m), r(p90)], "rigid_off_mm": [r(rm), r(rp90)], "map_error_pct": round(100 * e, 3), "judged_m2": round(m2, 1)}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("twin"); ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--run", required=True); ap.add_argument("--picture", nargs="*", default=[], help="night 1's pictures to score beside it")
    ap.add_argument("--corrections", default=None, help="night 1's corrections the twin was built with (default the run's; a replayed run's own)")
    ap.add_argument("--json", default=None); a = ap.parse_args()
    tw = Path(a.twin); man = json.loads((tw / "twin.json").read_text()); xml = tw / f"{man['name']}.xml"
    z = np.load(tw / "twin.npz"); seen = z["seen"]
    tex = np.asarray(Image.open(tw / f"{man['name']}_floor.png").convert("RGB"))
    if tex.shape[:2] != seen.shape: tex = np.asarray(Image.fromarray(tex).resize(seen.shape[::-1], Image.BOX))
    run = Path(a.run); corr = Path(a.corrections or run / "shape" / "corrections_1.npz")
    frame = ff.frame_from_path(corr, run / f"nights_s{a.seed}_n1.traj.csv")
    out = {"seed": a.seed, "twin": str(tw), "geometry": geometry(xml, seen, frame, a.seed, run)}
    with tempfile.TemporaryDirectory() as td:
        out["floor_twin"] = floor(tex, seen, frame, a.seed, run, td, corr)
        for p in a.picture:
            z_ = np.load(p); out.setdefault("floor_pictures", {})[str(p)] = floor(z_["rgb"], np.isfinite(z_["dist"].astype(np.float32)), frame, a.seed, run, td, corr)
    print(json.dumps(out))
    if a.json: Path(a.json).write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
