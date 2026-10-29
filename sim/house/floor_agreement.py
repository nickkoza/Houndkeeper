#!/usr/bin/env python3
"""Do two floor pictures of the same unchanged floor agree ([FHA])? Two empty nights' pictures compared by the detector's
own test (detect_changes2.feats, tolerant_diff at the confirmer's tc and slack), over the floor the confirmer judges
(seen in both, away from mapped structures): the share of it that reads as changed. Each cell of that share is a place
a toy has to beat the map's own noise. Split by where it lies: the rug, near a chair that moved (its shadow moves too),
and the open floor. A run with no sim truth (a real night, [V35]) is not split: the rug and the chairs are only known from
the truth, so the whole judged floor is reported as open floor.
Usage: floor_agreement.py <run dir> [--a mosaic_n1.npz] [--b mosaic_n2.npz] [--seed 70] [--json out.json]   (dimos venv python)"""
import argparse, json, math
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
import confirm_stream as cs
from detect_changes2 import feats, tolerant_diff


def footprint(shape, corners):
    """cells inside a polygon given in world x, y (m)"""
    from matplotlib.path import Path as Poly
    rc = np.array([((cs.HY - y) / cs.RES, (x + cs.HX) / cs.RES) for x, y in corners])
    r0, c0 = np.floor(rc.min(0)).astype(int); r1, c1 = np.ceil(rc.max(0)).astype(int) + 1
    r0, c0 = max(r0, 0), max(c0, 0); r1, c1 = min(r1, shape[0]), min(c1, shape[1]); m = np.zeros(shape, bool)
    if r1 <= r0 or c1 <= c0: return m
    R, C = np.mgrid[r0:r1, c0:c1]; m[r0:r1, c0:c1] = Poly(rc).contains_points(np.c_[R.ravel(), C.ravel()]).reshape(R.shape)
    return m


def box(x, y, yaw, w, d):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * u - s * v, y + s * u + c * v) for u, v in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2), (-w / 2, d / 2))]


def regions(run, seed, nights, shape):
    """the rug (wherever it lay on either night, +5 cm) and the chairs that moved (both places, +30 cm for their shadows)"""
    import make_nights as mn
    T = [json.load(open(run / f"nights_s{seed}_n{k}.truth.json")) for k in nights]
    rug = np.zeros(shape, bool)
    for t in T:
        if t.get("rug"): rug |= footprint(shape, mn.rug_corners(t["rug"]))
    chairs = np.zeros(shape, bool); A, B = ({m["id"]: m for m in t.get("movables") or []} for t in T)
    for k in set(A) | set(B):
        a, b = A.get(k), B.get(k)
        if a and b and math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < 0.01 and abs(a["yaw"] - b["yaw"]) < 0.01: continue
        for m in (a, b):
            if m: chairs |= footprint(shape, box(m["x"], m["y"], m["yaw"], m["w"], m["d"]))
    grow = lambda m, cm: ndi.binary_dilation(m, iterations=cm) if m.any() else m
    return grow(rug, 5), grow(chairs, 30)


def has_truth(run, seed, nights=(1, 2)):
    """the sim's truth for these nights is in the run dir (a real night has none, and no seed)"""
    return seed is not None and all((Path(run) / f"nights_s{seed}_n{k}.truth.json").exists() for k in nights)


def seed_of(run):
    """the house seed a sim run dir is named for (runs/nights_s<seed>), or None"""
    tail = Path(run).name.split("_s")[-1] if "_s" in Path(run).name else ""
    return int(tail) if tail.isdigit() else None


def agreement(run, seed, a, b, nights=(1, 2)):
    """share of the judged floor reading as changed between pictures a and b: overall, open floor, and with the sim's
    truth also on the rug and near moved chairs (without it, seed None or no truth files, those two are left out)"""
    A, B = np.load(a), np.load(b)
    Va, Vb = (np.isfinite(z["dist"].astype(np.float32)) for z in (A, B))
    static = A["occ"] & B["occ"]                         # structures mapped in both pictures, as the confirmer's baseline
    band = ndi.distance_transform_edt(~static) * cs.RES < cs.CONFIRM["band_m"]   # (from the pictures themselves: pictures
    J = Va & Vb & ~band                                  # placed in another frame than the bench's are judged in their own)
    ca, ra = feats(A["rgb"], Va); cb, rb = feats(B["rgb"], Vb)
    dC, _br = tolerant_diff(ca, ra, cb, rb, slack=cs.CONFIRM["slack"])
    ch = dC > cs.CONFIRM["tc"]
    share = lambda m: (float((ch & m).sum() / m.sum()) if m.sum() else float("nan"), int(m.sum()))
    if not has_truth(run, seed, nights): return {"all": share(J), "open": share(J)}, ch & J
    rug, chairs = regions(run, seed, nights, J.shape); open_ = J & ~rug & ~chairs
    return {"all": share(J), "open": share(open_), "rug": share(J & rug), "chairs": share(J & chairs & ~rug)}, ch & J


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--a"); ap.add_argument("--b")
    ap.add_argument("--seed", type=int); ap.add_argument("--png", default=None, help="draw where it reads as changed")
    ap.add_argument("--json", default=None, help="write the shares here too")
    a = ap.parse_args(); run = Path(a.run); seed = a.seed if a.seed is not None else seed_of(run)
    res, ch = agreement(run, seed, a.a or run / "mosaic_n1.npz", a.b or run / "mosaic_n2.npz")
    who = f"s{seed}" if has_truth(run, seed) else f"{run.name} (no sim truth: rug and chairs not split out)"
    print(f"{who}: reads as changed: " + "; ".join(f"{k} {v[0]:.2%} of {v[1] / 1e4:.1f} m2" for k, v in res.items()))
    if a.json:
        Path(a.json).write_text(json.dumps({k: {"share": v[0], "m2": v[1] / 1e4} for k, v in res.items()}, indent=1))
    if a.png:
        from PIL import Image
        Image.fromarray((ch * 255).astype(np.uint8)).save(a.png)


if __name__ == "__main__":
    main()
