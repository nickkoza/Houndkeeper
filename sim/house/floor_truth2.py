#!/usr/bin/env python3
"""A floor picture scored against the floor the sim truly drew, everywhere it saw ([FHA], [WFI]): the true texture
(make_nights' photographed floor, drawn at 3 mm) averaged to the picture's 1 cm grid, and every 60 cm patch of the
picture laid on it by masked correlation of edges within +-8 cm, kept only where the match is clearly best
(floor_register.snap; phase correlation on brightness matched at random: a true-pose picture read 36-92 mm off).
Reports each patch's shift, and the shifts
left once one rigid offset for the whole picture is taken off: what the picture's own warping costs, which is what the
nightly comparison feels. The seams metric (bench_mapping.py) measures two lines in one corner; this measures the map.
Usage: floor_truth2.py <mosaic.npz> [<mosaic.npz> ...] --seed 70 --night 1 (dimos venv)"""
import argparse
import numpy as np
from PIL import Image
from scipy import ndimage as ndi

import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
from houndkeeper import floor_register as fr
import make_nights as mn

HX, HY, RES, P, PAD = 8.4, 4.9, 0.01, 60, 12


def truth(seed, night, shape):
    """the true floor texture on the picture's 1 cm grid"""
    tex = np.asarray(Image.open(mn.TEX_DIR / f"house_floor_s{seed}_n{night}.png").convert("RGB")).astype(np.float32)
    k = tex.shape[1] / (2 * HX / RES)                                    # texture pixels per 1 cm cell (3.33 at 3 mm)
    out = ndi.zoom(tex, (1 / k, 1 / k, 1), order=1)
    return out[:shape[0], :shape[1]]


def score(path, seed, night):
    z = np.load(path); rgb = z["rgb"]; seen = np.isfinite(z["dist"].astype(np.float32)) if "dist" in z else rgb.any(-1)
    tex = truth(seed, night, rgb.shape); hh, ww = min(tex.shape[0], rgb.shape[0]), min(tex.shape[1], rgb.shape[1])
    rgb, seen, tex = rgb[:hh, :ww], seen[:hh, :ww], tex[:hh, :ww]
    g = rgb.astype(np.float32).mean(-1); t = tex.mean(-1); S = []
    for r in range(PAD, hh - P - PAD, P):                               # each patch matched on edges (lighting and exposure
        for c in range(PAD, ww - P - PAD, P):                            # don't matter) within +-8 cm, as the stitch matches
            sl = (slice(r, r + P), slice(c, c + P)); wl = (slice(r - PAD, r + P + PAD), slice(c - PAD, c + P + PAD))
            if seen[sl].mean() < 0.9: continue
            F = np.zeros((P + 2 * PAD, P + 2 * PAD), np.float32); Fm = np.zeros(F.shape, bool)
            F[PAD:-PAD, PAD:-PAD] = g[sl]; Fm[PAD:-PAD, PAD:-PAD] = seen[sl]
            s_ = fr.snap(F, Fm, t[wl], np.ones(F.shape, bool), maxs=8, turns_deg=(0.0,), lead_cells=4, turn_edge_locks=True)
            if s_ is None or s_[3] < 0.3 or s_[4] < 0.05: continue       # no clear match: floor too plain to say
            S.append((s_[1], s_[2]))
    if not S: return 0, np.nan, np.nan, np.nan, np.nan
    S = np.array(S, float) * 10                                         # mm
    raw = np.hypot(*S.T); rel = np.hypot(*(S - np.median(S, 0)).T)
    return len(S), np.median(raw), np.percentile(raw, 90), np.median(rel), np.percentile(rel, 90)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("mosaics", nargs="+"); ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--night", type=int, required=True); a = ap.parse_args()
    for p in a.mosaics:
        n, m, p90, rm, rp90 = score(p, a.seed, a.night)
        print(f"{p[-60:]}: {n} patches; off the true floor {m:.0f} mm median, {p90:.0f} p90; "
              f"after one rigid offset {rm:.0f} mm median, {rp90:.0f} p90")


if __name__ == "__main__":
    main()
