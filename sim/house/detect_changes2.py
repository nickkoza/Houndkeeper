#!/usr/bin/env python3
"""Floor change detection v2 (v1 = detect_changes.py, frozen). Changes, decided before any v2 held-out result:
  1. Baseline = median of ALL previous nights (up to --window), not the last 3: the floor is a steady state,
     and a toy left out for a night or two no longer becomes part of it.
  2. Holes in the compared floor (where a tall object hides the floor) are not treated as its edge, so the
     object sitting in the hole is not thrown away as an edge sliver.
  3. "Stands up" evidence comes from raw lidar heights (tall_nN.npz from tall_layer.py), not the planner's
     costmap, which grades small objects below its lethal cut-off.
  4. Blobs that are both tiny (< --small-px cm2) and faint (peak < --small-peak) are dropped (tuned on nights 2-4).
  5. Candidate fragments within --link-px (2 px) join one blob, so thin strings are not lost as specks (tuned on nights 2-4).
  Structure mask is causal (tonight and earlier nights only); v1 used every night passed in --nights.
v1 docstring follows.

Per night n >= 2: baseline = per-cell median of up to the 3 previous nights' mosaics (so anything
that stays 2+ nights is absorbed: the promotion rule). Candidates = cells whose colour (chromaticity)
shifts or that get brighter relative to their surroundings; darkening (shadows) is ignored. Floor near
structures DimOS maps on every night is skipped. Blobs are split into "something is here" vs
"something left" by comparing to their surroundings. Ground truth is only used for scoring.
Usage: detect_changes.py --seed 7 --nights 4   (dimos venv python)"""
import argparse, json
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
from PIL import Image, ImageDraw

RES, HX, HY = 0.01, 8.4, 4.9
TWO_VIEW = False   # frozen for held-out: maximise catches; the arm's look-closer step handles false alarms
TC, TB, MIN_PX, BIG_PX = 0.06, 0.30, 4, 5000     # chroma shift, brightening, min blob, "floor-sized" blob (0.5 m^2)


def w2rc(x, y):
    return int((HY - y) / RES), int((x + HX) / RES)


def feats(rgb, valid, sigma=25):
    """Colour and brightness relative to the surrounding floor, so the tint and level of the light cancel out. sigma:
    the surround's size in cells (25: 25 cm on the 1 cm grid; a finer grid passes more cells for the same floor)"""
    f = rgb.astype(np.float32); I = f.mean(-1)
    V = valid.astype(np.float32); den = np.maximum(ndi.gaussian_filter(V, sigma), 1e-3)
    L = ndi.gaussian_filter(I * V, sigma) / den
    c = f / (f.sum(-1, keepdims=True) + 1.0)
    cl = np.stack([ndi.gaussian_filter(c[..., k] * V, sigma) / den for k in range(3)], -1)
    return c - cl, I / np.maximum(L, 1.0)


def tolerant_diff(cn, rn, cb, rb, slack=2):
    """Per-pixel change vs the baseline, allowing up to `slack` px (cm) of misalignment between nights."""
    dC = np.full(rn.shape, np.inf, np.float32); br = np.full(rn.shape, np.inf, np.float32)
    for dy in range(-slack, slack + 1):
        for dx in range(-slack, slack + 1):
            cbs = np.roll(cb, (dy, dx), (0, 1)); rbs = np.roll(rb, (dy, dx), (0, 1))
            dC = np.minimum(dC, np.linalg.norm(cn - cbs, axis=-1)); br = np.minimum(br, np.abs(rn - rbs) + 0 * (rn - rbs))
            br = np.where(rn - rbs < br, rn - rbs, br)
    return dC, br


def main(argv=None, scorer=None):
    """the detector over a run's nights: each night's blobs and marks (detect_nN.json, .npz). scorer(run, seed): the
    scorer that matches them with the sim's truth (score_detect2.py); the robot never passes one ([1B2])"""
    ap = argparse.ArgumentParser(); ap.add_argument("--seed", type=int, default=7); ap.add_argument("--nights", type=int, default=4)
    ap.add_argument("--run-dir", default=None, help="read mosaics / write results here (IO only)")
    ap.add_argument("--window", type=int, default=7); ap.add_argument("--small-px", type=int, default=8); ap.add_argument("--link-px", type=int, default=2); ap.add_argument("--small-peak", type=float, default=0.45); ap.add_argument("--hole-m2", type=float, default=0.25)
    ap.add_argument("--model-dir", default=None, help="baseline = composite floor model after the previous night (floor_model_after_nightN.png), with tonight aligned to it")
    a = ap.parse_args(argv)
    run = Path(a.run_dir) if a.run_dir else Path(__file__).resolve().parent / "runs" / f"nights_s{a.seed}"
    M = {n: np.load(run / f"mosaic_n{n}.npz") for n in range(1, a.nights + 1) if (run / f"mosaic_n{n}.npz").exists()}
    occ_count = np.sum([M[n]["occ"] for n in M], axis=0)
    static = occ_count >= 2                                               # mapped as occupied on 2+ nights
    band = ndi.distance_transform_edt(~static) * RES < 0.40
    sc = scorer(run, a.seed) if scorer else None
    for n in sorted(M)[1:]:
        occ_count = np.sum([M[k]["occ"] for k in M if k <= n], axis=0)       # causal: tonight and earlier nights only
        static = occ_count >= 2; band = ndi.distance_transform_edt(~static) * RES < 0.40
        prev = [k for k in range(max(1, n - a.window), n) if k in M]
        rgb_n = M[n]["rgb"]; Vn = np.isfinite(M[n]["dist"].astype(np.float32))
        stack = np.ma.masked_array(np.stack([M[k]["rgb"] for k in prev]).astype(np.float32),
                                   mask=~np.stack([np.isfinite(M[k]["dist"].astype(np.float32)) for k in prev])[..., None].repeat(3, -1))
        base = np.ma.median(stack, axis=0); Vb = ~np.ma.getmaskarray(base)[..., 0]; base = base.filled(0).astype(np.uint8)
        if a.model_dir and (Path(a.model_dir) / f"floor_model_after_night{n-1}.png").exists():
            from PIL import Image as _I; import floor_model as fm
            mdl = np.asarray(_I.open(Path(a.model_dir) / f"floor_model_after_night{n-1}.png").convert("RGB"))[: rgb_n.shape[0], : rgb_n.shape[1]]
            pad = np.zeros_like(rgb_n); pad[: mdl.shape[0], : mdl.shape[1]] = mdl; base = pad; Vb = base.sum(-1) > 0
            al, Va, _ = fm.align(rgb_n.astype(np.float32), Vn, base.astype(np.float32), Vb)   # tonight aligned to the model
            rgb_n = np.clip(al, 0, 255).astype(np.uint8); Vn = Va
        region = Vn & Vb & ~band
        cn, rn = feats(rgb_n, Vn); cb, rb = feats(base, Vb)
        dC, bright = tolerant_diff(cn, rn, cb, rb)
        if TWO_VIEW and "rgb2" in M[n].files:                            # confirm with the second view (off: max-catch)
            V2 = np.isfinite(M[n]["dist2"].astype(np.float32))
            cn2, rn2 = feats(M[n]["rgb2"], V2)
            dC2, bright2 = tolerant_diff(cn2, rn2, cb, rb)
            both = ((dC > TC) | (bright > TB)) & ((dC2 > TC) | (bright2 > TB)) & V2
            dC = np.where(both, dC, 0); bright = np.where(both, bright, 0)
        holes = ndi.binary_fill_holes(region) & ~region; hl, _ = ndi.label(holes)
        hs = ndi.sum(holes, hl, range(1, hl.max() + 1)) * RES * RES
        small = np.isin(hl, np.nonzero(hs <= a.hole_m2)[0] + 1)      # small enclosed holes: something standing on the floor
        core = ndi.binary_erosion(region | small, iterations=6) & region              # comparisons are unreliable at the edge of seen floor
        cand = core & ((dC > TC) | (bright > TB))
        cand = ndi.binary_closing(cand, iterations=2) & core
        rim = (region | small) & ~ndi.binary_erosion(region | small, iterations=6)
        lab, nb = ndi.label(ndi.binary_dilation(cand, iterations=a.link_px) if a.link_px else cand); lab = lab * cand; blobs = []   # fragments within link_px join one blob
        for i, sl in enumerate(ndi.find_objects(lab), 1):
            m = lab[sl] == i; area = int(m.sum())
            if area < MIN_PX: continue
            if (ndi.binary_dilation(m, iterations=2) & rim[sl]).any(): continue      # touches the edge of seen floor
            pad = tuple(slice(max(s.start - 8, 0), s.stop + 8) for s in sl)
            mm = lab[pad] == i; ring = ndi.binary_dilation(mm, iterations=6) & ~mm & region[pad]
            if ring.sum() < 5: continue
            here = np.linalg.norm(cn[pad][mm].mean(0) - cn[pad][ring].mean(0)) + max(0, rn[pad][mm].mean() - rn[pad][ring].mean())
            left = np.linalg.norm(cb[pad][mm].mean(0) - cb[pad][ring].mean(0)) + abs(rb[pad][mm].mean() - rb[pad][ring].mean())
            kind = "floor_change" if area > BIG_PX else ("present" if here >= 0.8 * left else "gone")
            peak = float(max(dC[sl][m].max(), bright[sl][m].max()))
            if area < a.small_px and peak < a.small_peak: continue    # tiny AND faint: texture/registration speckle
            blobs.append({"id": i, "area_cm2": area, "kind": kind, "sl": sl, "here": round(float(here), 4), "peak": round(peak, 4)})
        # Known marks: spots that stand out from the floor around them tonight AND in the baseline (stains).
        def odd(c, r):
            V = region.astype(np.float32)
            cs = np.stack([ndi.gaussian_filter(c[..., k] * V, 25) / np.maximum(ndi.gaussian_filter(V, 25), 1e-3) for k in range(3)], -1)
            return (r < 0.8) | (np.linalg.norm(c - cs, axis=-1) > TC)
        known = region & ~cand & odd(cn, rn) & odd(cb, rb)
        known = ndi.binary_opening(known, iterations=2)            # drops thin grout lines
        klab, _ = ndi.label(known); marks = []
        for i, sl in enumerate(ndi.find_objects(klab), 1):
            area = int((klab[sl] == i).sum())
            if 50 <= area <= BIG_PX: marks.append({"id": i, "area_cm2": area})
            else: klab[klab == i] = 0
        # Lidar side: obstacles DimOS mapped tonight that aren't there on other nights (things that stand up)
        if (run / f"tall_n{n}.npz").exists() and all((run / f"tall_n{k}.npz").exists() for k in prev):
            Tn = np.load(run / f"tall_n{n}.npz")["h"] > 0
            Tb = ndi.binary_dilation(np.sum([np.load(run / f"tall_n{k}.npz")["h"] > 0 for k in prev], axis=0) >= (len(prev) + 1) // 2, iterations=8)   # usually there (8 cm: lidar/pose drift between nights)
            tall = Tn & ~Tb & ~band & ndi.binary_dilation(region, iterations=20)
        else:
            others = np.sum([M[k]["occ"] for k in M if k != n], axis=0)
            tall = M[n]["occ"] & (others == 0) & core
        tall = ndi.binary_opening(tall, iterations=1)
        tlab, _ = ndi.label(tall); off = int(lab.max())
        for i, sl in enumerate(ndi.find_objects(tlab), 1):
            m = tlab[sl] == i
            if m.sum() < 20: continue
            if (lab[sl][m] > 0).any(): continue                   # the camera already flagged it
            if Vb[sl][m].mean() < 0.5: continue                   # earlier nights never saw bare floor here: not evidence of new
            lab[sl][m] = off + i
            blobs.append({"id": off + i, "area_cm2": int(m.sum()), "kind": "present", "sl": sl, "here": None, "peak": None, "source": "lidar"})
        present_ids = {b["id"] for b in blobs if b["kind"] == "present"}
        pres = np.isin(lab, list(present_ids))
        for m_ in marks:                                           # where each known mark lies
            rr, cc = np.nonzero(klab == m_["id"])
            m_["x"], m_["y"] = round(float(-HX + cc.mean() * RES), 3), round(float(HY - rr.mean() * RES), 3)
        ctx = dict(n=n, prev=prev, lab=lab, klab=klab, pres=pres, present_ids=present_ids, region=region, blobs=blobs,
                   marks=marks, rgb_n=rgb_n)
        if sc: sc.night(ctx)                                      # the scorer's match, before the blobs lose their slices
        for b in blobs:
            rr, cc = np.nonzero(lab[b["sl"]] == b["id"]); r0, c0 = b["sl"][0].start, b["sl"][1].start
            b["x"] = round(float(-HX + (cc.mean() + c0) * RES), 3); b["y"] = round(float(HY - (rr.mean() + r0) * RES), 3)
            b["bbox_rc"] = [b["sl"][0].start, b["sl"][1].start, b["sl"][0].stop, b["sl"][1].stop]; del b["sl"]
        np.savez_compressed(run / f"detect_n{n}.npz", lab=lab.astype(np.int32), klab=klab.astype(np.int32))
        (run / f"detect_n{n}.json").write_text(json.dumps({"blobs": blobs, "marks": marks}, indent=1))
        if sc: sc.report(ctx)
        else:
            kinds = {k: sum(b["kind"] == k for b in blobs) for k in ("present", "gone", "floor_change")}
            print(f"night {n} (baseline {prev}): blobs {kinds}; known marks {len(marks)}; compared {round(region.sum() * RES * RES, 1)} m2")
    if sc: sc.finish()


if __name__ == "__main__":
    main()
