#!/usr/bin/env python3
"""How close a floor picture is to the floor itself ([FHA]): scored against the sim's true floor texture. A scorer only:
the truth never reaches anything the robot runs. The picture's own frame (the pose graph's, anchored where night 1
began) may sit a few cm and a degree off the sim's world, which is harmless (every night shares it) but would swamp a
local score, so the picture is first laid on the truth as a whole (one turn and shift, by correlation of the floor's
structure at 2 cm), then scored per 40 cm patch: the correlation of its structure with the truth's, after the patch's
own best shift (within +-15 cm: the map's slow bend, which the robot never sees, isn't held against it), and that shift (the picture's local warp).
Usage: floor_fidelity.py <picture.npz> --seed 70 [--night 1]   (dimos venv)"""
import argparse, math
from pathlib import Path
import numpy as np
from PIL import Image
from scipy import ndimage as ndi
from make_nights import TEX_DIR

RES, P, LOCAL = 0.01, 40, 15
EDGE_CM = 5                      # [nv5]: the seen floor's edge, not judged


def structure(g, V, s=10):
    """brightness relative to its surroundings (mean and spread over s cells): lighting and exposure fall away"""
    V = V.astype(np.float32); den = np.maximum(ndi.gaussian_filter(V, s), 1e-3)
    mu = ndi.gaussian_filter(g * V, s) / den; sd = np.sqrt(np.maximum(ndi.gaussian_filter((g - mu) ** 2 * V, s) / den, 1))
    return np.where(V > 0, (g - mu) / sd, 0)


def truth(seed, night, shape):
    if seed is None or not (TEX_DIR / f"house_floor_s{seed}_n{night}.png").exists():   # a real night ([V35]): no truth to score
        raise SystemExit(f"floor map error: sim only (needs the sim's own floor for house {seed}, night {night}); skipped")
    t = Image.open(TEX_DIR / f"house_floor_s{seed}_n{night}.png").convert("RGB").resize((shape[1], shape[0]), Image.BOX)
    return np.asarray(t).astype(np.float32)


def lay_on(A, V, B, turns=np.arange(-6.0, 6.01, 0.5), reach_cm=80):
    """the one turn (deg) and shift (rows, cols) that best lay picture structure A (seen where V) on truth structure B,
    at 2 cm by masked correlation: the frame's offset, not the picture's quality"""
    from scipy.signal import fftconvolve
    a = A[::2, ::2]; v = V[::2, ::2].astype(np.float32); b = B[::2, ::2]; m = reach_cm // 2; best = None
    for td in turns:
        ar = ndi.rotate(a * v, td, reshape=False, order=1); vr = ndi.rotate(v, td, reshape=False, order=1)
        num = fftconvolve(b, ar[::-1, ::-1], mode="same"); n = fftconvolve(np.ones_like(b), vr[::-1, ::-1], mode="same")
        c = num / np.maximum(n, 1); cy, cx = np.array(c.shape) // 2
        w = c[cy - m:cy + m + 1, cx - m:cx + m + 1]; k = np.unravel_index(np.argmax(w), w.shape)
        if best is None or w[k] > best[0]: best = (float(w[k]), td, (k[0] - m) * 2, (k[1] - m) * 2)
    return best[1], best[2], best[3]


def frame_from_path(corr, traj):
    """the picture's frame against the sim's (turn deg, rows, cols), from the night's corrected path against its true path
    (the one rigid move that lays one on the other): exact where image correlation over a warped picture wanders"""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
    from houndkeeper.localize import Localizer
    T = np.genfromtxt(traj, delimiter=",", names=True); z = np.load(corr) if corr else None
    loc = Localizer(times=list(z["times"]), Ts=list(z["Ts"])) if z is not None else None
    A = []
    for t, x, y in zip(T["wall_t"], T["odom_x"], T["odom_y"]):
        C = loc.correction(t) if loc else np.eye(3); A.append(C[:2, :2] @ [x, y] + C[:2, 2])
    A = np.array(A); B = np.c_[T["true_x"], T["true_y"]]; ma, mb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ma).T @ (B - mb)); R = (U @ Vt).T
    if np.linalg.det(R) < 0: Vt[-1] *= -1; R = (U @ Vt).T
    return R, mb - R @ ma                                   # world_true = R @ picture + t


def to_true_frame(img, R, t, order=1):
    """resample a picture (rows, cols at RES, centred grid) into the sim's frame"""
    H, W = img.shape[:2]; HX, HY = W * RES / 2, H * RES / 2
    r, c = np.mgrid[:H, :W]; X = -HX + (c + 0.5) * RES; Y = HY - (r + 0.5) * RES
    P = np.stack([X - t[0], Y - t[1]], -1) @ R          # inverse: picture = R^T (true - t)
    cc = (P[..., 0] + HX) / RES - 0.5; rr = (HY - P[..., 1]) / RES - 0.5
    if img.ndim == 2: return ndi.map_coordinates(img.astype(np.float32), [rr, cc], order=order, cval=0)
    return np.stack([ndi.map_coordinates(img[..., k].astype(np.float32), [rr, cc], order=order, cval=0) for k in range(img.shape[2])], -1)


def score(rgb, V, T, frame=None):
    """per patch: (correlation after its best local shift, that shift in cm), over patches with floor and structure;
    frame: (R, t) from frame_from_path, else found by lay_on"""
    if frame is not None:
        rgb = to_true_frame(rgb, *frame); V = to_true_frame(V.astype(np.float32), *frame, order=0) > 0.5
    A = structure(rgb.mean(-1), V); B = structure(T.mean(-1), np.ones(V.shape, bool))
    td, dr, dc = (0.0, 0, 0) if frame is not None else lay_on(A, V, B)
    A = np.roll(ndi.rotate(A * V, td, reshape=False, order=1), (dr, dc), (0, 1))
    V = np.roll(ndi.rotate(V.astype(np.float32), td, reshape=False, order=0) > 0.5, (dr, dc), (0, 1))
    from scipy.signal import fftconvolve
    out = []
    for r in range(LOCAL, V.shape[0] - P - LOCAL, P):
        for c in range(LOCAL, V.shape[1] - P - LOCAL, P):
            if V[r:r + P, c:c + P].mean() < 0.95 or B[r:r + P, c:c + P].std() < 0.3: continue
            a = A[r:r + P, c:c + P]; a = (a - a.mean()) / (a.std() + 1e-6)          # every placement within +-LOCAL at
            Bw = B[r - LOCAL:r + P + LOCAL, c - LOCAL:c + P + LOCAL]                 # once: normalised cross-correlation
            k = np.ones((P, P)); s1 = fftconvolve(Bw, k, mode="valid"); s2 = fftconvolve(Bw ** 2, k, mode="valid")
            num = fftconvolve(Bw, a[::-1, ::-1], mode="valid"); n = P * P
            den = np.sqrt(np.maximum(s2 - s1 ** 2 / n, 1e-6)) * math.sqrt(n)
            cc = num / den; i, j = np.unravel_index(np.argmax(cc), cc.shape)
            out.append((float(cc[i, j]), math.hypot(i - LOCAL, j - LOCAL)))
    return np.array(out), (td, dr, dc)


def map_error(rgb, V, T, exclude):
    """[nv5] the floor map error: the share of the floor the map shows (V, less exclude) where it differs from the true
    floor T by the detector's own test (its colour, relative to the surrounding floor, so a lamp's pool is not a
    difference; its threshold and slack). rgb and T on the same grid, in the same frame"""
    import confirm_stream as cs
    from detect_changes2 import feats, tolerant_diff
    J = ndi.binary_erosion(V, iterations=EDGE_CM) & ~exclude          # the seen floor's edge not trusted (as the confirmer)
    if not J.any(): return float("nan")
    ca, ra = feats(rgb.astype(np.uint8), V); cb, rb = feats(np.clip(T, 0, 255).astype(np.uint8), V)   # the truth judged
                                                                         # over the same floor: its surroundings are the map's
    dC, _br = tolerant_diff(ca, ra, cb, rb, slack=cs.CONFIRM["slack"])
    return float((dC[J] > cs.CONFIRM["tc"]).mean())


def placement(rgb, V, T, exclude, patch=P, reach=LOCAL, min_match=0.5):
    """the floor map's placement against the true floor, in mm: per patch (patch cells square, overlapping by half, every
    cell seen and open, with structure in the truth) the shift, to a fraction of a cell, that best lays the map's
    structure on the truth's within +-reach cells: {(row, col): (x east, y north)}, a patch matched no better than
    min_match, or at the search's edge, left out. [nv5]'s error forgives what the detector forgives (2 cm of slack,
    colour only), so it cannot see a map placed a few mm or cm off (decision [qLu]'s control, 4 Oct 2026); this can"""
    from scipy.signal import fftconvolve
    J = ndi.binary_erosion(V, iterations=EDGE_CM) & ~exclude
    A = structure(rgb.mean(-1), V); B = structure(T.mean(-1), np.ones(V.shape, bool)); out = {}
    sub = lambda m_, c0, p_: 0.0 if m_ - 2 * c0 + p_ >= 0 else float(np.clip(0.5 * (m_ - p_) / (m_ - 2 * c0 + p_), -0.5, 0.5))
    k = np.ones((patch, patch)); n = patch * patch
    for r in range(reach, V.shape[0] - patch - reach, patch // 2):
        for c in range(reach, V.shape[1] - patch - reach, patch // 2):
            if J[r:r + patch, c:c + patch].mean() < 0.95 or B[r:r + patch, c:c + patch].std() < 0.3: continue
            a = A[r:r + patch, c:c + patch]; a = (a - a.mean()) / (a.std() + 1e-6)
            Bw = B[r - reach:r + patch + reach, c - reach:c + patch + reach]
            s1 = fftconvolve(Bw, k, mode="valid"); s2 = fftconvolve(Bw ** 2, k, mode="valid")
            cc = fftconvolve(Bw, a[::-1, ::-1], mode="valid") / (np.sqrt(np.maximum(s2 - s1 ** 2 / n, 1e-6)) * math.sqrt(n))
            i, j = np.unravel_index(np.argmax(cc), cc.shape)
            if i in (0, cc.shape[0] - 1) or j in (0, cc.shape[1] - 1) or cc[i, j] < min_match: continue
            di = i - reach + sub(cc[i - 1, j], cc[i, j], cc[i + 1, j]); dj = j - reach + sub(cc[i, j - 1], cc[i, j], cc[i, j + 1])
            out[(r, c)] = (-dj * RES * 1000, di * RES * 1000)     # the map's floor lies at the truth's (r + di, c + dj): the
    return out                                                     # map sits that much the other way


def placement_summary(s, ref=None):
    """placement's patches summed up: {patches, abs_mm_mean / _p90: each patch's distance from the truth; median_mm: the
    shift most of the map shares; off_mm_median: each patch's distance from that}; with ref (another map's placement of
    the same floor) also paired_mm: the mean, over the patches both matched, of this map's distance less ref's (negative:
    nearer the truth)"""
    if not s: return {"patches": 0}
    a = np.array(list(s.values())); med = np.median(a, 0); ab = np.hypot(*a.T)
    out = {"patches": int(len(a)), "abs_mm_mean": round(float(ab.mean()), 2), "abs_mm_p90": round(float(np.percentile(ab, 90)), 2),
           "median_mm": [round(float(v), 2) for v in med], "off_mm_median": round(float(np.median(np.hypot(*(a - med).T))), 2)}
    if ref is not None:
        both = [k for k in s if k in ref]
        out["paired_mm"] = round(float(np.mean([math.hypot(*s[k]) - math.hypot(*ref[k]) for k in both])), 2) if both else None
        out["paired_patches"] = len(both)
    return out


def placement_of(picture, seed, night, traj, corrections, run):
    """a night's map laid on the true floor by its path's rigid fit (as error_of): its placement per patch (mm), as
    placement gives it"""
    import confirm_stream as cs, floor_agreement as fa
    z = np.load(picture); V = np.isfinite(z["dist"].astype(np.float32)); rgb = z["rgb"].astype(np.float32)
    R, t = frame_from_path(corrections, traj)
    rgb = to_true_frame(rgb, R, t); V = to_true_frame(V.astype(np.float32), R, t, order=0) > 0.5
    occ = truth_structures(seed, V.shape); band = ndi.distance_transform_edt(~occ) * RES < cs.CONFIRM["band_m"]
    rug, chairs = fa.regions(Path(run), seed, (1, 2), V.shape)
    return placement(rgb, V, truth(seed, night, V.shape), band | rug | chairs)


def truth_structures(seed, shape):
    """the house's walls and furniture as the sim drew them (make_house's plan for this seed), on the picture's grid: the
    one mask every map is judged with, so maps built differently are scored over the same floor (scoring only)"""
    import make_house as mh
    mh.use_layout(seed); g = mh.base_grid(); occ = np.zeros(shape, bool); HX, HY = shape[1] * RES / 2, shape[0] * RES / 2
    for y in range(mh.H):
        for x in range(mh.W):
            if g[y][x]:
                cx, cy = mh.cell_to_world(x, y); r0 = int((HY - cy - mh.CELL / 2) / RES); c0 = int((cx - mh.CELL / 2 + HX) / RES)
                occ[max(r0, 0):r0 + int(mh.CELL / RES), max(c0, 0):c0 + int(mh.CELL / RES)] = True
    return occ


def error_of(picture, seed, night, traj, corrections, run):
    """a night's map, laid on the true floor by its path's rigid fit: (floor map error over the open floor, m2 judged)"""
    import json as _json
    import confirm_stream as cs, floor_agreement as fa
    z = np.load(picture); V = np.isfinite(z["dist"].astype(np.float32)); rgb = z["rgb"].astype(np.float32)
    R, t = frame_from_path(corrections, traj)
    rgb = to_true_frame(rgb, R, t); V = to_true_frame(V.astype(np.float32), R, t, order=0) > 0.5
    occ = truth_structures(seed, V.shape)                                  # the same mask for every map (the maps' own
    band = ndi.distance_transform_edt(~occ) * RES < cs.CONFIRM["band_m"]    # sat in their own frames: 59-83 m2, 1 Oct)
    rug, chairs = fa.regions(Path(run), seed, (1, 2), V.shape)
    ex = band | rug | chairs
    return map_error(rgb, V, truth(seed, night, V.shape), ex), float((V & ~ex).sum() * RES * RES)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("picture"); ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--night", type=int, default=1)
    ap.add_argument("--traj", default=None, help="the night's true path (traj.csv): the frame from it (scoring only)")
    ap.add_argument("--corrections", default=None, help="the corrections the picture was painted with (none: odometry's frame)")
    ap.add_argument("--error", default=None, metavar="RUN_DIR", help="[nv5] the floor map error instead (needs --traj; the run dir for the rug and chairs)")
    ap.add_argument("--built", default="", help="--error: how the map was built, for the log")
    ap.add_argument("--log", default=str(Path(__file__).resolve().parents[2] / "data" / "eval" / "floor_map_error.jsonl"))
    a = ap.parse_args()
    if a.error:
        import json as _json, time as _time
        e, m2 = error_of(a.picture, a.seed, a.night, a.traj, a.corrections, a.error)
        rec = dict(date=_time.strftime("%Y-%m-%dT%H:%M"), seed=a.seed, night=a.night, built=a.built, picture=str(a.picture), error=round(e, 5), judged_m2=round(m2, 1))
        Path(a.log).parent.mkdir(parents=True, exist_ok=True); open(a.log, "a").write(_json.dumps(rec) + "\n")
        print(f"s{a.seed} n{a.night}: floor map error {e:.2%} of {m2:.1f} m2 of open floor ({a.built or 'unlabelled'})"); return
    z = np.load(a.picture); V = np.isfinite(z["dist"].astype(np.float32)); rgb = z["rgb"].astype(np.float32)
    fr_ = frame_from_path(a.corrections, a.traj) if a.traj else None
    s, (td, dr, dc) = score(rgb, V, truth(a.seed, a.night, V.shape), fr_)
    if fr_ is not None: td, dr, dc = math.degrees(math.atan2(fr_[0][1, 0], fr_[0][0, 0])), int(round(-fr_[1][1] / RES)), int(round(fr_[1][0] / RES))
    print(f"s{a.seed}: frame off the truth by {td:+.1f} deg, ({dc:+d}, {-dr:+d}) cm; per 40 cm patch ({len(s)}): correlation with "
          f"the true floor median {np.median(s[:, 0]):.2f} (p10 {np.percentile(s[:, 0], 10):.2f}); local warp median "
          f"{np.median(s[:, 1]):.1f} cm, p90 {np.percentile(s[:, 1], 90):.1f}")


if __name__ == "__main__":
    main()
