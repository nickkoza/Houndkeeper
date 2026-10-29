#!/usr/bin/env python3
"""The wrist stereo pair (waymark 0011 [A8K]) scored offline on looks saved by width_bench.py with HK_WRIST_STEREO=1:
depth from OpenCV's semi-global matching on the left and right frames, the cameras taken as calibrated (parallel, the
baseline along the left one's x; the rendered right one is off by a calibration's residual), each pixel's height above
the floor, and what stands above it as the item. Scored against the sim's true pixels, beside the detector's own mask.
Seconds a look, no GPU. Usage: stereo_eval.py looks.pkl [--min-height-mm 8]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import argparse, pickle, sys                                                 # the benches' own files, written on this machine
import numpy as np
sys.path.insert(0, str(_HK / "sim" / "arm")); import bench_io
import cv2
from scipy import ndimage as ndi


OPTS = {"block": 5, "uniq": 10, "lr": False, "max_h": 0.40}
MATCH_W = 640       # stereo is matched at this width whatever the cameras deliver (the detector takes the full frame)


def heights(cv, block=None, uniq=None, lr=None, max_h=None):
    """each left pixel's height above the floor (m; NaN where the match failed), from the pair. lr: keep only pixels
    whose match agrees both ways (left to right and back), which drops the band beside an item's edge that one camera
    sees and the other doesn't (the matcher smears the item's depth over it)"""
    block = block or OPTS["block"]; uniq = OPTS["uniq"] if uniq is None else uniq; lr = OPTS["lr"] if lr is None else lr
    max_h = max_h or OPTS["max_h"]
    full = cv["img"].shape[:2]
    if full[1] > MATCH_W:                                                   # match at 640 x 360 and scale the heights back up:
        n_ = full[1] // MATCH_W                                             # as good on the same frames as the whole 1280 x 720
        h_small = heights(binned(cv, n_), block, uniq, lr, max_h)           # (29 Sep: IoU 0.79 against 0.73), a quarter the work
        P_small = heights.last_points
        up = lambda a: cv2.resize(a, (full[1], full[0]), interpolation=cv2.INTER_NEAREST)
        heights.last_points = np.stack([up(P_small[..., k]) for k in range(3)], -1)
        return up(h_small)
    L = cv2.cvtColor(cv["img"], cv2.COLOR_RGB2GRAY); Rr = cv2.cvtColor(cv["img_r"], cv2.COLOR_RGB2GRAY)
    H, W = L.shape; f = (H / 2) / np.tan(np.radians(cv["fovy"]) / 2); B = float(cv["baseline"])
    n = int(np.ceil(f * B / 0.10 / 16) * 16)                                 # disparities down to 10 cm away
    sg = cv2.StereoSGBM_create(minDisparity=0, numDisparities=n, blockSize=block, P1=8 * block * block, P2=32 * block * block,
                               uniquenessRatio=uniq, speckleWindowSize=100, speckleRange=2, disp12MaxDiff=1, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    disp = sg.compute(L, Rr).astype(np.float32) / 16.0
    ok = disp > 0.5
    if lr:                                                                  # the right image's own disparities, mirrored
        sr = cv2.StereoSGBM_create(minDisparity=0, numDisparities=n, blockSize=block, P1=8 * block * block, P2=32 * block * block,
                                   uniquenessRatio=uniq, speckleWindowSize=100, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        dr = sr.compute(cv2.flip(Rr, 1), cv2.flip(L, 1)).astype(np.float32)[:, ::-1] / 16.0
        xs = np.clip((np.arange(W)[None, :] - disp).round().astype(int), 0, W - 1)
        back = np.take_along_axis(dr, xs, axis=1); ok &= np.abs(back - disp) <= 1.0
    Z = np.where(ok, f * B / np.maximum(disp, 0.5), np.nan)
    vv, uu = np.mgrid[0:H, 0:W]
    Pc = np.stack([(uu - W / 2) * Z / f, -(vv - H / 2) * Z / f, -Z], -1)    # camera frame: x right, y up, looking down -z
    Pw = np.asarray(cv["c"]) + Pc @ np.asarray(cv["R"]).T
    h = Pw[..., 2]; h[~ok | cv["robot"] | (h > max_h) | (h < -0.02)] = np.nan   # beyond any toy, or below the floor: a bad match
    heights.last_points = Pw                                                 # each pixel's point in the world, for measuring
    return h


def standing(cv, min_h=0.008):
    """the pixels standing more than min_h above the floor, cleaned: specks out, gaps closed, holes filled"""
    h = heights(cv); up = np.nan_to_num(h, nan=0.0) > min_h
    up = ndi.binary_opening(up, iterations=1)
    return ndi.binary_fill_holes(ndi.binary_closing(up, iterations=3)) & ~cv["robot"], h


def binned(cv, n):
    """a look as the same cameras would give it binned n x n: the frames averaged, the masks thinned, the lens the same"""
    H, W = cv["img"].shape[:2]; size = (W // n, H // n)
    down = lambda im: cv2.resize(im, size, interpolation=cv2.INTER_AREA)
    mdown = lambda m: cv2.resize(m.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST).astype(bool)
    out = dict(cv, img=down(cv["img"]), img_r=down(cv["img_r"]), robot=mdown(cv["robot"]))
    if cv.get("truth") is not None: out["truth"] = mdown(cv["truth"])       # the sim's truth: benches only, never a live look
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("pkls", nargs="+"); ap.add_argument("--min-height-mm", type=float, default=8.0)
    ap.add_argument("--block", type=int, default=5); ap.add_argument("--uniq", type=int, default=10); ap.add_argument("--lr", action="store_true")
    ap.add_argument("--quiet", action="store_true"); ap.add_argument("--bin", type=int, default=1, help="match the frames binned n x n")
    a = ap.parse_args(); OPTS.update(block=a.block, uniq=a.uniq, lr=a.lr); ious, det_ious, per_kind = [], [], {}
    for p in a.pkls:
        for r in bench_io.load(p):
            for cv, det in r["looks"]:
                if cv.get("img_r") is None or cv.get("truth") is None: continue
                if a.bin > 1: cv = binned(cv, a.bin)
                t = cv["truth"] & ~cv["robot"]
                if t.sum() < 50: continue
                m, h = standing(cv, a.min_height_mm / 1000)
                lab, _ = ndi.label(m); ids = lab[t & m]
                m = lab == np.bincount(ids).argmax() if len(ids) else np.zeros_like(m)
                iou = (m & t).sum() / max((m | t).sum(), 1); ious.append(iou)
                dm = None if det is None else (det["mask"] if det["mask"].shape == t.shape else
                                               cv2.resize(det["mask"].astype(np.uint8), t.shape[::-1], interpolation=cv2.INTER_NEAREST).astype(bool))
                di = None if dm is None else (dm & t).sum() / max((dm | t).sum(), 1)
                if di is not None: det_ious.append(di)
                per_kind.setdefault(r["kind"], []).append((iou, di, float(np.nanmax(np.where(t, h, np.nan))) if np.isfinite(h[t]).any() else np.nan))
    ious, det_ious = np.array(ious), np.array(det_ious)
    print(f"looks {len(ious)}: stereo IoU median {np.median(ious):.2f}, >=0.7 {np.mean(ious >= 0.7):.0%};  "
          f"detector's mask IoU median {np.median(det_ious):.2f}, >=0.7 {np.mean(det_ious >= 0.7):.0%}")
    for k, v in ([] if a.quiet else sorted(per_kind.items())):
        s = np.array([x[0] for x in v]); dd = [x[1] for x in v if x[1] is not None]; hh = [x[2] for x in v]
        print(f"  {k[:26]:26s} looks {len(v):2d}  stereo {np.median(s):.2f}  detector {np.median(dd) if dd else float('nan'):.2f}  "
              f"tallest seen {np.nanmedian(hh) * 100 if np.isfinite(hh).any() else float('nan'):.1f} cm")


if __name__ == "__main__":
    main()
