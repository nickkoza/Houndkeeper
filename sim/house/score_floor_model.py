#!/usr/bin/env python3
"""The persistent floor model (floor_model.py) built night after night and scored at each step against the sim's
unlit floor texture (patch-wise structure match and shift). A scorer: the generator's texture is read here, never by the
floor model the robot builds ([1B2]).
Usage: score_floor_model.py --src runs/nights_s7_v2b --nights 8 [--out ../media/floor]"""
import argparse, json
from pathlib import Path
import numpy as np
from PIL import Image
from make_nights import TEX_DIR
from floor_model import build, gray, structure


def score(model, MV, night, seed=7):
    tex = np.asarray(Image.open(TEX_DIR / f"house_floor_s{seed}_n{night}.png").convert("RGB"))
    h, w = min(tex.shape[0], model.shape[0]), min(tex.shape[1], model.shape[1]); tex, m, V = tex[:h, :w], model[:h, :w], MV[:h, :w]
    A, B = structure(gray(m), V), structure(gray(tex), np.ones_like(V))
    sh, fd = [], []
    for r in range(0, h - 40, 40):
        for c in range(0, w - 40, 40):
            sl = (slice(r, r + 40), slice(c, c + 40))
            if V[sl].mean() < 0.9 or B[sl].std() < 0.3: continue
            X = np.fft.fft2(A[sl]) * np.conj(np.fft.fft2(B[sl])); X /= np.abs(X) + 1e-6
            cc = np.fft.ifft2(X).real; dy, dx = np.unravel_index(cc.argmax(), cc.shape)
            dy = dy - 40 if dy > 20 else dy; dx = dx - 40 if dx > 20 else dx
            fd.append(np.corrcoef(A[sl].ravel(), np.roll(B[sl], (dy, dx), (0, 1)).ravel())[0, 1]); sh.append(np.hypot(dy, dx))
    fd, sh = np.array(fd), np.array(sh); conf = fd >= 0.3
    return {"coverage": float(V.mean()), "fidelity": float(np.median(fd)), "confident": float(conf.mean()),
            "wavy_cm": float(np.median(sh[conf])) if conf.any() else None}


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("--src", default="runs/nights_s7_v2b"); ap.add_argument("--nights", type=int, default=8)
    ap.add_argument("--out", default="../media/floor"); ap.add_argument("--only-last", action="store_true"); ap.add_argument("--seed", type=int, default=7); a = ap.parse_args(argv)
    src, out = Path(a.src), Path(a.out); out.mkdir(parents=True, exist_ok=True); curve = []
    for n in (range(a.nights, a.nights + 1) if a.only_last else range(1, a.nights + 1)):
        model, MV = build(list(range(1, n + 1)), src)
        s = score(model, MV, n, a.seed); curve.append({"nights": n, **s})
        Image.fromarray(model).save(out / f"floor_model_after_night{n}.png"); Image.fromarray(model).save(out / "floor_model_latest.png")
        print(f"after {n} night(s): fidelity {s['fidelity']:.2f}, confident patches {s['confident']:.0%}, waviness {s['wavy_cm']} cm, coverage {s['coverage']:.0%}", flush=True)
    if not a.only_last: (src / "floor_model_curve.json").write_text(json.dumps(curve, indent=1))


if __name__ == "__main__":
    main()
