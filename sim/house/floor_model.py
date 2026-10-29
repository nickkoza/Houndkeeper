#!/usr/bin/env python3
"""Persistent floor model, refined night after night.

For nights 1..n: remove each night's lighting (divide by a smooth lighting field estimated against the
current model), align each night to the model patch by patch (phase correlation on floor structure,
smoothed displacement field), then take a robust per-pixel median across nights. Anything present on
fewer than half the nights (clutter, shadows, artifacts) drops out. Two refinement passes per step.
Robot-side: it reads only the night's own floor pictures. score_floor_model.py builds it night by night and scores
each step against the sim's unlit floor texture (patch-wise structure match and shift)."""
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi

P = 48                                   # alignment patch (cm)


def gray(rgb): return rgb.astype(np.float32).mean(-1)


def smooth(x, V, s):
    den = np.maximum(ndi.gaussian_filter(V.astype(np.float32), s), 1e-3)
    return ndi.gaussian_filter(np.where(V, x, 0), s) / den


def structure(g, V, s=8):
    mu = smooth(g, V, s); sd = np.sqrt(np.maximum(smooth((g - mu) ** 2, V, s), 1)); return np.where(V, (g - mu) / sd, 0)


def align(img, V, ref, RV):
    """Per-patch shift aligning img to ref (floor structure), smoothed into a field; returns warped img, V."""
    A, B = structure(gray(img), V), structure(gray(ref), RV)
    H, W = V.shape; ny, nx = H // P, W // P
    dy = np.zeros((ny, nx)); dx = np.zeros((ny, nx)); wt = np.zeros((ny, nx))
    for i in range(ny):
        for j in range(nx):
            sl = (slice(i * P, (i + 1) * P), slice(j * P, (j + 1) * P))
            m = V[sl] & RV[sl]
            if m.mean() < 0.8 or B[sl].std() < 0.3: continue
            X = np.fft.fft2(A[sl]) * np.conj(np.fft.fft2(B[sl])); X /= np.abs(X) + 1e-6
            cc = np.fft.ifft2(X).real; py, px = np.unravel_index(cc.argmax(), cc.shape)
            py = py - P if py > P // 2 else py; px = px - P if px > P // 2 else px
            if abs(py) > 10 or abs(px) > 10: continue      # beyond plausible drift: ignore
            dy[i, j], dx[i, j], wt[i, j] = py, px, cc.max()
    # confidence-weighted smoothing of the sparse field, then upsample
    s = 1.5; W_ = ndi.gaussian_filter(wt, s) + 1e-6
    fy = ndi.gaussian_filter(dy * wt, s) / W_; fx = ndi.gaussian_filter(dx * wt, s) / W_
    fy = ndi.zoom(fy, (H / ny, W / nx), order=1)[:H, :W]; fx = ndi.zoom(fx, (H / ny, W / nx), order=1)[:H, :W]
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    out = np.stack([ndi.map_coordinates(img[..., k].astype(np.float32), [yy + fy, xx + fx], order=1) for k in range(3)], -1)
    Vw = ndi.map_coordinates(V.astype(np.float32), [yy + fy, xx + fx], order=0) > 0.5
    return out, Vw, float(np.median(np.hypot(dy[wt > 0], dx[wt > 0]))) if (wt > 0).any() else 0.0


def build(nights, src, passes=2):
    M = {n: np.load(src / f"mosaic_n{n}.npz") for n in nights}
    imgs = {n: M[n]["rgb"].astype(np.float32) for n in nights}; Vs = {n: np.isfinite(M[n]["dist"].astype(np.float32)) for n in nights}
    # init: lighting-flattened first night
    n0 = nights[0]; g = gray(imgs[n0]); L = smooth(g, Vs[n0], 60)[..., None]
    model = np.where(Vs[n0][..., None], imgs[n0] / np.maximum(L, 1) * 128, 0); MV = Vs[n0].copy()
    for _ in range(passes):
        stack, masks = [], []
        for n in nights:
            im, V, _ = align(imgs[n], Vs[n], model, MV) if len(nights) > 1 or _ else (imgs[n], Vs[n], 0)
            ratio = np.where(V & MV, gray(im) / np.maximum(gray(model), 1), np.nan)
            Lf = smooth(np.nan_to_num(ratio, nan=1.0), V & MV, 60)       # tonight's lighting relative to the model
            Lf = np.where(MV, Lf, smooth(gray(im), V, 60) / 128)
            stack.append(np.where(V[..., None], im / np.maximum(Lf, 1e-3)[..., None], np.nan)); masks.append(V)
        S = np.stack(stack); model = np.nan_to_num(np.nanmedian(S, 0)); MV = np.any(np.stack(masks), 0)
        model = np.clip(model * (128 / max(np.median(gray(model)[MV]), 1)), 0, 255)
    return model.astype(np.uint8), MV
