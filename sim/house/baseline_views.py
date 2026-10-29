#!/usr/bin/env python3
"""A baseline night's own views of the floor, for confirm_stream.py --views ([KkQ], confirm_stream.Views): every
EVERY-th head frame, placed as the night's floor picture was (the house's frame, the robot's stored frame lag for the recording's clock
taken off, [gJq]), painted
onto the floor grid by the confirmer's own painter, with where the camera stood and faced.
Usage: baseline_views.py <run dir> <night> <memory.db> [--corrections mosaic_n<k>.corrections.npz] --out views_n<k>.npz"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import argparse, time
from pathlib import Path

import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from houndkeeper.lens import Rectifier
from houndkeeper.localize import Localizer
import confirm_stream as cs
from floor_mosaic2 import arr, frame_lags, LAG_FILE, clock_of, add_clock_args
from houndkeeper import calibrate as _cal

EVERY = 8                                             # ~0.4 s apart at 20 frames a second: 12 cm of walking (est.)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("night", type=int); ap.add_argument("db")
    ap.add_argument("--corrections", default=None); ap.add_argument("--out", required=True); ap.add_argument("--every", type=int, default=EVERY)
    ap.add_argument("--stack", type=int, default=1, help="frames stacked into each view (each --step-th from its first): less noise, more floor")
    ap.add_argument("--step", type=int, default=2)
    ap.add_argument("--mount", default=None, help="the head camera's calibrated mount (.json, calibrate_mount.py)")
    ap.add_argument("--relief", default="auto", help="the floor's relief (.npy, floor_selfcal.py relief): 'auto' (default): the run's own when painting in its frame (--corrections it names), else the plane z = 0, said; 'none': the plane; or a path")
    ap.add_argument("--lag-file", default=str(LAG_FILE), help="the robot's frame lag (.json from calibrate_lag.py)")
    ap.add_argument("--res", type=float, default=cs.RES, help="the grid painted on (m a cell): 0.01, or a finer one dividing it for the near grid ([H9w], confirm_stream --near-res)")
    ap.add_argument("--rmax", type=float, default=cs.CONFIRM["rmax"], help="how far ahead each view paints (m): the near grid's views only its reach")
    add_clock_args(ap)
    a = ap.parse_args(); t0 = time.time(); k = int(round(cs.RES / a.res)); cs.grid(a.res)
    base, Vb, band, hmap, _ = cs.baseline(Path(a.run), a.night + 1)          # the painter's lines of sight: the nights so far
    st = SqliteStore(path=a.db); st.start()
    ci = next(iter(st.stream("camera_info"))).data; rect = Rectifier.from_camera_info(ci)
    pose, T_bo = cs.poses(st)
    lag, note = frame_lags(st, "robot", ["color_image"], a.lag_file, check=False, clock=clock_of(a.db, a.clock, a.traj))["color_image"]
    print(f"color_image: frame lag {lag:.3f} s, {note}", flush=True)   # the robot's, on the recording's clock ([gJq], [55G])
    loc = None
    if a.corrections: z = np.load(a.corrections); loc = Localizer(times=list(z["times"]), Ts=list(z["Ts"]))
    mount = None
    if a.mount:
        import json, math; from houndkeeper import calibrate as cal
        mount, mount_off = cal.mount(json.loads(open(a.mount).read()))
    up = lambda x: cs.upsample(x, k)                                # the 1 cm baselines on the grid painted (k = 1: as they are)
    painter = cs.Confirmer(up(base), up(Vb), up(band), up(hmap), rect.K, (ci.width, ci.height),
                           __import__("argparse").Namespace(**dict(cs.CONFIRM, rmax=a.rmax)), res=a.res)
    from houndkeeper import floor_relief as frl                     # the floor where it stands ([yFg])
    painter.relief, rpath, rnote = frl.resolve(a.relief, a.run, a.corrections) if a.corrections else frl.resolve_odometry(a.relief, a.run)
    if painter.relief is not None: painter.relief = up(painter.relief)
    print(f"floor: {rnote}", flush=True)
    cal_stamp = _cal.stamp(a.mount, a.lag_file, rpath)              # what the views were painted with, kept with them ([Q9k])
    C, F, sls, patches, ts, Ts = [], [], [], [], [], []
    acc = None                                                       # the view being stacked: (anchor, sums, counts, box)
    def close(acc):
        if acc is None: return
        (t_, c_, f_, T_), S_, N_, (r0, r1, c0, c1) = acc
        ts.append(t_); C.append(c_); F.append(f_); Ts.append(T_); sls.append((r0, r1, c0, c1))
        patches.append(np.where(N_[..., None] > 0, S_ / np.maximum(N_, 1)[..., None], 0).astype(np.uint8))
    for k, o in enumerate(st.stream("color_image")):
        j = k % a.every
        if j >= a.stack * a.step or j % a.step: continue            # the view's frames: every step-th of its first few
        M = pose(o.ts - lag)
        if M is None: continue
        T = (M if loc is None else loc.apply(M, o.ts - lag)) @ T_bo
        if mount is not None:                                       # the camera truly sits a little turned, and a little off ([KkQ])
            Tb = M if loc is None else loc.apply(M, o.ts - lag); T = T.copy(); T[:3, :3] = T[:3, :3] @ mount; T[:3, 3] += Tb[:3, :3] @ mount_off
        p = painter.paint(rect(arr(o.data)), T)
        if p is None: continue
        sl, _, _, _, patch, V = p
        if j == 0:
            close(acc); c, f = cs.Views.of_frame(T); acc = [(o.ts - lag, c, f, T.copy()), None, None, None]
        if acc is None: continue
        box = (sl[0].start, sl[0].stop, sl[1].start, sl[1].stop)
        if acc[3] is None: acc[1], acc[2], acc[3] = np.zeros(patch.shape, np.float32), np.zeros(V.shape, np.int16), box
        ob = acc[3]; nb = (min(ob[0], box[0]), max(ob[1], box[1]), min(ob[2], box[2]), max(ob[3], box[3]))
        if nb != ob:                                                 # grow the stack's box to take this frame
            S2 = np.zeros((nb[1] - nb[0], nb[3] - nb[2], 3), np.float32); N2 = np.zeros(S2.shape[:2], np.int16)
            S2[ob[0] - nb[0]:ob[1] - nb[0], ob[2] - nb[2]:ob[3] - nb[2]] = acc[1]; N2[ob[0] - nb[0]:ob[1] - nb[0], ob[2] - nb[2]:ob[3] - nb[2]] = acc[2]
            acc[1], acc[2], acc[3] = S2, N2, nb
        r_, c_ = box[0] - acc[3][0], box[2] - acc[3][2]; h_, w_ = V.shape
        acc[1][r_:r_ + h_, c_:c_ + w_] += np.where(V[..., None], patch, 0); acc[2][r_:r_ + h_, c_:c_ + w_] += V
    close(acc)
    st.stop(); cs.Views(C, F, sls, patches, ts, Ts, res=a.res).save(a.out, cal_stamp)
    print(f"night {a.night}: {len(C)} views kept (lag {lag * 1000:.0f} ms), {time.time() - t0:.0f} s -> {a.out}")


if __name__ == "__main__":
    main()
