#!/usr/bin/env python3
"""Tonight's 'stands up off the floor' layer, from raw lidar returns (not DimOS's costmap, not the seeded map).
A 2 cm cell is tall if >= MIN_HITS lidar returns in it lie between LO and HI above the floor. Written at the
floor pictures' 1 cm grid so the detector can use it directly. A cell must also have a share of its returns up
(MIN_SHARE): the floor's own returns scatter (range sd 2 cm, [nAH] JaG) and over a night put 3+ above 4 cm in most cells.
--corrections: each scan moved into the house's frame first ([KkQ]; floor_mosaic2 --house writes them).
Usage: tall_layer.py <memory.db> <out.npz> [--corrections c.npz]"""
import argparse
import numpy as np
from dimos.memory.store.sqlite import SqliteStore
HX, HY, RES, HRES = 8.4, 4.9, 0.01, 0.02
LO, HI, MIN_HITS = 0.04, 0.60, 3        # 4 cm: above floor noise and flat litter; 60 cm: below table tops
MIN_SHARE = 0.3                         # of all the cell's returns below HI (est.; 27 Sep: count alone streaked the pictures)
ap = argparse.ArgumentParser(); ap.add_argument("db"); ap.add_argument("out"); ap.add_argument("--corrections", default=None)
a = ap.parse_args(); db, out = a.db, a.out
loc = None
if a.corrections:
    sys_ = __import__("sys"); from pathlib import Path; sys_.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
    from houndkeeper.localize import Localizer
    z = np.load(a.corrections); loc = Localizer(times=list(z["times"]), Ts=list(z["Ts"]))
hw, hh = int(2 * HX / HRES), int(2 * HY / HRES)
cnt = np.zeros((hh, hw), np.int32); alln = np.zeros((hh, hw), np.int32); zmax = np.zeros((hh, hw), np.float32)
st = SqliteStore(path=db); st.start()
for o in st.stream("lidar"):
    p = o.data.as_numpy(); P = np.asarray(p[0] if isinstance(p, tuple) else p)
    if loc is not None:                                 # the house's frame, heights from the scan's own floor
        from houndkeeper.localize import level
        C = loc.correction(o.ts); P = level(P); P = np.c_[P[:, :2] @ C[:2, :2].T + C[:2, 2], P[:, 2]]
    P = P[P[:, 2] < HI]
    j = ((P[:, 0] + HX) / HRES).astype(int); i = ((HY - P[:, 1]) / HRES).astype(int)
    ok = (j >= 0) & (j < hw) & (i >= 0) & (i < hh); np.add.at(alln, (i[ok], j[ok]), 1)
    up = P[:, 2] > LO; P, i, j, ok = P[up], i[up], j[up], ok[up]
    np.add.at(cnt, (i[ok], j[ok]), 1); np.maximum.at(zmax, (i[ok], j[ok]), P[ok, 2].astype(np.float32))
st.stop()
h = np.where((cnt >= MIN_HITS) & (cnt >= MIN_SHARE * alln), zmax, 0).astype(np.float16)
h1 = np.repeat(np.repeat(h, 2, 0), 2, 1)[: int(2 * HY / RES), : int(2 * HX / RES)]
np.savez_compressed(out, h=h1)
print(f"tall cells {(h > 0).sum() * HRES * HRES:.1f} m2")
