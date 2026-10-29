#!/usr/bin/env python3
"""Tonight's frames in the baselines' frame ([KkQ]): the house's lidar map rebuilt from the baseline nights' scans as the
pose graph placed them (house_frame.py's corrections), and tonight's scans matched to it as they arrive
(localize.Localizer: each scan from this one and the earlier ones only, as the robot would live). The floor pictures,
built in the pose graph's frame, are then compared with frames placed in the same one.
Usage: tonight_frame.py <tonight.db> (--run <run dir> | --base <night1.db> <corrections_1.npz> --base <night2.db>
       <corrections_2.npz>) --out corrections_tonight.npz   (dimos venv)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))
import argparse, time
from pathlib import Path
import numpy as np
from dimos.memory.store.sqlite import SqliteStore
from houndkeeper.localize import Localizer, scans_from, structure, _voxel, level


def scans(db):
    st = SqliteStore(path=db); st.start(); s = scans_from(st); st.stop(); return s


def house_map(bases):
    """every baseline scan's structure, placed by its night's correction: the map tonight is matched to"""
    pts = []
    for db, corr in bases:
        z = np.load(corr); loc = Localizer(times=list(z["times"]), Ts=list(z["Ts"]))
        for t, p in scans(db):
            s = structure(level(p))
            if len(s) < 50: continue
            C = loc.correction(t); pts.append(s @ C[:2, :2].T + C[:2, 2])
    return _voxel(np.vstack(pts))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("db"); ap.add_argument("--base", nargs=2, action="append", default=None)
    ap.add_argument("--run", default=None, help="a run dir whose floor map (stitch_baselines.sh) wrote floor_frame.json: its nights as --base")
    ap.add_argument("--out", required=True); a = ap.parse_args(); t0 = time.time()
    if a.run:
        import json as _json; ff = _json.load(open(Path(a.run) / "floor_frame.json")); a.base = list(zip(ff["dbs"], ff["corrections"]))
    m = house_map(a.base)
    loc = Localizer(house=m).run([(t, level(p)) for t, p in scans(a.db)])
    np.savez(a.out, times=np.array(loc.times), Ts=np.array(loc.Ts), fits=np.array(loc.fits, float))   # fits: (rms m, kept), the health
    rms = np.array([f[0] for f in loc.fits[1:]]) if len(loc.fits) > 1 else np.zeros(1)
    print(f"tonight: {len(loc.times)} scans placed on the baselines' map ({len(m)} points): fit rms median "
          f"{np.median(rms) * 1000:.0f} mm; {time.time() - t0:.0f} s -> {a.out}")


if __name__ == "__main__":
    main()
