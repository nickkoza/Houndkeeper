#!/usr/bin/env python3
"""How often the detector flags moved furniture (chairs, stool) as clutter. Not a pick-up target, so any
'present' blob touching a chair's footprint tonight, or the spot it stood on last night, is a furniture flag.
Reads detect_nN.npz/json + truth 'movables' from a run dir. Usage: score_movables.py <run_dir>"""
import json, math, sys
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
RES, HX, HY = 0.01, 8.4, 4.9


def footprint(c, shape, pad=0.10):
    m = np.zeros(shape, bool); r = int((math.hypot(c["w"], c["d"]) / 2 + pad) / RES)
    r0, c0 = int((HY - c["y"]) / RES), int((c["x"] + HX) / RES)
    yy, xx = np.mgrid[r0 - r:r0 + r + 1, c0 - r:c0 + r + 1]
    X, Y = (xx - c0) * RES, (r0 - yy) * RES; ca, sa = math.cos(-c["yaw"]), math.sin(-c["yaw"])
    u, v = X * ca - Y * sa, X * sa + Y * ca
    inside = (abs(u) <= c["w"] / 2 + pad) & (abs(v) <= c["d"] / 2 + pad)
    ok = (yy >= 0) & (yy < shape[0]) & (xx >= 0) & (xx < shape[1]) & inside
    m[yy[ok], xx[ok]] = True; return m


run = Path(sys.argv[1]); tot = {"flags_on_chairs": 0, "flags_on_vacated": 0, "false_alarms": 0, "chair_nights_moved_a_lot": 0, "chair_nights_moved_a_lot_flagged": 0}
res = json.loads((run / "detect_results.json").read_text())
for n in sorted(int(k) for k in res):
    lab = np.load(run / f"detect_n{n}.npz")["lab"]; blobs = json.loads((run / f"detect_n{n}.json").read_text())["blobs"]
    pres = [b for b in blobs if b["kind"] == "present" and not b["matched"]]
    t = json.loads(next(run.glob(f"nights_s*_n{n}.truth.json")).read_text()); tp = json.loads(next(run.glob(f"nights_s*_n{n-1}.truth.json")).read_text())
    now = {c["id"]: footprint(c, lab.shape) for c in t.get("movables", [])}
    was = {c["id"]: footprint(c, lab.shape) for c in tp.get("movables", [])}
    on, vac = set(), set(); lot_flag = 0
    for c in t.get("movables", []):
        ids_now = set(np.unique(lab[now[c["id"]]]).tolist()) - {0}; ids_was = set(np.unique(lab[was[c["id"]] & ~now[c["id"]]]).tolist()) - {0}
        pid = {b["id"] for b in pres}; on |= ids_now & pid; vac |= (ids_was & pid) - on
        if c["mode"] == "a lot": tot["chair_nights_moved_a_lot"] += 1; lot_flag += bool((ids_now | ids_was) & pid)
    tot["flags_on_chairs"] += len(on); tot["flags_on_vacated"] += len(vac); tot["false_alarms"] += res[str(n)]["false_alarms"]; tot["chair_nights_moved_a_lot_flagged"] += lot_flag
    print(f"night {n}: false alarms {res[str(n)]['false_alarms']}, of which on a chair {len(on)}, on a chair's old spot {len(vac)}")
print(f"{run.name} total: {tot}")
