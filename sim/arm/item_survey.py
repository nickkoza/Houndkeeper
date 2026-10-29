#!/usr/bin/env python3
"""Library survey ([QxY], [CaM]): every item dropped in several random orientations on a bare floor; prints its size,
mass (est.) and how often it lands grippable (items.judge). Catches broken models and odd sizes before a bench uses them.
Usage: item_survey.py [drops per item]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import sys, time
import numpy as np, mujoco
sys.path.insert(0, str(_HK / "sim" / "arm"))
import items
N = int(sys.argv[1]) if len(sys.argv) > 1 else 6
names = [n for n in items.ITEMS if items.ITEMS[n][0] not in ("cloth", "rope")]
rng = np.random.default_rng(0)
for n in names:
    spec = mujoco.MjSpec(); spec.option.timestep = 0.002                       # one small model per item: fast drops
    g = spec.worldbody.add_geom(); g.type = mujoco.mjtGeom.mjGEOM_PLANE; g.size = [5, 5, 0.1]; g.friction = [1.0, 0.02, 0.001]
    items.add(spec, n, [0, 0, 0.3]); m = spec.compile(); d = mujoco.MjData(m)
    b = m.body(n).id; ja = m.jnt_qposadr[m.body_jntadr[b]]; mass = items.set_mass(m, n, rng); bins = 0; ws = []; why = set()
    for k in range(N):
        mujoco.mj_resetData(m, d); q = rng.normal(size=4); q /= np.linalg.norm(q)
        d.qpos[ja:ja + 7] = [0, 0, 0.3, *q]; mujoco.mj_forward(m, d)
        for _ in range(1500): mujoco.mj_step(m, d)
        e, y = items.judge(m, d, n); ws.append(items.true_width(items.item_points(m, d, b))[0]); bins += e == "bin"
        if y: why.add(y.split(":")[-1].strip())
    ext = np.sort(np.ptp(items.item_points(m, d, b), axis=0))[::-1]
    print(f"{n:28s} {ext[0] * 100:5.1f}x{ext[1] * 100:4.1f}x{ext[2] * 100:4.1f} cm {mass * 1000:5.0f} g  grip w {np.median(ws) * 100:4.1f} cm  "
          f"pickable {bins}/{N}  {items.ITEMS[n][3]:6s} {'; '.join(sorted(why))[:50]}", flush=True)
