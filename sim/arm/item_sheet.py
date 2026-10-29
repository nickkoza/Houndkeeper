#!/usr/bin/env python3
"""Contact sheet of the item library ([QxY]): every item settled on a grey floor, one tile each, to check by eye that
each scanned model is one real object at a sensible size. Usage: item_sheet.py [out.png]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import sys
import numpy as np, mujoco, PIL.Image as I, PIL.ImageDraw as D
sys.path.insert(0, str(_HK / "sim" / "arm"))
import items
out = sys.argv[1] if len(sys.argv) > 1 else str(_HK / "data" / "eval" / "item_sheet.png")
names = [n for n in items.ITEMS if items.ITEMS[n][0] not in ("cloth", "rope")]; T = 200; cols = 10; tiles = []
for n in names:
    sp = mujoco.MjSpec(); g = sp.worldbody.add_geom(); g.type = mujoco.mjtGeom.mjGEOM_PLANE; g.size = [5, 5, .1]; g.rgba = [.75, .75, .75, 1]
    li = sp.worldbody.add_light(); li.pos = [0.3, -0.3, 2]; li.dir = [-0.15, 0.15, -1]
    items.add(sp, n, [0, 0, 0.3]); m = sp.compile(); d = mujoco.MjData(m)
    for _ in range(1500): mujoco.mj_step(m, d)
    P = items.item_points(m, d, m.body(n).id); r = mujoco.Renderer(m, T, T); c = mujoco.MjvCamera()
    c.lookat = P.mean(0); c.distance = 0.55; c.elevation = -50; c.azimuth = 60    # same distance for all: sizes compare
    r.update_scene(d, c); im = I.fromarray(r.render()); D.Draw(im).text((4, 4), n[:30], fill=(0, 0, 0)); tiles.append(im); r.close()
rows = (len(tiles) + cols - 1) // cols; sheet = I.new("RGB", (cols * T, rows * T), "white")
for k, t in enumerate(tiles): sheet.paste(t, ((k % cols) * T, (k // cols) * T))
sheet.save(out); print("saved", out, len(tiles), "items")
