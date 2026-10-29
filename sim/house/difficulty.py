#!/usr/bin/env python3
"""Difficulty tier per dropped item, from its own properties only (size and contrast with the floor under it).
Fixed before any detection results. easy: >= 100 cm2 and contrast >= 0.25; hard: < 15 cm2, string thinner
than 3 mm, or contrast < 0.10; medium: everything else. Contrast = RGB distance (0-1 scale) to the floor
material at the item's spot (carpet, tile, or the rug once it's down)."""
import make_house as mh

CARPET, TILE, RUG = (0.55, 0.50, 0.46), (0.78, 0.76, 0.72), (128 / 255, 42 / 255, 46 / 255)


def floor_rgb(x, y, rug):
    if rug and abs(x - rug["x"]) <= rug["w"] / 2 and abs(y - rug["y"]) <= rug["h"] / 2: return RUG
    cx, cy = int((x + mh.W * mh.CELL / 2) / mh.CELL), int((mh.H * mh.CELL / 2 - y) / mh.CELL)
    return CARPET if any(x0 <= cx <= x1 and y0 <= cy <= y1 for x0, y0, x1, y1 in mh.CARPET) else TILE   # this house's ([uCP])


def tier(it, rug):
    rgb = [float(v) for v in it["rgba"].split()[:3]]
    contrast = sum((a - b) ** 2 for a, b in zip(rgb, floor_rgb(it["x"], it["y"], rug))) ** 0.5
    if it["kind"] == "string":
        area = it.get("length_m", 0.3) * 100 * 2 * it.get("radius", 0.002) * 100
        if 2 * it.get("radius", 0.002) < 0.003 or contrast < 0.10: return "hard", contrast, area
        return "medium", contrast, area
    sx, sy = it["half"]; area = 4 * sx * sy * 1e4 if it.get("shape", "box") in ("box",) else 3.1416 * sx * sx * 1e4
    if area < 15 or contrast < 0.10: return "hard", contrast, area
    if area >= 100 and contrast >= 0.25: return "easy", contrast, area
    return "medium", contrast, area
