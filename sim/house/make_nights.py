#!/usr/bin/env python3
"""Multi-night house timeline for floor change detection.

Each night gets its own scene: carpet + tile floor texture with permanent stains, one stain cleaned
partway through, a rug (down from the first night unless --rug-night says later), per-night clutter (toys, Lego-size bricks, tissues,
string) where some items carry over, and uncontrolled lighting. Ground truth per night is JSON.
Clutter is non-solid (depth cameras see it; the robot can't trip on it).

Usage: make_nights.py --seed 7 --nights 6   (run with the dimos venv python; needs numpy + Pillow)
Outputs: <dimos>/data/mujoco_sim/scene_nights_s<seed>_n<k>.xml, textures, runs/nights_s<seed>/*.json
"""
import argparse, colorsys, io, json, math, os, random, sys, zlib
import cabinets
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFilter
import make_house as mh
import movables as mv

TEX_DIR = mh.DIMOS_DATA / "scene_office1/textures"   # the only texture folder DimOS's sim loads
PX = 0.01                                             # texture metres per pixel (set to PHOTO_PX for photographed floors)
PHOTO_PX = 0.003          # photographed house floor: 3 mm a pixel (the head camera resolves ~1.5-3 mm at 1-2 m; finer
                          # costs over a GB of GPU texture across Filament's per-camera renderers and DimOS's classic ones)
HX, HY = mh.W * mh.CELL / 2, mh.H * mh.CELL / 2       # house half extents (m)
# the carpeted areas, the lamp and the pet's bowls are the house plan's (make_house: CARPET, LAMP, BOWLS; [uCP])
RUG_NIGHT, CLEAN_NIGHT = 1, 4                          # rug down from the first night (in the baseline:
                                                       # a floor change is its own test, --rug-night); stain_1 cleaned


def w2px(x, y):
    return (x + HX) / PX, (HY - y) / PX                 # row 0 = +y edge


_BASE = {}


def base_floor(seed):
    """The house's floor, generated once: tile tones, grout, carpet colour and grain. Reused every night."""
    if seed in _BASE: return _BASE[seed]
    wpx, hpx = int(2 * HX / PX), int(2 * HY / PX)
    nr = np.random.default_rng(seed * 7919)
    tile = np.zeros((hpx, wpx, 3), np.float32); tile[:] = (0.78, 0.76, 0.72)
    t = int(0.30 / PX)
    for r in range(0, hpx, t):
        for c in range(0, wpx, t):
            tile[r:r + t, c:c + t] *= nr.uniform(0.96, 1.04)
    tile[::t, :] = tile[1::t, :][: tile[::t, :].shape[0]] * 0.55
    tile[:, ::t] = tile[:, 1::t][:, : tile[:, ::t].shape[1]] * 0.55
    carpet = np.zeros((hpx, wpx), bool)
    for x0, y0, x1, y1 in mh.CARPET:
        (ax, ay), (bx, by) = mh.cell_to_world(x0, y0), mh.cell_to_world(x1, y1)
        c0, r0 = w2px(min(ax, bx) - mh.CELL / 2, max(ay, by) + mh.CELL / 2)
        c1, r1 = w2px(max(ax, bx) + mh.CELL / 2, min(ay, by) - mh.CELL / 2)
        carpet[int(max(r0, 0)):int(min(r1, hpx)), int(max(c0, 0)):int(min(c1, wpx))] = True
    grain = nr.normal(1, 0.06, (hpx, wpx)).astype(np.float32)
    _BASE[seed] = (tile, carpet, grain); return _BASE[seed]


def _photo_region(rng, row, hpx, wpx):
    """one photographed floor tiled over the whole house at its real size (arm-sim floors' convention: row i is +y)"""
    import floors
    col, nrm, rough, occ = floors._dihedral(rng, floors._photo_maps(row["id"]))
    per = max(8, int(round(float(row["size_m"]) / PX)))
    def lay(a):
        one = floors.cv2.resize(np.ascontiguousarray(a), (per, per), interpolation=floors.cv2.INTER_AREA)
        reps = (hpx // per + 1, wpx // per + 1) + (1,) * (a.ndim - 2)
        return np.tile(one, reps)[:hpx, :wpx]
    n = lay(nrm); n /= np.linalg.norm(n, axis=-1, keepdims=True) + 1e-6
    return lay(col), n, lay(rough), lay(occ)


PATTERNED_HOUSES = {71}   # houses whose hard floor is a complex, high-contrast tile (27 Sep: complex high-contrast
                          # designs are kept, not backed off): house 71 a patterned tile from the catalogue (had been a
                          # low-contrast one)
SLATE_HOUSES = {70, 95}   # and house 70, the main tuning house (had been wood): large diagonal slate-look porcelain
                          # tiles with inset squares (slate_tile.py); and the demo room (95)
LOW_PILE_HOUSES, LOW_PILE_CARPET, LOW_PILE_M = {70}, "Carpet016", 0.008   # house 70's carpet: low pile, at the lower
                          # end of what homes have (27 Sep): the flattest home carpet here (its normals tip 16
                          # deg on average; Carpet012, house 70's before, 33), its top 8 mm above the tile (est.)
HOUSE70_CARPET_SRGB = (0.64, 0.61, 0.57)   # house 70's carpet colour, a warm neutral (30 Sep): its hue is used, not its
                           # brightness (the photo's own sets that)
FLOOR_VARIETY = {82: ("WoodFloor039", "Carpet001"), 83: ("Tiles074", "Carpet004"), 84: ("WoodFloor043", "Fabric025"),
                 85: ("Tiles105", "Fabric027"), 86: ("Tiles012", "Fabric028"), 87: ("WoodFloor047", "Carpet014"),
                 88: ("Tiles142", "Carpet016"), 89: ("WoodFloor051", "Carpet001"), 90: ("Tiles099", "Carpet004"),
                 91: ("WoodFloor054", "Fabric025"), 92: ("Tiles075", "Fabric027"), 93: ("Tiles021", "Carpet012")}
# ([aj8]) the floor-variety tuning houses: (hard floor, carpet), training floors the first six houses never had
STAIN_FILL = {95: 60}     # a stain's strength (of 255; the tuning houses' 150): the demo room's are faint (its
                          # two stains lighter, not so dark)
DEMO_FLOORS = {95: ("Tiles142", "Carpet016")}   # subsystem-demos [qEP]: its carpet; its tile is the slate-look one
                                                # with inset corners (SLATE_HOUSES; 2 Oct 2026)
HOUSE_CARPETS = {"Carpet012", "Carpet014", "Carpet016", "Fabric028"}   # the training carpets a home is laid with: cut pile,
                           # berber, plush. The commercial pattern (Carpet004) and the woven and sisal photos stay in the
                           # arm sim's patches and the rugs: a woven grey carpet stood in for a beige cut
                           # pile and read flat (pile contrast 4 where a cut pile reads 10, at 40 cm, 26 Sep)
def photo_base(seed):
    """The house's photographed floor ([QMt], [N4z]'s floors): a carpet in the carpeted areas and a hard floor (wood or
    tile) elsewhere, drawn per house from the training floors (the held-out ones are for a held-out house). Returned in
    the PNG's layout (row 0 = +y edge): albedo 0-1, encoded normal and ORM maps, and the carpet mask."""
    if ("photo", seed) in _BASE: return _BASE[("photo", seed)]
    import floors
    wpx, hpx = int(2 * HX / PX), int(2 * HY / PX)
    nr = np.random.default_rng(seed * 7919 + 11)
    split = os.environ.get("HK_FLOORS", "train")               # "heldout": a house for judging only, on floors never drawn
    carpets = [c for c in floors.photos(split, ["carpet"]) if c["id"] in HOUSE_CARPETS or split == "heldout"]   # for tuning
    hards = floors.photos(split, ["wood", "tile"])
    c_row, h_row = carpets[nr.integers(len(carpets))], hards[nr.integers(len(hards))]
    if (seed in FLOOR_VARIETY or seed in DEMO_FLOORS) and split == "train":   # [aj8]: named floors, every carpet kind (woven,
        allf = {r["id"]: r for r in floors.photos("train")}                    # commercial too); a demo room's, chosen to be seen
        h_row, c_row = (allf[f] for f in (FLOOR_VARIETY.get(seed) or DEMO_FLOORS[seed]))
    if seed in PATTERNED_HOUSES:                               # carpet and a complex, high-contrast tile (drawn after the
        pats = floors.photos(split, ["tile_pattern"]) or floors.photos("train", ["tile_pattern"])   # (drawn after the others,
                                                               # so every other house keeps its floors)
        h_row = pats[nr.integers(len(pats))]
    carpet = np.zeros((hpx, wpx), bool)                        # in the PNG's layout, as base_floor's
    for x0, y0, x1, y1 in mh.CARPET:
        (ax, ay), (bx, by) = mh.cell_to_world(x0, y0), mh.cell_to_world(x1, y1)
        c0, r0 = w2px(min(ax, bx) - mh.CELL / 2, max(ay, by) + mh.CELL / 2)
        c1, r1 = w2px(max(ax, bx) + mh.CELL / 2, min(ay, by) - mh.CELL / 2)
        carpet[int(max(r0, 0)):int(min(r1, hpx)), int(max(c0, 0)):int(min(c1, wpx))] = True
    if seed in LOW_PILE_HOUSES:                                # house 70's carpet: low pile (after every draw: other houses keep
        c_row = next(c for c in carpets if c["id"] == LOW_PILE_CARPET)   # theirs; before the maps are made, so it's the one
    maps = []                                                  # laid: from 27 to 30 Sep it came after, and house 70 kept its
    for row in (h_row, c_row):                                 # first draw, a dark cut pile, under this one's name ([jmf])
        col, n, rough, occ = _photo_region(nr, row, hpx, wpx)
        maps.append((col[::-1], n[::-1], rough[::-1], occ[::-1]))  # to the PNG's layout (rows toward -y); the normals keep world axes
    if seed in LOW_PILE_HOUSES:                                # and in house 70's carpet colour: its hue, the photo's brightness
        cc = maps[1][0]; mean = cc.reshape(-1, 3).mean(0, dtype=np.float64)   # (float64: a float32 sum over 18M pixels drifts)
        target = np.array(HOUSE70_CARPET_SRGB) * (mean.mean() / np.mean(HOUSE70_CARPET_SRGB))
        maps[1] = (np.clip(cc * (target / mean), 0, 1).astype(np.float32), *maps[1][1:])
    if seed in SLATE_HOUSES:                                   # large diagonal slate-look tiles
        import slate_tile                                      # (slate_tile.py), in the PNG's layout already
        maps[0] = slate_tile.slate_tile(np.random.default_rng(seed * 7919 + 23), hpx, wpx, PX, HX, HY)
        h_row = {"id": "slate_tile"}
    g_ = mh.base_grid(); open_ = np.zeros_like(carpet)        # open floor (no wall, no furniture, no carpet): where people step on
    for yy in range(mh.H):
        for xx in range(mh.W):
            if not g_[yy][xx]:
                (ox, oy) = mh.cell_to_world(xx, yy); oc, orr = w2px(ox - mh.CELL / 2, oy + mh.CELL / 2)
                open_[int(max(orr, 0)):int(min(orr + mh.CELL / PX, hpx)), int(max(oc, 0)):int(min(oc + mh.CELL / PX, wpx))] = True
    open_ &= ~carpet
    wr = wear_paths(seed, carpet, open_=open_)                 # [bP1] walked flat: greyer and darker, less textured, flatter
    if wr.max() > 0:
        from scipy import ndimage as _ndi
        cc, cn, cr, co = maps[1]; w3 = wr[..., None]
        local = np.stack([_ndi.gaussian_filter(cc[..., i], 0.03 / PX) for i in range(3)], -1)   # the pile's own colour, unflecked
        grey = local.mean(-1, keepdims=True) * np.array([0.98, 0.97, 0.95])       # matted and ground-in: greyer, darker,
        cc = np.clip(cc * (1 - 0.75 * w3) + (0.5 * local + 0.5 * grey) * 0.80 * 0.75 * w3, 0, 1)   # its flecks smoothed away
        flat = np.zeros_like(cn); flat[..., 2] = 1.0
        cn = cn * (1 - 0.55 * w3) + flat * 0.55 * w3; cn = cn / np.maximum(np.linalg.norm(cn, axis=-1, keepdims=True), 1e-6)
        maps[1] = (cc.astype(np.float32), cn.astype(np.float32), np.clip(cr - 0.08 * wr, 0, 1).astype(np.float32), co)
    m_ = carpet[..., None]
    col = np.where(m_, maps[1][0], maps[0][0]); n = np.where(m_, maps[1][1], maps[0][1])
    rough = np.where(carpet, maps[1][2], maps[0][2]); occ = np.where(carpet, maps[1][3], maps[0][3])
    nrgb, orm = floors._encode(n, rough, occ)
    _BASE[("photo", seed)] = (col.astype(np.float32), nrgb, orm, carpet, f"{h_row['id']}+{c_row['id']}")
    return _BASE[("photo", seed)]


def wear_paths(seed, carpet, px=None, open_=None):
    """[bP1] where the carpet is walked flat: 0 (untouched) to 1 (a well-worn lane), on the carpet only, in the PNG's
    layout. For each carpeted region, a few smooth lanes: across it from one edge to another, winding gently through a
    point or two inside, 35-60 cm wide with soft shoulders. The house's own (seeded by it, not by the night)"""
    from scipy import ndimage as _ndi
    px = px or PX; H_, W_ = carpet.shape; rng = np.random.default_rng(seed * 131 + 7)
    lab, nreg = _ndi.label(carpet); wear = np.zeros(carpet.shape, np.float32)
    for k in range(1, nreg + 1):
        rr, cc = np.nonzero(lab == k); r0, r1, c0, c1 = rr.min(), rr.max(), cc.min(), cc.max()
        h_, w_ = (r1 - r0) * px, (c1 - c0) * px
        if min(h_, w_) < 0.8: continue
        lanes = np.zeros(carpet.shape, bool)
        reg = lab == k; ring = _ndi.binary_dilation(reg, iterations=3) & ~reg
        gate = np.argwhere(_ndi.binary_dilation(ring & open_, iterations=3) & reg) if open_ is not None else np.zeros((0, 2))
        for _ in range(int(rng.integers(2, 4))):                        # two or three lanes a region
            def edge():
                s_ = rng.integers(4); t = rng.uniform(0.15, 0.85)
                return [(r0, c0 + t * (c1 - c0)), (r1, c0 + t * (c1 - c0)), (r0 + t * (r1 - r0), c0), (r0 + t * (r1 - r0), c1)][s_]
            a = gate[rng.integers(len(gate))].astype(float) if len(gate) else np.array(edge())   # stepping on where it meets open floor
            b = np.array(edge())
            for _try in range(6):                                        # and off it across the carpet, not alongside
                if np.hypot(*(a - b)) > 0.5 * max(r1 - r0, c1 - c0): break
                b = np.array(edge())
            if np.hypot(*(a - b)) * px < 0.8: continue
            mids = [a + (b - a) * f + rng.normal(0, 0.12, 2) * np.array([r1 - r0, c1 - c0]) for f in (0.35, 0.7)]
            P = np.array([a] + mids + [b]); t = np.linspace(0, 1, len(P)); ts = np.linspace(0, 1, int(np.hypot(*(b - a)) / 2) + 20)
            curve = np.stack([np.interp(ts, t, P[:, 0]), np.interp(ts, t, P[:, 1])], 1)
            curve = _ndi.gaussian_filter1d(curve, max(1, len(ts) // 12), axis=0, mode="nearest")   # smooth, not polygonal
            ri, ci = np.clip(curve[:, 0].round().astype(int), 0, H_ - 1), np.clip(curve[:, 1].round().astype(int), 0, W_ - 1)
            lane = np.zeros(carpet.shape, bool); lane[ri, ci] = True
            half = rng.uniform(0.17, 0.30) / px
            d = _ndi.distance_transform_edt(~lane)
            wear = np.maximum(wear, np.clip(1 - (d / half) ** 2, 0, 1).astype(np.float32) * rng.uniform(0.75, 1.0))
    return np.where(carpet, _ndi.gaussian_filter(wear, 0.05 / px), 0).astype(np.float32)


def photo_floor_texture(rng, stains, rug, seed=7):
    """floor_texture for a photographed floor: tonight's colour, normal and ORM images (PIL). The same drift (up to one
    pixel, 3 mm; brightness within 1%), a photographed rug from the rug floors when it's down, the same stains"""
    import floors
    col, nrgb, orm, carpet, _ = photo_base(seed)
    dy, dx = rng.randint(-1, 1), rng.randint(-1, 1)
    col = np.roll(col, (dy, dx), (0, 1)) * rng.uniform(0.99, 1.01); nrgb = np.roll(nrgb, (dy, dx), (0, 1)); orm = np.roll(orm, (dy, dx), (0, 1))
    if rug:                                                   # the same rug every night, laid at tonight's pose (turned)
        hp, wp = int(round(rug["h"] / PX)), int(round(rug["w"] / PX))
        rugs = floors.photos(os.environ.get("HK_FLOORS", "train"), ["rug"]) or floors.photos("train", ["rug"]); rrow = rugs[np.random.default_rng(seed * 31 + 5).integers(len(rugs))]
        rc, rn, rr, ro = _photo_region(np.random.default_rng(seed * 31 + 6), rrow, hp, wp)
        rn_rgb, r_orm = floors._encode(rn[::-1], rr[::-1], ro[::-1])
        cx, cy = w2px(rug["x"], rug["y"]); M = floors.cv2.getRotationMatrix2D((wp / 2, hp / 2), rug.get("yaw_deg", 0.0), 1.0)
        M[:, 2] += (cx - wp / 2, cy - hp / 2)                  # image rows run toward -y: a positive yaw turns it anticlockwise
        size = (col.shape[1], col.shape[0]); warp = lambda im, interp: floors.cv2.warpAffine(np.ascontiguousarray(im), M, size, flags=interp)
        mask = warp(np.ones((hp, wp), np.float32), floors.cv2.INTER_LINEAR)[..., None]
        col = col * (1 - mask) + warp(rc[::-1].astype(np.float32), floors.cv2.INTER_LINEAR) * mask
        m8 = mask > 0.5
        nrgb = np.where(m8, warp(rn_rgb, floors.cv2.INTER_NEAREST), nrgb); orm = np.where(m8, warp(r_orm, floors.cv2.INTER_NEAREST), orm)
    pil = Image.fromarray((np.clip(col, 0, 1) * 255).astype(np.uint8))
    layer = Image.new("L", pil.size, 0); dl = ImageDraw.Draw(layer)
    for s_ in stains:                                         # each stain keeps its own fixed shape (as floor_texture)
        if s_.get("r", 0) <= 0: continue
        sr = np.random.default_rng(zlib.crc32(s_["id"].encode()))
        cx, cy = w2px(s_["x"], s_["y"]); r = s_["r"] / PX
        for _ in range(6):
            ox, oy = sr.normal(0, r * 0.35, 2); rr_ = r * sr.uniform(0.5, 0.9)
            dl.ellipse([cx + ox - rr_, cy + oy - rr_ * 0.8, cx + ox + rr_, cy + oy + rr_ * 0.8], fill=STAIN_FILL.get(seed, 150))
    layer = layer.filter(ImageFilter.GaussianBlur(4 * 0.01 / PX))
    return Image.composite(Image.new("RGB", pil.size, (70, 50, 35)), pil, layer), Image.fromarray(nrgb), Image.fromarray(orm)


def floor_texture(rng, stains, rug, seed=7):
    """Tonight's floor = the fixed base floor + tonight's deliberate changes (stains, rug) + a small drift:
    carpet grain shifted by up to 1 px (1 cm) and brightness within about 1%. Much smaller than any object."""
    tile, carpet, grain = base_floor(seed)
    g = np.roll(grain, (rng.randint(-1, 1), rng.randint(-1, 1)), (0, 1)) * rng.uniform(0.99, 1.01)
    img = tile.copy(); img[carpet] = np.array((0.55, 0.50, 0.46), np.float32) * g[carpet][:, None]
    pil = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
    if rug:
        d = ImageDraw.Draw(pil)
        c0, r0 = w2px(rug["x"] - rug["w"] / 2, rug["y"] + rug["h"] / 2)
        c1, r1 = w2px(rug["x"] + rug["w"] / 2, rug["y"] - rug["h"] / 2)
        d.rectangle([c0, r0, c1, r1], fill=(128, 42, 46))
        for k in range(1, 8):
            yk = r0 + (r1 - r0) * k / 8
            d.line([c0 + 8, yk, c1 - 8, yk], fill=(205, 170, 95), width=5)
    layer = Image.new("L", pil.size, 0); dl = ImageDraw.Draw(layer)
    for s_ in stains:                                         # each stain keeps its own fixed shape
        if s_.get("r", 0) <= 0: continue
        sr = np.random.default_rng(zlib.crc32(s_["id"].encode()))
        cx, cy = w2px(s_["x"], s_["y"]); r = s_["r"] / PX
        for _ in range(6):
            ox, oy = sr.normal(0, r * 0.35, 2); rr = r * sr.uniform(0.5, 0.9)
            dl.ellipse([cx + ox - rr, cy + oy - rr * 0.8, cx + ox + rr, cy + oy + rr * 0.8], fill=150)
    layer = layer.filter(ImageFilter.GaussianBlur(4))
    return Image.composite(Image.new("RGB", pil.size, (70, 50, 35)), pil, layer)


def lighting(rng):
    a = rng.uniform(0, 2 * math.pi)
    return {"sun": round(rng.uniform(0.25, 0.85), 2), "ambient": round(rng.uniform(0.12, 0.45), 2),
            "dir": [round(0.4 * math.cos(a), 2), round(0.4 * math.sin(a), 2), -1.0],
            "tint": rng.choice(["warm", "neutral", "cool"]), "lamp_on": rng.random() < 0.5}


OUTDOOR_DAY = 8.0          # the view out by day: an emission the camera blows out past the blinds, as a camera's
                           # windows do (est., set by eye)
TINT = {"warm": (1.0, 0.88, 0.72), "neutral": (1, 1, 1), "cool": (0.85, 0.92, 1.0), "bulb": (1.0, 0.78, 0.55)}
WINDOW_Z = (0.9, 2.1)      # m: sill and head; the walls stay whole below the sill, where the robot's lidar looks
WINDOW_W, WINDOW_EVERY = 3, 11   # cells: a ~1 m window every ~3.9 m of outside wall
BOUNCE, BOUNCE_BULB = 0.25, 1.2   # indirect light (Filament doesn't trace it): a share of each light's own, from a source as
                           # big as a patch of ceiling and walls (est.; bounce is simulated on purpose)
WINDOW_BULB = 0.45         # a window's sky light comes from the whole opening (m)
LIGHT_COPIES = 1           # each indoor light as one light. It was 4 copies spread over the lamp's size, a penumbra for the
                           # classic renderer (it ignores bulbradius, 3.10); the robot's cameras are Filament now, and the
                           # copies filled the viewer's 8 lights, so lights dropped in and out as it moved (seen 26 Sep)


def regime(seed, night, only=None):
    """day or night for this night's lighting, from its own stream (so every seed's items and the PoC's draws stay as
    they were): nights are warm lamp and ceiling light, days are sunlight through the windows with
    sharp shadows (both randomized). only="night" (or "day") keeps every night to one: the tidy
    works at night, and a floor lit by the window compared with one lit by a lamp differs everywhere (26 Sep). The
    ceiling light is a fixture: where it hangs is the house's, not the night's (it had moved up to 1.2 m a night)"""
    r = random.Random(seed * 104729 + night * 7 + 3)
    day = r.random() < 0.5
    if only == "lit":                                       # the testing light (27 Sep: lights on to start with, a lever
        h = random.Random(seed * 104729 + 1)                # that is easy to improve in real life):
        return {"regime": "night", "sun": 0.0, "ambient": 0.16, "tint": "bulb", "ceiling": 0.9, "lamp_on": True,   # every night
                "ceiling_at": [round(h.uniform(-0.3, 0.3), 2), round(h.uniform(-0.3, 0.3), 2)], "lit": True}  # the same, all on
    if only is not None: day = only == "day"
    if day:                                                 # day: soft window light (sky, blinds, bounce); a third of days the
        return {"regime": "day", "window": round(r.uniform(0.5, 1.0), 2), "ambient": round(r.uniform(0.25, 0.45), 2),
                "tint": r.choice(["neutral", "cool"]), "lamp_on": False, "sun_in": r.random() < 0.35,   # sun shines in
                "sun": round(r.uniform(0.7, 1.0), 2), "sun_elev": round(r.uniform(25, 55), 1), "sun_turn": round(r.uniform(-40, 40), 1),
                "sun_window": r.randrange(2)}
    h = random.Random(seed * 104729 + 1)                     # the house's own: where its ceiling light hangs
    return {"regime": "night", "sun": 0.0, "ambient": round(r.uniform(0.06, 0.16), 2), "tint": "bulb",
            "ceiling": round(r.uniform(0.5, 0.9), 2), "lamp_on": r.random() < 0.6, "ceiling_at": [round(h.uniform(-0.3, 0.3), 2), round(h.uniform(-0.3, 0.3), 2)]}


def day_lights_xml(light, near, windows, c3):
    """daylight indoors: the two windows nearest `near` each glow into the room (a light just inside, aimed in and down,
    as copies across the window's width: soft); on a sun-in day the sun shines through one of them (sharp) and its patch
    on the floor lights the room from below (a dim warm fill: the classic renderer has no bounce light)"""
    near = np.asarray(near, float); win = sorted(windows, key=lambda w: np.hypot(w[0] - near[0], w[1] - near[1]))[:2]
    out = []; k = LIGHT_COPIES
    for j, (wx, wy, nx, ny) in enumerate(win):
        side = np.array([-ny, nx])
        for i in range(k):
            off = (i - (k - 1) / 2) * 0.3 / max(k - 1, 1) if k > 1 else 0.0
            px, py = wx + nx * 0.3 + side[0] * off, wy + ny * 0.3 + side[1] * off
            out.append(f'    <light name="window{j}_{i}" pos="{px:.3f} {py:.3f} 1.60" dir="{nx:.3f} {ny:.3f} -0.6" cutoff="70" exponent="2" '
                       f'diffuse="{c3(light["window"] * 1.4 / k)}" specular="0 0 0" attenuation="1 0.15 0.05" bulbradius="{WINDOW_BULB}"/>')
        bx, by = wx + nx * 1.5, wy + ny * 1.5                  # its light back off the floor and walls inside it
        out.append(f'    <light name="window{j}_bounce" pos="{bx:.3f} {by:.3f} {mh.WALL_H - 0.3:.2f}" dir="0 0 -1" cutoff="88" exponent="0" '
                   f'diffuse="{c3(light["window"] * 1.4 * BOUNCE)}" specular="0 0 0" attenuation="1 0.02 0.01" bulbradius="{BOUNCE_BULB}"/>')
    if light["sun_in"] and win:
        wx, wy, nx, ny = win[light["sun_window"] % len(win)]
        a = math.atan2(ny, nx) + math.radians(light["sun_turn"]); el = math.radians(light["sun_elev"])
        d = np.array([math.cos(a) * math.cos(el), math.sin(a) * math.cos(el), -math.sin(el)])   # travelling into the house
        out.append(f'    <light name="sun" directional="true" pos="0 0 6" dir="{d[0]:.3f} {d[1]:.3f} {d[2]:.3f}" diffuse="{c3(light["sun"])}"/>')
        t = 1.5 / -d[2]; px, py = wx + d[0] * t, wy + d[1] * t              # the patch's middle, from the window's middle
        out.append(f'    <light name="sun_bounce" pos="{px:.3f} {py:.3f} 0.05" dir="0 0 1" cutoff="85" exponent="1" '   # shadowing: a
                   f'diffuse="{c3(light["sun"] * 0.25)}" specular="0 0 0" attenuation="1 0.3 0.1" bulbradius="{BOUNCE_BULB}"/>')   # shadowless light
                                                             # beside shadowed ones misbehaved in mujofil (30 Sep)
    return "\n".join(out) + "\n"


LIT_CEILING = 1.54         # each lit-regime fixture's strength (per unit of `ceiling`), set by measuring the robot's cameras
                           # over a 48-spot grid of house 70 (28 Sep): a standing look at the carpet runs at 4.9x gain (median,
                           # 15% at the 16x ceiling), a head view 2.4x, both exposed (brightness ~100); a room of ~70 lux, between
                           # a typical lit living room (36-50 lux, the 75th percentile of a DOE field study of TV rooms) and a well-lit
                           # one (70-108, its 87.5th): a lit room. A dark carpet meters up to mid-grey, so looking down
                           # at it asks for ~2 stops more gain than a mixed view


def lights_xml(light, near, windows=()):
    """the scene's lights for a lighting dict: day = the sun, entering through the windows; night = a ceiling light near
    `near` (the part of the house being swept) and maybe the lamp, each one light (LIGHT_COPIES)"""
    tr, tg, tb = TINT[light["tint"]]; c3 = lambda k: " ".join(f"{k * c:.3f}" for c in (tr, tg, tb))
    if "regime" not in light:                                # the PoC's nights (before [CaM]'s indoor house): an open-topped sun
        return f'    <light name="sun" directional="true" pos="0 0 6" dir="{" ".join(map(str, light["dir"]))}" diffuse="{c3(light["sun"])}"/>\n'
    if light["regime"] == "day": return day_lights_xml(light, near, windows, c3)
    out = []; n = LIGHT_COPIES
    def spread(name, x, y, z, radius, k, att):
        for i in range(n):
            a, rr = 2 * math.pi * i / n, radius if n > 1 else 0.0
            out.append(f'    <light name="{name}_{i}" pos="{x + rr * math.cos(a):.3f} {y + rr * math.sin(a):.3f} {z:.2f}" dir="0 0 -1" '
                       f'cutoff="85" exponent="1" diffuse="{c3(k / n)}" specular="0 0 0" attenuation="{att}" bulbradius="{radius:.3f}"/>')
        bounce(name, x, y, k)
    def bounce(name, x, y, k):                               # the light it throws back off the floor, walls and ceiling:
        out.append(f'    <light name="{name}_bounce" pos="{x:.3f} {y:.3f} {mh.WALL_H - 0.3:.2f}" dir="0 0 -1" cutoff="88" exponent="0" '   # faint, from
                   f'diffuse="{c3(k * BOUNCE)}" specular="0 0 0" attenuation="1 0.02 0.01" bulbradius="{BOUNCE_BULB}"/>')   # everywhere
    if light.get("lit"):                                     # the lights on: a fixture over each part of the open plan
        for i, (x0, y0, x1, y1, _) in enumerate(mh.REGIONS):  # (play area, living, ...), as a real open-plan home lights it; one
            cx, cy = mh.cell_to_world((x0 + x1) / 2, (y0 + y1) / 2)   # light for 165 m2 left the camera at full gain and still
            spread(f"ceiling{i}", cx, cy, mh.WALL_H - 0.05, 0.12, light["ceiling"] * LIT_CEILING, "1 0.05 0.05")   # dark (28 Sep)
    else:
        cx, cy = near[0] + light["ceiling_at"][0] * 4, near[1] + light["ceiling_at"][1] * 4
        spread("ceiling", cx, cy, mh.WALL_H - 0.05, 0.12, light["ceiling"] * 2.2, "1 0.05 0.05")     # a 25 cm diffuser
    plan = mh.DEMO_PLANS.get(mh.LAYOUT_SEED) or {}
    lamp_z = 1.42 if any(f[5] == "reading_lamp" for f in mh.FURNITURE) else 1.6   # under a reading lamp's shade, where drawn
    if light["lamp_on"]: spread("lamp", mh.LAMP[0], mh.LAMP[1], lamp_z, 0.05, 0.9, "1 0.2 0.05")                # a lampshade
    if plan.get("pendant"):                                  # a pendant over the dining table (the demo room's)
        px, py = pendant_at(); spread("pendant", px, py, PENDANT_Z - 0.06, 0.06, 0.7, "1 0.25 0.08")
    return "\n".join(out) + "\n"


SURFACES = {   # photographed CC0 textures (tools/fetch_textures.sh -> models/textures); repeats per metre (est. real scale)
    "wall": (["PaintedPlaster017", "Plaster001"], 1.0), "wood": (["Wood092", "Wood094", "Wood095"], 2.0),
    "fabric": (["Fabric061", "Fabric081C"], 2.5), "stone": (["Marble012"], 1.0), "leather": (["Leather011", "Leather037"], 2.0)}
PAINTS = {   # the walls' paint: the colours most rooms are (sRGB, est.; the plaster photo gives the surface, not the colour:
    "white": (0.93, 0.92, 0.89), "warm white": (0.94, 0.90, 0.83), "beige": (0.87, 0.80, 0.69),   # PaintedPlaster017's
    "tan": (0.80, 0.69, 0.53), "greige": (0.78, 0.75, 0.69), "light grey": (0.82, 0.82, 0.80)}    # own blue-grey read as
                                                                    # a dark slate room, 26 Sep)
TEXTURES = Path(__file__).resolve().parents[2] / "models" / "textures"


WOOD_PNG = [None]   # the house's wood as surfaces_xml copied it: the cabinets' finish starts from it ([WTj])
MESH_REP = {"wood": 2.0, "stone": 1.0, "upholstery": 2.5}   # repeats a metre on the furniture meshes (SURFACES' own; the
                                                            # upholstery at the fabric's, leather's 2.0 near enough)


def surfaces_xml(seed):
    """the house's surface materials: one wall plaster, one wood, one sofa fabric and a worktop stone, drawn per house
    from `seed` ([CaM]: real surfaces, so the head camera tells floor from wall and furniture as it will in a real
    house). Copies the chosen images next to the floor textures, where the scene looks for them (as PNG)."""
    r = random.Random(seed * 7907 + 11); out = []
    for mat, (ids, rep_m) in SURFACES.items():
        tid = r.choice(ids); src = TEXTURES / f"{tid}_Color.jpg"
        if not src.exists(): raise SystemExit(f"{src} missing: run tools/fetch_textures.sh")
        rgba = "1 1 1 1"; dst = TEX_DIR / f"{tid}.png"      # MuJoCo 3.10 reads PNG textures, not JPEG
        if mat == "wall":                                   # the plaster's relief in grey, painted
            dst = TEX_DIR / f"{tid}_grey.png"
            if not dst.exists():
                g = np.asarray(Image.open(src).convert("L"), float); g = np.clip(g * (0.92 * 255 / g.mean()), 0, 255)
                Image.fromarray(g.astype(np.uint8)).convert("RGB").save(dst)
            rgba = " ".join(f"{c:.3f}" for c in PAINTS[r.choice(sorted(PAINTS))]) + " 1"
        elif not dst.exists(): Image.open(src).convert("RGB").save(dst)
        if mat == "wood": WOOD_PNG[0] = dst
        out.append(f'    <texture type="2d" name="tx_{mat}" file="{dst.name}"/>\n'
                   f'    <material name="{mat}" texture="tx_{mat}" texuniform="true" texrepeat="{rep_m} {rep_m}" rgba="{rgba}"/>\n'
                   f'    <material name="{mat}_m" texture="tx_{mat}" texuniform="false" texrepeat="1 1" rgba="{rgba}"'
                   f'{' specular="0.2" shininess="0.25"' if mat == "wood" else ""}/>')   # meshes; wood in a satin finish (MuJoCo's
                                                               # default read as gloss: a table under its light went white)
    up = r.choice(["fabric", "leather"])                    # the sofa: fabric or leather, per house
    out.append(f'    <material name="upholstery" texture="tx_{up}" texuniform="true" texrepeat="{SURFACES[up][1]} {SURFACES[up][1]}" '
               f'rgba="1 1 1 1"{' specular="0.4" shininess="0.6"' if up == "leather" else ""}/>')
    out.append(f'    <material name="upholstery_m" texture="tx_{up}" texuniform="false" texrepeat="1 1" '
               f'rgba="1 1 1 1"{' specular="0.4" shininess="0.6"' if up == "leather" else ""}/>')
    pr = r.choice([(0.85, 0.80, 0.70), (0.55, 0.62, 0.70), (0.72, 0.45, 0.35), (0.90, 0.88, 0.84)])   # throw pillows
    out.append(f'    <material name="pillow" texture="tx_fabric" texuniform="true" texrepeat="3 3" rgba="{pr[0]} {pr[1]} {pr[2]} 1"/>')
    out.append(cabinets.material_xml(seed, WOOD_PNG[0], TEX_DIR))       # the cabinets' finish, its own draw ([WTj])
    out.append("\n".join(cabinets.ASSETS))                              # the furniture's meshes (built before this)
    return "\n".join(out)


def furniture_xml():
    """the indoor house's furniture as real shapes where it has legs or overhangs:
    tables on legs, a sofa on short legs with a back and arms, a console on legs, an island
    whose worktop overhangs the dining side, a counter with a toe-kick. Same footprints as make_house.FURNITURE (the
    sweep route and item placement still keep off the whole footprint). Rows run down the grid, so -y faces the room."""
    out = []; cabinets.reset()
    def box(name, x0, y0, x1, y1, z0, z1, mat="wood", solid=True, euler=None):
        if mat in MESH_REP and not euler:                                # textured on every face: a mesh with its own
            return cabinets.piece(name, x0, y0, z0, x1, y1, z1, f"{mat}_m", solid, MESH_REP[mat])   # coordinates ([jmf])
        out.append(f'    <geom name="{name}" type="box" pos="{(x0 + x1) / 2:.3f} {(y0 + y1) / 2:.3f} {(z0 + z1) / 2:.3f}" '
                   f'size="{(x1 - x0) / 2:.3f} {(y1 - y0) / 2:.3f} {(z1 - z0) / 2:.3f}" material="{mat}" '
                   f'contype="{int(solid)}" conaffinity="0"{f' euler="{euler}"' if euler else ""}/>')
    def legs(n, x0, y0, x1, y1, h, t=0.04, inset=0.05):
        for i, (lx, ly) in enumerate(((x0 + inset, y0 + inset), (x1 - inset - t, y0 + inset), (x0 + inset, y1 - inset - t), (x1 - inset - t, y1 - inset - t))):
            box(f"{n}_leg{i}", lx, ly, lx + t, ly + t, 0, h)
    for c0, r0, c1, r1, h, n in mh.FURNITURE:
        (ax, ay), (bx, by) = mh.cell_to_world(c0, r0), mh.cell_to_world(c1, r1); e = mh.CELL / 2
        x0, x1, y0, y1 = min(ax, bx) - e, max(ax, bx) + e, min(ay, by) - e, max(ay, by) + e
        q = mh.FACING.get(n, 0) % 4                                     # turned to face its room ([A7G]): built facing -y
        if q:                                                           # on its footprint turned back, then turned about
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2                       # its centre as a body
            if q % 2: hx, hy = (y1 - y0) / 2, (x1 - x0) / 2; x0, x1, y0, y1 = cx - hx, cx + hx, cy - hy, cy + hy
            k_out, k_geo = len(out), len(cabinets.GEOMS)
        if n in ("coffee_table", "dining_table"):                       # a top on four legs, open underneath
            legs(n, x0, y0, x1, y1, h - 0.03); box(f"{n}_top", x0, y0, x1, y1, h - 0.03, h)
        elif n == "sofa":                                                # on 10 cm legs; back on +y; arms at the ends;
            legs(n, x0, y0, x1, y1, 0.10, t=0.05); box(f"{n}_seat", x0, y0, x1, y1, 0.10, 0.30, "upholstery")   # a base,
            box(f"{n}_back", x0, y1 - 0.20, x1, y1, 0.30, h, "upholstery")  # three seat cushions and three back cushions
            box(f"{n}_arm0", x0, y0, x0 + 0.15, y1, 0.30, 0.62, "upholstery"); box(f"{n}_arm1", x1 - 0.15, y0, x1, y1, 0.30, 0.62, "upholstery")
            w = (x1 - x0 - 0.30) / 3                                     # (split, as real ones are: the seams and gaps
            for k in range(3):                                           # are what the camera sees)
                cx0 = x0 + 0.15 + k * w
                box(f"{n}_cushion{k}", cx0 + 0.005, y0 + 0.02, cx0 + w - 0.005, y1 - 0.20, 0.30, 0.44, "upholstery", solid=False)
                box(f"{n}_backcushion{k}", cx0 + 0.005, y1 - 0.34, cx0 + w - 0.005, y1 - 0.20, 0.44, h - 0.05, "upholstery", solid=False)
            for k, (px, yaw) in enumerate(((x0 + 0.32, 0.4), (x1 - 0.32, -0.3))):   # two throw pillows against the arms
                box(f"{n}_pillow{k}", px - 0.2, y1 - 0.42, px + 0.2, y1 - 0.30, 0.44, 0.80, "pillow", solid=False, euler=f"-0.25 0 {yaw}")
        elif n == "tv_console":                                          # a cabinet on 12 cm legs, a TV on a stand on it
            legs(n, x0, y0, x1, y1, 0.12); cabinets.carcass(f"{n}_body", x0, y0 + cabinets.PROUD, x1, y1, 0.12, h)
            cabinets.fronts(n, x0, x1, y0 + cabinets.PROUD, -1, 0.12, h)                                     # doors ([WTj])
            mx, my = (x0 + x1) / 2, (y0 + y1) / 2
            box("tv_stand", mx - 0.15, my - 0.08, mx + 0.15, my + 0.08, h, h + 0.08, "screen", solid=False)
            box("tv", mx - 0.62, my - 0.02, mx + 0.62, my + 0.02, h + 0.08, h + 0.80, "screen", solid=False)
        elif n == "kitchen_island":                                      # worktop overhangs the dining side (-y) by 25 cm
            P = cabinets.PROUD                                              # doors on the kitchen side, plain panels on the
            cabinets.carcass(f"{n}_base", x0 + P, y0 + 0.25 + P, x1 - P, y1 - P, 0.0, h - 0.04)                # seating side
            cabinets.fronts(n, x0 + P, x1 - P, y1 - P, 1, 0.10, h - 0.04)                                     # and the ends
            cabinets.panels(f"{n}_seat", x0 + P, x1 - P, y0 + 0.25 + P, -1, 0.10, h - 0.04)                   # ([WTj])
            cabinets.ends(n, x0 + P, x1 - P, y0 + 0.25, y1, 0.0, h - 0.04)
            box(f"{n}_top", x0, y0, x1, y1, h - 0.04, h, "stone")
        elif n == "kitchen_counter":                                     # cabinets over a 10 cm high, 7 cm deep toe-kick
            box(f"{n}_kick", x0, y0 + 0.07, x1, y1, 0.0, 0.10)
            cabinets.carcass(f"{n}_body", x0, y0 + cabinets.PROUD, x1, y1, 0.10, h - 0.04)                   # drawers over doors
            cabinets.fronts(n, x0, x1, y0 + cabinets.PROUD, -1, 0.10, h - 0.04, drawers=True)                # ([WTj])
            box(f"{n}_top", x0, y0 - 0.02, x1, y1, h - 0.04, h, "stone")
        elif n in FIRESIDE:
            out.extend(FIRESIDE[n](x0, y0, x1, y1, h, box))
        else:
            box(f"{n}", x0, y0, x1, y1, 0.0, h)
        if q:                                                           # world = centre + R (p - centre)
            inner = out[k_out:] + cabinets.GEOMS[k_geo:]; del out[k_out:]; del cabinets.GEOMS[k_geo:]
            out.append(f'    <body name="turn_{n}" pos="{cx:.4f} {cy:.4f} 0" euler="0 0 {q * math.pi / 2:.6f}">'
                       f'<body pos="{-cx:.4f} {-cy:.4f} 0">\n' + "\n".join(inner) + '\n    </body></body>')
    if (mh.DEMO_PLANS.get(mh.LAYOUT_SEED) or {}).get("dressing"): out += demo_dressing(box)
    if (mh.DEMO_PLANS.get(mh.LAYOUT_SEED) or {}).get("pendant"): out += pendant()
    return out + cabinets.GEOMS


def _prim(name, kind, x, y, z, size, rgba, yaw=0.0, euler=None):
    e = euler if euler else (f"0 0 {yaw:.4f}" if yaw else None)
    return (f'    <geom name="{name}" type="{kind}" pos="{x:.3f} {y:.3f} {z:.3f}" size="{size}" rgba="{rgba}" contype="1" '
            f'conaffinity="0"' + (f' euler="{e}"' if e else "") + '/>')


def fireplace(x0, y0, x1, y1, h, box):
    """a fireplace on the wall behind it (-x) (2 Oct 2026): a stucco surround in the wall's finish under
    a thick shelf mantel with a rounded lip, a panel of big beige stone tiles, and a black firebox with brass trim and
    smoked glass doors. Unlit: the house is asleep (a lit fire is a steady glow, never a flicker)"""
    out = []; front = x1 - 0.02; my = (y0 + y1) / 2; shelf = h - 0.12
    box("fp_surround", x0, y0, front, y1, 0.0, shelf, "wall")
    box("fp_mantel", x0, y0 - 0.10, x1 + 0.05, y1 + 0.10, shelf, h, "wall")
    out.append(_prim("fp_mantel_lip", "capsule", x1 + 0.05, my, h - 0.05, f"0.05 {(y1 - y0) / 2 + 0.10:.3f}", "0.62 0.50 0.38 1",
                     euler="1.5708 0 0"))
    ty0, ty1, tz1, T, G = y0 + 0.16, y1 - 0.16, shelf - 0.14, 0.30, 0.004       # the tile panel: 30 cm tiles, 4 mm grout
    out.append(_prim("fp_grout", "box", front + 0.004, my, tz1 / 2, f"0.004 {(ty1 - ty0) / 2:.3f} {tz1 / 2:.3f}", "0.58 0.52 0.44 1"))
    fy0, fy1, fz0, fz1 = my - 0.40, my + 0.40, 0.10, 0.70                         # the firebox opening
    rng = np.random.default_rng(95); k = 0
    for zz in np.arange(0.0, tz1 - 1e-6, T):
        for yy in np.arange(ty0, ty1 - 1e-6, T):
            a, b = yy + G / 2, min(yy + T, ty1) - G / 2; c, d = zz + G / 2, min(zz + T, tz1) - G / 2
            if a >= fy0 - 0.02 and b <= fy1 + 0.02 and d <= fz1 + 0.02: continue      # behind the firebox
            t = rng.uniform(-0.05, 0.05); col = f"{0.74 + t:.3f} {0.62 + t * 0.9:.3f} {0.48 + t * 0.7:.3f} 1"
            out.append(_prim(f"fp_tile{k}", "box", front + 0.010, (a + b) / 2, (c + d) / 2, f"0.004 {(b - a) / 2:.4f} {(d - c) / 2:.4f}", col)); k += 1
    out.append(_prim("fp_firebox", "box", front + 0.006, my, (fz0 + fz1) / 2, f"0.010 {(fy1 - fy0) / 2:.3f} {(fz1 - fz0) / 2:.3f}", "0.03 0.03 0.03 1"))
    for nm, yy, zz, sy, sz in (("top", my, fz1 - 0.02, (fy1 - fy0) / 2, 0.02), ("bot", my, fz0 + 0.015, (fy1 - fy0) / 2, 0.015),
                               ("l", fy0 + 0.02, (fz0 + fz1) / 2, 0.02, (fz1 - fz0) / 2), ("r", fy1 - 0.02, (fz0 + fz1) / 2, 0.02, (fz1 - fz0) / 2)):
        out.append(_prim(f"fp_frame_{nm}", "box", front + 0.020, yy, zz, f"0.006 {sy:.3f} {sz:.3f}", "0.05 0.05 0.05 1"))
    out.append(_prim("fp_brass", "box", front + 0.027, my, fz1 - 0.045, f"0.003 {(fy1 - fy0) / 2 - 0.04:.3f} 0.008", "0.74 0.58 0.30 1"))
    out.append(_prim("fp_glass", "box", front + 0.018, my, (fz0 + fz1) / 2 - 0.01, f"0.002 {(fy1 - fy0) / 2 - 0.04:.3f} {(fz1 - fz0) / 2 - 0.05:.3f}",
                     "0.06 0.05 0.05 0.92"))
    return out


FURNITURE_DIR = Path(__file__).resolve().parents[2] / "models" / "furniture"   # imported models (tools/import_glb.py)


def model(name, x, y, z=0.0, yaw=0.0, tag=None, orm=True, finish=None):
    """an imported model (models/furniture/<name>: tools/import_glb.py) placed at x, y, z turned by yaw, drawn only
    (contype 0: what the robot bumps into is the caller's); [] when it has not been imported. Its assets go into the
    scene once. finish: (roughness, metallic, specular) instead of its own gloss map (orm False)"""
    d = FURNITURE_DIR / name; meta = d / "meta.json"
    if not meta.exists(): return []
    parts = json.loads(meta.read_text())["parts"]; out = []; tag = tag or name
    for i, p in enumerate(parts):
        stem = Path(p["obj"]).stem; mat = f"{stem}_mat"; tx = p["textures"]
        if f'name="{stem}_m"' not in "".join(cabinets.ASSETS):
            cabinets.ASSETS.append(f'    <mesh name="{stem}_m" file="{d / p["obj"]}"/>')
            layers = ""
            for role, key, lin in (("rgb", "rgb", False), ("normal", "normal", True), ("orm", "orm", True)):
                if tx.get(key) and (role != "orm" or (orm and not finish)):
                    cabinets.ASSETS.append(f'    <texture type="2d" name="{stem}_{role}" file="{d / tx[key]}"' + (' colorspace="linear"/>' if lin else "/>"))
                    layers += f'<layer texture="{stem}_{role}" role="{role}"/>'
            r_, m_, s_ = finish or (p.get("roughness", 0.6), p.get("metallic", 0.0), 0.5)
            col = "" if tx.get("rgb") else f' rgba="{" ".join(str(c) for c in p["rgba"])}"'
            cabinets.ASSETS.append(f'    <material name="{mat}" texuniform="false" roughness="{r_}" metallic="{m_}" specular="{s_}"{col}>{layers}</material>')
        out.append(f'    <geom name="{tag}_{i}" type="mesh" mesh="{stem}_m" material="{mat}" pos="{x:.3f} {y:.3f} {z:.3f}" '
                   f'euler="0 0 {yaw:.4f}" contype="0" conaffinity="0"/>')
    return out


CHAIR_YAW = -math.pi / 4             # the wing chair turned out to the room
GSO_DIR = Path(__file__).resolve().parents[2] / "models" / "items" / "gso"   # the scanned household items (tools/fetch_items.sh)


def scanned(item, x, y, z, yaw=0.0, tag=None):
    """a Google Scanned Objects item (CC-BY 4.0) set down at x, y with its lowest point on z, drawn only; [] when not
    fetched"""
    d = GSO_DIR / item
    if not (d / "model.obj").exists(): return []
    zmin = min(float(l.split()[3]) for l in open(d / "model.obj") if l.startswith("v "))
    if f'name="gso_{item}_m"' not in "".join(cabinets.ASSETS):
        cabinets.ASSETS += [f'    <mesh name="gso_{item}_m" file="{d / "model.obj"}"/>',
                            f'    <texture type="2d" name="gso_{item}_t" file="{d / "texture_1k.png"}"/>',
                            f'    <material name="gso_{item}_mat" texture="gso_{item}_t" roughness="0.35" specular="0.4"/>']
    return [f'    <geom name="{tag or item}" type="mesh" mesh="gso_{item}_m" material="gso_{item}_mat" pos="{x:.3f} {y:.3f} {z - zmin:.4f}" '
            f'euler="0 0 {yaw:.4f}" contype="0" conaffinity="0"/>']


def wing_chair(x0, y0, x1, y1, h, box, yaw=CHAIR_YAW, col="0.09 0.20 0.14 1"):
    """a leather wing chair beside the fire, turned out to the room (a reading chair by the lamp, side-on to the fire, not
    facing it): the scanned model (Sketchfab 'Vintage
    Wingback Armchair' by pranavranjan2212, CC-BY 4.0; models/furniture/wing_chair) where it has been imported, drawn
    only; the simple shapes beneath it (seat, tall back with wings, rolled arms, legs) are what the robot bumps into,
    unseen. Without the model, those shapes are the chair"""
    out = []; cx, cy = (x0 + x1) / 2, (y0 + y1) / 2; c, s = math.cos(yaw), math.sin(yaw)
    drawn = model("wing_chair", cx, cy, 0.0, yaw, tag="wc_model", finish=(0.62, 0.0, 0.25))   # satin: its gloss map glared
    if drawn:
        out += drawn
        col = "0 0 0 0"                                                     # the shapes below: collision only
    def at(fx, ry): return cx + c * fx - s * ry, cy + s * fx + c * ry         # chair frame: fx forward, ry to its left
    for nm, fx, ry, z, sz in (("seat", 0.02, 0.0, 0.42, "0.27 0.27 0.07"), ("back", -0.30, 0.0, 0.78, "0.07 0.36 0.32"),
                              ("wing_l", -0.16, 0.33, 0.86, "0.15 0.05 0.20"), ("wing_r", -0.16, -0.33, 0.86, "0.15 0.05 0.20"),
                              ("base", 0.0, 0.0, 0.24, "0.34 0.38 0.10")):
        x, y = at(fx, ry); out.append(_prim(f"wc_{nm}", "box", x, y, z, sz, col, yaw=yaw))
    for side in (1, -1):
        x, y = at(0.02, side * 0.33); out.append(_prim(f"wc_arm{side}", "capsule", x, y, 0.58, "0.06 0.28", col, euler=f"0 1.5708 {yaw:.4f}"))
        for fx in (0.28, -0.30):
            x, y = at(fx, side * 0.30); out.append(_prim(f"wc_leg{side}_{fx:+.2f}", "cylinder", x, y, 0.07, "0.025 0.07", "0.22 0.13 0.07 1" if col[-1] != "0" else col))
    return out


def side_table(x0, y0, x1, y1, h, box):
    """a small round side table at the wing chair's left arm, a hand's reach from it (for the cold open's martini);
    at its own cells when there is no chair"""
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    for c0, r0, c1, r1, _h, n in mh.FURNITURE:
        if n == "wing_chair":
            (ax, ay), (bx, by) = mh.cell_to_world(c0, r0), mh.cell_to_world(c1, r1); wx, wy = (ax + bx) / 2, (ay + by) / 2
            fwd = np.array([math.cos(CHAIR_YAW), math.sin(CHAIR_YAW)]); left = np.array([-fwd[1], fwd[0]])
            cx, cy = np.array([wx, wy]) + 0.60 * left - 0.05 * fwd           # its edge just clear of the arm
    return [_prim("st_top", "cylinder", cx, cy, h - 0.015, "0.20 0.015", "0.30 0.18 0.10 1"),
            _prim("st_post", "cylinder", cx, cy, (h - 0.03) / 2 + 0.02, f"0.025 {(h - 0.05) / 2:.3f}", "0.30 0.18 0.10 1"),
            _prim("st_foot", "cylinder", cx, cy, 0.015, "0.14 0.015", "0.30 0.18 0.10 1")]


LAMPSHADE = '    <material name="lampshade" rgba="0.96 0.86 0.66 1" emission="0.55" specular="0" shininess="0"/>'
PENDANT_Z = 1.55                     # m: the pendant's shade bottom, ~80 cm over the table


def reading_lamp(x0, y0, x1, y1, h, box):
    """a floor reading lamp behind the wing chair: a weighted base,
    a slim stem and a fabric drum shade, lit from inside (the house's lamp light sits just under it: mh.LAMP)"""
    cx, cy = mh.LAMP; cabinets.ASSETS.append(LAMPSHADE)
    return [_prim("rl_base", "cylinder", cx, cy, 0.012, "0.14 0.012", "0.10 0.09 0.08 1"),
            _prim("rl_stem", "cylinder", cx, cy, 0.75, "0.011 0.74", "0.55 0.44 0.26 1"),
            f'    <geom name="rl_shade" type="cylinder" pos="{cx:.3f} {cy:.3f} {h - 0.13:.3f}" size="0.19 0.13" material="lampshade" '
            f'contype="0" conaffinity="0"/>']


def pendant_at():
    for c0, r0, c1, r1, _h, n in mh.FURNITURE:
        if n == "dining_table":
            (ax, ay), (bx, by) = mh.cell_to_world(c0, r0), mh.cell_to_world(c1, r1); return (ax + bx) / 2, (ay + by) / 2
    return mh.LAMP


def pendant():
    """the pendant over the dining table: a cord from the ceiling and a dark metal dome, its bulb glowing under it"""
    px, py = pendant_at(); cabinets.ASSETS.append(LAMPSHADE) if LAMPSHADE not in cabinets.ASSETS else None
    top = mh.WALL_H
    return [_prim("pd_cord", "cylinder", px, py, (top + PENDANT_Z + 0.16) / 2, f"0.004 {(top - PENDANT_Z - 0.16) / 2:.3f}", "0.08 0.08 0.08 1"),
            _prim("pd_dome", "ellipsoid", px, py, PENDANT_Z + 0.07, "0.21 0.21 0.10", "0.09 0.08 0.07 1"),
            _prim("pd_rim", "cylinder", px, py, PENDANT_Z + 0.005, "0.205 0.005", "0.62 0.48 0.26 1"),
            f'    <geom name="pd_bulb" type="sphere" pos="{px:.3f} {py:.3f} {PENDANT_Z + 0.02:.3f}" size="0.05" material="lampshade" '
            f'contype="0" conaffinity="0"/>']


FIRESIDE = {"fireplace": fireplace, "wing_chair": wing_chair, "side_table": side_table, "reading_lamp": reading_lamp}   # the
                                                                                                    # demo room's fireside (plan 95)


def demo_dressing(box):
    """a demo room lived in (subsystem-demos [qEP]; the back cabinet no longer shallow
    and empty): upper cabinets over the counter, and things on every surface,
    as simple shapes in the house's finishes (static: the robot's lidar and cameras see them as it would a room's)"""
    out = []
    def prim(name, kind, x, y, z, size, rgba, euler=None):
        out.append(f'    <geom name="dd_{name}" type="{kind}" pos="{x:.3f} {y:.3f} {z:.3f}" size="{size}" rgba="{rgba}" '
                   f'contype="1" conaffinity="0"{f' euler="{euler}"' if euler else ""}/>')
    import math as _m
    def plant(name, x, y, z, scale=1.0, k=9):
        """a snake plant: slim leaves fanning up and out of the pot (ellipsoids, each leaned and turned its own way)"""
        for i in range(k):
            a = 2 * _m.pi * i / k + 0.4 * _m.sin(3.1 * i); lean = 0.18 + 0.12 * ((i * 7) % 3) / 2; L = (0.20 + 0.06 * ((i * 5) % 4) / 3) * scale
            cx, cy, cz = x + _m.cos(a) * _m.sin(lean) * L * 0.9, y + _m.sin(a) * _m.sin(lean) * L * 0.9, z + _m.cos(lean) * L
            prim(f"{name}_leaf{i}", "ellipsoid", cx, cy, cz, f"{0.022 * scale:.3f} {0.006 * scale:.3f} {L:.3f}",
                 "0.16 0.34 0.15 1" if i % 3 else "0.26 0.42 0.18 1", euler=f"{-lean * _m.sin(a):.3f} {lean * _m.cos(a):.3f} {a:.3f}")
    def flowers(name, x, y, z, k=6):
        """a few stems in a vase, each with a bloom, at different heights and leaning out"""
        cols = ("0.86 0.72 0.30 1", "0.80 0.32 0.22 1", "0.92 0.88 0.80 1")
        for i in range(k):
            a = 2 * _m.pi * i / k + 0.3; lean = 0.22; L = 0.16 + 0.05 * (i % 3)
            cx, cy = x + _m.cos(a) * _m.sin(lean) * L / 2, y + _m.sin(a) * _m.sin(lean) * L / 2
            prim(f"{name}_stem{i}", "cylinder", cx, cy, z + L / 2, f"0.0035 {L / 2:.3f}", "0.24 0.36 0.18 1",
                 euler=f"{-lean * _m.sin(a):.3f} {lean * _m.cos(a):.3f} 0")
            prim(f"{name}_bloom{i}", "sphere", x + _m.cos(a) * _m.sin(lean) * L, y + _m.sin(a) * _m.sin(lean) * L, z + _m.cos(lean) * L,
                 "0.028", cols[i % 3])
    for c0, r0, c1, r1, h, n in mh.FURNITURE:
        (ax, ay), (bx, by) = mh.cell_to_world(c0, r0), mh.cell_to_world(c1, r1); e = mh.CELL / 2
        x0, x1, y0, y1 = min(ax, bx) - e, max(ax, bx) + e, min(ay, by) - e, max(ay, by) + e
        if n == "kitchen_counter":                                    # uppers on the wall over it, 35 cm deep, doors down
            yb = y1; xu = min(x1, x0 + 1.30)                          # (stopping short of the window over the sink)
            cabinets.carcass("dd_upper", x0, yb - 0.35, xu, yb, 1.45, 2.15)
            cabinets.fronts("dd_upper", x0, xu, yb - 0.35, -1, 1.45, 2.15)
            xr = x0 + 2.55                                            # and past the window, where the counter runs on
            if x1 - xr > 0.45:
                cabinets.carcass("dd_upper2", xr, yb - 0.35, x1, yb, 1.45, 2.15)
                cabinets.fronts("dd_upper2", xr, x1, yb - 0.35, -1, 1.45, 2.15)
            t = h
            prim("kettle", "cylinder", x0 + 0.35, y1 - 0.25, t + 0.11, "0.085 0.11", "0.12 0.12 0.13 1")
            for k, (dx, hh) in enumerate(((0.75, 0.14), (0.88, 0.11), (0.99, 0.09))):   # canisters
                prim(f"can{k}", "cylinder", x0 + dx, y1 - 0.18, t + hh, f"0.055 {hh}", "0.86 0.82 0.74 1")
            box("dd_board", x0 + 1.25, y1 - 0.45, x0 + 1.65, y1 - 0.18, t, t + 0.02, "wood")
            fruit = model("bananas", x0 + 1.40, y0 + 0.16, t, 0.3, tag="dd_bananas")     # scanned where imported, else simple shapes
            out.extend(fruit)
            if not fruit:
                prim("fruitbowl", "cylinder", x1 - 0.55, y0 + 0.30, t + 0.04, "0.15 0.04", "0.55 0.33 0.20 1")
                for k, (dx, dy, col) in enumerate(((-0.05, 0.0, "0.80 0.18 0.10 1"), (0.06, 0.03, "0.88 0.62 0.12 1"), (0.0, 0.07, "0.30 0.55 0.15 1"))):
                    prim(f"fruit{k}", "sphere", x1 - 0.55 + dx, y0 + 0.30 + dy, t + 0.11, "0.045", col)
            snake = model("snake_plant", x1 - 0.20, y1 - 0.20, t, 0.3, tag="dd_snake")
            out.extend(snake)
            if not snake:
                prim("pot", "cylinder", x1 - 0.18, y1 - 0.20, t + 0.09, "0.08 0.09", "0.62 0.32 0.20 1")
                plant("plant", x1 - 0.18, y1 - 0.20, t + 0.17, 1.0)
            for k, dx in enumerate((1.90, 1.98)):
                prim(f"bottle{k}", "cylinder", x0 + dx, y1 - 0.15, t + 0.13, "0.035 0.13", "0.20 0.32 0.22 1" if k else "0.45 0.12 0.10 1")
        elif n == "tv_console":
            t = h
            for k, (hh, col) in enumerate(((0.035, "0.55 0.15 0.12 1"), (0.03, "0.18 0.28 0.45 1"), (0.025, "0.80 0.74 0.60 1"))):
                z = t + sum(2 * q for q, _ in ((0.035, 0), (0.03, 0), (0.025, 0))[:k]) + hh
                prim(f"book{k}", "box", x0 + 0.30, (y0 + y1) / 2, z, f"0.13 0.09 {hh}", col, euler=f"0 0 {0.15 * k}")
            prim("tvpot", "cylinder", x1 - 0.20, (y0 + y1) / 2, t + 0.08, "0.07 0.08", "0.85 0.84 0.80 1")
            plant("tvplant", x1 - 0.20, (y0 + y1) / 2, t + 0.15, 0.85)
        elif n == "dining_table":
            t = h; mx, my = (x0 + x1) / 2, (y0 + y1) / 2
            for k, (dx, dy) in enumerate(((-0.35, 0.28), (0.35, 0.28), (-0.35, -0.28), (0.35, -0.28))):   # placemats
                box(f"dd_mat{k}", mx + dx - 0.18, my + dy - 0.13, mx + dx + 0.18, my + dy + 0.13, t, t + 0.004, "upholstery")
            centre = model("table_plant", mx, my, t, 0.4, tag="dd_tableplant")          # a small plant, salt and pepper
            out.extend(centre); out.extend(model("salt_pepper", mx + 0.20, my - 0.04, t, 1.2, tag="dd_saltpepper"))
            if not centre:
                prim("vase", "cylinder", mx, my, t + 0.12, "0.05 0.12", "0.25 0.30 0.38 1")
                flowers("flowers", mx, my, t + 0.22)
            mug = scanned("Room_Essentials_Mug_White_Yellow", mx + 0.42, my + 0.22, t, 2.4, tag="dd_mug_d")   # a real mug
            out.extend(mug)
            if not mug: prim("mug_d", "cylinder", mx + 0.42, my + 0.22, t + 0.05, "0.04 0.05", "0.92 0.90 0.86 1")
        elif n == "coffee_table":
            t = h; mx, my = (x0 + x1) / 2, (y0 + y1) / 2
            for k, (hh, col) in enumerate(((0.02, "0.15 0.35 0.40 1"), (0.018, "0.85 0.80 0.70 1"))):
                prim(f"cbook{k}", "box", mx - 0.25, my, t + 0.02 + 0.04 * k, f"0.14 0.10 {hh}", col, euler=f"0 0 {0.3 * k - 0.1}")
            prim("mug_c", "cylinder", mx + 0.25, my + 0.08, t + 0.05, "0.04 0.05", "0.75 0.25 0.18 1")
            prim("remote", "box", mx + 0.05, my - 0.15, t + 0.012, "0.09 0.025 0.012", "0.10 0.10 0.11 1", euler="0 0 0.4")
    return out


def window_dressing(name, wx, wy, n, half, along):
    """what a window looks like from inside: slatted blinds half open across its inner face, and beyond it the view
    out, a plane lit by day and dark by night (material "outdoors", set per night). Filament has no sky behind a
    window (it drew the renderer's flat blue-grey), and real windows are mostly blinds and a blown-out view (26 Sep).
    Neither collides; the lidar looks below the sill"""
    out, t = [], mh.CELL / 2
    if (mh.DEMO_PLANS.get(mh.LAYOUT_SEED) or {}).get("shutters"): out += shutters(name, wx, wy, n, half, along)
    bx_, by_ = wx + n[0] * (t - 0.03), wy + n[1] * (t - 0.03)
    for k, z in (() if out else enumerate(np.arange(WINDOW_Z[0] + 0.03, WINDOW_Z[1] - 0.02, 0.05))):   # 5 cm slats, tilted 30 deg
        size = f"{half:.3f} 0.025 0.0015" if along else f"0.025 {half:.3f} 0.0015"
        eul = "0.52 0 0" if along else "0 0.52 0"
        out.append(f'    <geom name="{name}_slat{k}" type="box" pos="{bx_:.3f} {by_:.3f} {z:.3f}" euler="{eul}" size="{size}" '
                   f'material="blinds" contype="0" conaffinity="0"/>')
    ox, oy = wx - n[0] * (t + 0.4), wy - n[1] * (t + 0.4)
    ax_ = (f"{-n[1]:.0f} 0 0 0 0 1" if n[1] else "1 0 0 0 0 1") if along else (f"0 {n[0]:.0f} 0 0 0 1" if n[0] else "0 1 0 0 0 1")
    out.append(f'    <geom name="{name}_out" type="plane" pos="{ox:.3f} {oy:.3f} {(WINDOW_Z[0] + WINDOW_Z[1]) / 2:.3f}" xyaxes="{ax_}" '
               f'size="{half + 0.4:.3f} {(WINDOW_Z[1] - WINDOW_Z[0]) / 2 + 0.3:.3f} 0.1" material="outdoors" contype="0" conaffinity="0"/>')
    return out


SHUTTER_RGBA = "0.07 0.05 0.04 1"     # dark espresso


def shutters(name, wx, wy, n, half, along):
    """plantation shutters in a window (in place of Venetian blinds, which looked cheap): a dark frame
    round the opening, two panels split by a mid rail, wide louvres (9 cm, 8.5 cm apart) closed in the top section and
    tilted open in the bottom, a thin tilt rod down each section. The demo room's (its plan's "shutters")"""
    out, t = [], mh.CELL / 2; a = (1.0, 0.0) if along else (0.0, 1.0); z0, z1 = WINDOW_Z; mid = z0 + 0.62 * (z1 - z0)
    def piece(nm, u, w, z, su, sw, sz, tilt=0.0):
        x, y = wx + a[0] * u + n[0] * w, wy + a[1] * u + n[1] * w
        size = f"{su:.4f} {sw:.4f} {sz:.4f}" if along else f"{sw:.4f} {su:.4f} {sz:.4f}"
        eul = (f' euler="{tilt:.3f} 0 0"' if along else f' euler="0 {tilt:.3f} 0"') if tilt else ""
        out.append(f'    <geom name="{name}_sh_{nm}" type="box" pos="{x:.3f} {y:.3f} {z:.3f}" size="{size}"{eul} rgba="{SHUTTER_RGBA}" '
                   f'contype="0" conaffinity="0"/>')
    F, wf = 0.045, t - 0.005                                             # the frame: 9 cm face, just proud of the wall
    piece("frame_top", 0, wf, z1 + F, half + 2 * F, 0.015, F); piece("frame_bot", 0, wf, z0 - F, half + 2 * F, 0.015, F)
    piece("frame_l", -half - F, wf, (z0 + z1) / 2, F, 0.015, (z1 - z0) / 2); piece("frame_r", half + F, wf, (z0 + z1) / 2, F, 0.015, (z1 - z0) / 2)
    S, wp = 0.025, t - 0.035                                             # panels: 5 cm stiles and rails, set in the reveal
    for side, (u0, u1) in (("a", (-half, 0.0)), ("b", (0.0, half))):
        uc, uh = (u0 + u1) / 2, (u1 - u0) / 2
        piece(f"{side}_stile0", u0 + S, wp, (z0 + z1) / 2, S, 0.015, (z1 - z0) / 2); piece(f"{side}_stile1", u1 - S, wp, (z0 + z1) / 2, S, 0.015, (z1 - z0) / 2)
        for nm, z in (("top", z1 - S), ("bot", z0 + S), ("mid", mid)): piece(f"{side}_rail_{nm}", uc, wp, z, uh, 0.015, S + (0.015 if nm == "mid" else 0))
        for sec, (za, zb, tilt) in (("lo", (z0 + 2 * S, mid - S - 0.015, 0.45)), ("hi", (mid + S + 0.015, z1 - 2 * S, 1.45))):
            zs = np.arange(za + 0.045, zb - 0.03, 0.085)
            for k, z in enumerate(zs): piece(f"{side}_{sec}_louvre{k}", uc, wp, z, uh - 2 * S, 0.045, 0.006, tilt)
            if len(zs): piece(f"{side}_{sec}_rod", uc, wp + 0.05, (zs[0] + zs[-1]) / 2, 0.006, 0.006, (zs[-1] - zs[0]) / 2 + 0.03)
    return out


def walls_xml(g):
    """walls with windows in the outside ones (between WINDOW_Z), and a ceiling: the house is indoors.
    Returns (geom lines, windows): each window's centre (x, y) and the unit normal pointing into the house."""
    out, windows = [], []
    def box(name, x0, y0, x1, y1, z0, z1, mat, outside=False):
        """a wall block. An outside wall is seen through from outside, as the ceiling from above (to
        watch the robot in the viewer): its box moves to group 4 (the viewer hides it, the lidar's depth cameras draw
        it, Filament skips it) and its surface is a plane facing into the house, drawn from inside only"""
        (ax, ay), (bx, by) = mh.cell_to_world(x0, y0), mh.cell_to_world(x1, y1)
        cx, cy, sx, sy = (ax + bx) / 2, (ay + by) / 2, (abs(bx - ax) + mh.CELL) / 2, (abs(by - ay) + mh.CELL) / 2
        cz, sz = (z0 + z1) / 2, (z1 - z0) / 2
        out.append(f'    <geom name="{name}" type="box" pos="{cx:.3f} {cy:.3f} {cz:.3f}" size="{sx:.3f} {sy:.3f} {sz:.3f}" '
                   f'material="{mat}" contype="1" conaffinity="0"{' group="4"' if outside else ""}/>')
        if z0 == 0:                                           # a baseboard: 9 cm of white trim, 1.2 cm proud of the wall
            for sgn in ((-np.sign(cy if sx >= sy else cx),) if outside else (-1.0, 1.0)):
                if sx >= sy: bx_, by_, hx, hy = cx, cy + sgn * (sy + 0.006), sx, 0.006
                else: bx_, by_, hx, hy = cx + sgn * (sx + 0.006), cy, 0.006, sy
                out.append(f'    <geom name="{name}_base{"n" if sgn < 0 else "p"}" type="box" pos="{bx_:.3f} {by_:.3f} 0.045" '
                           f'size="{hx:.3f} {hy:.3f} 0.045" material="trim" contype="0" conaffinity="0"/>')
        if not outside: return
        if sx >= sy:                                          # runs along x: faces +y or -y, toward the house's middle
            ny = -np.sign(cy); px, py, half, ax_ = cx, cy + ny * (sy + 0.002), sx, f"{-ny:.0f} 0 0 0 0 1"
        else:
            nx = -np.sign(cx); px, py, half, ax_ = cx + nx * (sx + 0.002), cy, sy, f"0 {nx:.0f} 0 0 0 1"
        out.append(f'    <geom name="{name}_face" type="plane" pos="{px:.3f} {py:.3f} {cz:.3f}" xyaxes="{ax_}" '
                   f'size="{half:.3f} {sz:.3f} 0.1" material="{mat}" contype="0" conaffinity="0"/>')
    for i, (x0, y0, x1, y1) in enumerate(mh.wall_rects(g)):
        outside = x0 == 0 or y0 == 0 or x1 == mh.W - 1 or y1 == mh.H - 1
        on_edge = (y0 == y1 and y0 in (0, mh.H - 1)) or (x0 == x1 and x0 in (0, mh.W - 1))   # runs along it (seen through;
        if mh.LAYOUT_SEED in mh.HOLDOUT_OWN: outside = on_edge   # [A7G]: an inside wall meeting the outer one (merged with
                                                                 # its end cell) stays an inside wall: no windows, drawn whole.
                                                                 # The older plans keep theirs, byte for byte
        along = x1 - x0 >= y1 - y0; L = (x1 - x0 if along else y1 - y0) + 1
        if not outside or L < WINDOW_EVERY:
            box(f"wall_{i}", x0, y0, x1, y1, 0, mh.WALL_H, "wall", on_edge); continue
        k0 = 0
        for w0 in range((L % WINDOW_EVERY) // 2 + WINDOW_EVERY // 2 - 1, L - 2, WINDOW_EVERY):   # windows, centred along the run
            seg = lambda a, b: (x0 + a, y0, x0 + b, y1) if along else (x0, y0 + a, x1, y0 + b)
            if w0 > k0: box(f"wall_{i}_{k0}", *seg(k0, w0 - 1), 0, mh.WALL_H, "wall", outside=True)
            box(f"wall_{i}_{w0}_sill", *seg(w0, w0 + WINDOW_W - 1), 0, WINDOW_Z[0], "wall", outside=True)
            (ax, ay), (bx, by) = mh.cell_to_world(*seg(w0, w0 + WINDOW_W - 1)[:2]), mh.cell_to_world(*seg(w0, w0 + WINDOW_W - 1)[2:])
            wx, wy = (ax + bx) / 2, (ay + by) / 2; n = np.array([0.0, -np.sign(wy)]) if along else np.array([-np.sign(wx), 0.0])
            windows.append((wx, wy, float(n[0]), float(n[1])))
            half = (abs(bx - ax) + mh.CELL) / 2 if along else (abs(by - ay) + mh.CELL) / 2
            out.extend(window_dressing(f"win_{i}_{w0}", wx, wy, n, half, along))
            box(f"wall_{i}_{w0}_head", *seg(w0, w0 + WINDOW_W - 1), WINDOW_Z[1], mh.WALL_H, "wall", outside=True)
            k0 = w0 + WINDOW_W
        if k0 <= L - 1: box(f"wall_{i}_{k0}", *((x0 + k0, y0, x1, y1) if along else (x0, y0 + k0, x1, y1)), 0, mh.WALL_H, "wall", outside=True)
    out.append(f'    <geom name="ceiling" type="plane" pos="0 0 {mh.WALL_H:.2f}" quat="0 1 0 0" size="{HX + 0.2:.2f} {HY + 0.2:.2f} 0.1" '
               f'material="ceiling" contype="0" conaffinity="0"/>')   # faces down: a plane is drawn from its front only, so
                                                                      # the viewer sees in from above
    return out, windows
TOY_COLORS = ["0.85 0.2 0.2 1", "0.2 0.5 0.85 1", "0.95 0.75 0.1 1", "0.2 0.7 0.3 1", "0.6 0.3 0.75 1"]


def new_item(rng, i, night, g, taken):
    kind = rng.choices(["toy", "lego", "tissue", "string"], weights=[4, 3, 2, 2])[0]
    tw = sum(r[4] for r in mh.REGIONS)
    for _ in range(4000):
        pick = rng.random() * tw
        for reg in mh.REGIONS:
            pick -= reg[4]
            if pick <= 0:
                break
        x, y = rng.randint(reg[0], reg[2]), rng.randint(reg[1], reg[3])
        if abs(x - mh.START[0]) <= 3 and abs(y - mh.START[1]) <= 3:
            continue
        if any(not (0 <= xx < mh.W and 0 <= yy < mh.H) or g[yy][xx] or (xx, yy) in taken
               for yy in range(y - 2, y + 3) for xx in range(x - 2, x + 3)):
            continue
        taken.add((x, y)); break
    wx, wy = mh.cell_to_world(x, y)
    it = {"id": f"n{night}_{i:02d}", "kind": kind, "x": round(wx + rng.uniform(-.1, .1), 3),
          "y": round(wy + rng.uniform(-.1, .1), 3), "yaw": round(rng.uniform(0, math.pi), 3), "first_night": night, "cell": [x, y]}
    if kind == "toy":
        it["height_m"] = rng.choice([0.05, 0.08, 0.12, 0.2, 0.3]); it["half"] = [round(rng.uniform(.05, .13), 3)] * 2
        it["rgba"] = rng.choice(TOY_COLORS)
    elif kind == "lego":
        it["height_m"] = 0.011; it["half"] = [0.016, 0.008]; it["rgba"] = rng.choice(TOY_COLORS)
    elif kind == "tissue":
        it["height_m"] = 0.003; it["half"] = [round(rng.uniform(.06, .09), 3)] * 2; it["rgba"] = "0.97 0.97 0.95 1"
    else:
        it["height_m"] = 0.004; it["half"] = [0.15, 0.002]; it["rgba"] = "0.9 0.9 0.85 1"
        it["segments"] = [round(rng.uniform(-0.6, 0.6), 2) for _ in range(3)]   # zigzag bends
    return it


def rgba(rng, sat, val, alpha=1.0):
    r, g, b = colorsys.hsv_to_rgb(rng.random(), rng.uniform(*sat), rng.uniform(*val))
    return f"{r:.2f} {g:.2f} {b:.2f} {alpha}"


LEGO = ["0.8 0.1 0.1 1", "0.05 0.3 0.7 1", "0.95 0.8 0.05 1", "0.1 0.55 0.2 1", "0.95 0.95 0.95 1",
        "0.08 0.08 0.08 1", "0.55 0.55 0.55 1", "0.85 0.75 0.55 1", "0.95 0.45 0.05 1", "0.85 0.9 0.95 0.45"]


def new_item_varied(rng, i, night, g, taken):
    """Held-out litter: every kind varies in colour, size and shape, including colours that blend in."""
    it = new_item(rng, i, night, g, taken)                      # reuse placement; overwrite appearance
    kind = rng.choices(["toy", "lego", "tissue", "string", "sock", "paper"], weights=[3, 3, 2, 2, 2, 1])[0]
    it["kind"] = kind; it.pop("segments", None)
    if kind == "toy":
        it["shape"] = rng.choice(["box", "cylinder", "sphere", "capsule"]); r = round(rng.uniform(.03, .15), 3)
        it["half"] = [r, round(rng.uniform(.03, .15), 3)]; it["height_m"] = round(2 * r if it["shape"] == "sphere" else rng.uniform(.04, .3), 3)
        it["rgba"] = rgba(rng, (.1, 1), (.2, 1))
    elif kind == "lego":
        a, b = rng.choice([(1, 1), (1, 2), (2, 2), (2, 4), (1, 4)])
        it["half"] = [round(b * .004, 4), round(a * .004, 4)]; it["height_m"] = 0.0115; it["rgba"] = rng.choice(LEGO)
    elif kind == "tissue":
        crumpled = rng.random() < .5; it["crumpled"] = crumpled
        it["half"] = [round(rng.uniform(.03, .06), 3)] * 2 if crumpled else [round(rng.uniform(.05, .10), 3)] * 2
        it["height_m"] = round(rng.uniform(.02, .05), 3) if crumpled else round(rng.uniform(.002, .004), 4)
        it["rgba"] = rgba(rng, (0, .15), (.85, 1))
    elif kind == "string":
        segs = [[round(rng.uniform(-1, 1), 2), round(rng.uniform(.05, .15), 3)] for _ in range(rng.randint(2, 6))]
        it["segs"] = segs; it["radius"] = round(rng.uniform(.0015, .004), 4); it["length_m"] = round(sum(L for _, L in segs), 3)
        it["half"] = [it["length_m"] / 2, it["radius"]]; it["height_m"] = round(2 * it["radius"], 4); it["rgba"] = rgba(rng, (0, 1), (.1, 1))
    elif kind == "sock":
        it["half"] = [round(rng.uniform(.09, .13), 3), round(rng.uniform(.035, .05), 3)]; it["height_m"] = round(rng.uniform(.01, .025), 3)
        it["rgba"] = rgba(rng, (0, .6), (.2, .95))
    else:
        it["half"] = [round(rng.uniform(.10, .15), 3), round(rng.uniform(.07, .105), 3)]; it["height_m"] = 0.001
        it["rgba"] = rgba(rng, (0, .2), (.8, 1))
    return it


def item_xml(it):
    h = it["height_m"]
    if it["kind"] == "string":                       # zigzag of thin capsules
        parts, px, py, ang = [], 0.0, 0.0, 0.0
        rad = it.get("radius", 0.002)
        segs = it.get("segs") or [[b, 0.1] for b in it["segments"]]
        for bend, L in segs:
            ang += bend
            nx, ny = px + L * math.cos(ang), py + L * math.sin(ang)
            parts.append(f'<geom type="capsule" fromto="{px:.3f} {py:.3f} 0 {nx:.3f} {ny:.3f} 0" size="0.002" '
                         f'rgba="{it["rgba"]}" contype="0" conaffinity="0"/>'.replace('size="0.002"', f'size="{rad}"'))
            px, py = nx, ny
        return f'    <body name="{it["id"]}" pos="{it["x"]} {it["y"]} 0.002" euler="0 0 {it["yaw"]}">{"".join(parts)}</body>'
    sx, sy = it["half"]; shape = it.get("shape", "box"); tilt = "0.35 0.2" if it.get("crumpled") else "0 0"
    size = {"box": f"{sx} {sy} {h/2:.4f}", "cylinder": f"{sx} {h/2:.4f}", "sphere": f"{sx}", "capsule": f"{min(sx, h/2):.4f} {max(h/2 - sx, 0.001):.4f}"}[shape]
    return (f'    <body name="{it["id"]}" pos="{it["x"]} {it["y"]} {h/2:.4f}" euler="{tilt} {it["yaw"]}">'
            f'<geom type="{shape}" size="{size}" rgba="{it["rgba"]}" contype="0" conaffinity="0"/></body>')


CARPET_M, RUG_M = 0.012, 0.008   # relief ([nAH] TF8): the carpet's top above the hard floor (pile and pad, est. 6-15 mm in
                                  # homes) and a rug's thickness on it (est.); the floor had been one flat plane
RUG_SHIFT = (0.0, 0.0)            # the rug stays where it lies ([yyJ]: a moving rug asked too
                                  # much for now); was sd 2 cm and 1.5 deg (rugs may move around a bit)


PET_NUDGE = 0.03                  # the pet's bowls (make_house.BOWLS, [nAH] TJu): a pet nudges them: sd of the shift (m), est.


def pet_bowls_tonight(seed, night):
    """the pet's bowls where they stand tonight: home, nudged (never more than 3 sd), any way round"""
    r = np.random.default_rng(seed * 6007 + night); out = []
    for i, (name, x, y) in enumerate(mh.BOWLS):
        dx, dy = np.clip(r.normal(0, PET_NUDGE, 2), -3 * PET_NUDGE, 3 * PET_NUDGE)
        out.append({"name": name, "uid": f"pet_bowl_{i}", "x": round(x + float(dx), 3), "y": round(min(y + float(dy), 4.40), 3),
                    "yaw": round(float(r.uniform(-math.pi, math.pi)), 3)})
    return out


def rug_tonight(rug, seed, night):
    """where the rug lies tonight: its home pose, nudged (never more than 3 sd); unmoved while RUG_SHIFT is zero"""
    if not rug or not any(RUG_SHIFT): return rug
    r = np.random.default_rng(seed * 4099 + night); s, d = RUG_SHIFT
    dx, dy = np.clip(r.normal(0, s, 2), -3 * s, 3 * s); yaw = float(np.clip(r.normal(0, d), -3 * d, 3 * d))
    return {**rug, "x": round(rug["x"] + float(dx), 4), "y": round(rug["y"] + float(dy), 4), "yaw_deg": round(yaw, 3)}


def rug_corners(rug):
    c, s_ = math.cos(math.radians(rug.get("yaw_deg", 0.0))), math.sin(math.radians(rug.get("yaw_deg", 0.0)))
    return [(rug["x"] + c * u - s_ * v, rug["y"] + s_ * u + c * v) for u, v in
            ((-rug["w"] / 2, -rug["h"] / 2), (rug["w"] / 2, -rug["h"] / 2), (rug["w"] / 2, rug["h"] / 2), (-rug["w"] / 2, rug["h"] / 2))]


def carpet_rects():
    """the carpeted areas as world rectangles (x0, y0, x1, y1)"""
    out = []
    for x0, y0, x1, y1 in mh.CARPET:
        (ax, ay), (bx, by) = mh.cell_to_world(x0, y0), mh.cell_to_world(x1, y1)
        out.append((max(min(ax, bx) - mh.CELL / 2, -HX), max(min(ay, by) - mh.CELL / 2, -HY),
                    min(max(ax, bx) + mh.CELL / 2, HX), min(max(ay, by) + mh.CELL / 2, HY)))
    return out


def slab_obj(path, corners, z0, z1):
    """a flat slab over a floor polygon (4 corners, counter-clockwise), textured with the floor's own image by world
    position (u, v across the whole floor, as the floor plane maps it): its top shows exactly the floor beneath"""
    uv = lambda x, y: ((x + HX) / (2 * HX), (y + HY) / (2 * HY))
    lines = [f"v {x:.5f} {y:.5f} {z:.5f}" for z in (z0, z1) for x, y in corners]
    lines += [f"vt {u:.6f} {v:.6f}" for x, y in corners for u, v in [uv(x, y)]]
    t = lambda i: f"{i + 5}/{i + 1}"; b = lambda i: f"{i + 1}/{i + 1}"
    lines += [f"f {t(0)} {t(1)} {t(2)}", f"f {t(0)} {t(2)} {t(3)}", f"f {b(0)} {b(2)} {b(1)}", f"f {b(0)} {b(3)} {b(2)}"]
    for i in range(4):
        j = (i + 1) % 4; lines += [f"f {b(i)} {b(j)} {t(j)}", f"f {b(i)} {t(j)} {t(i)}"]
    Path(path).write_text("\n".join(lines) + "\n")


def carpet_m(seed):
    """how far this house's carpet stands above its hard floor"""
    return LOW_PILE_M if seed in LOW_PILE_HOUSES else CARPET_M


def relief_xml(name, rug, seed=0):
    """the carpet and tonight's rug as raised slabs of the floor's own texture ([nAH] TF8)"""
    assets, geoms = [], []; cm = carpet_m(seed)
    parts = [(f"carpet{i}", [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], 0.0, cm) for i, (x0, y0, x1, y1) in enumerate(carpet_rects())]
    if rug:
        on_carpet = any(x0 <= rug["x"] <= x1 and y0 <= rug["y"] <= y1 for x0, y0, x1, y1 in carpet_rects())
        base = cm if on_carpet else 0.0
        parts.append(("rug", rug_corners(rug), base, base + RUG_M))
    for part, corners, z0, z1 in parts:
        f = TEX_DIR / f"relief_{name}_{part}.obj"; slab_obj(f, corners, z0, z1)
        assets.append(f'    <mesh name="relief_{part}" file="{f}"/>')
        geoms.append(f'    <geom name="relief_{part}" type="mesh" mesh="relief_{part}" material="housefloor" contype="1" conaffinity="0" friction="1.0"/>')
    return "\n".join(assets), "\n".join(geoms)


def floor_material(pbr):
    """the house floor's material: its colour, and for a photographed floor its normal and ORM layers (Filament shades
    with them; the classic renderer ignores them)"""
    if pbr is None:
        return '    <material name="housefloor" texture="housefloor" texrepeat="1 1" texuniform="false"/>'
    n, o = pbr
    return (f'    <texture type="2d" name="housefloor_n" file="{n}" colorspace="linear"/>\n'
            f'    <texture type="2d" name="housefloor_orm" file="{o}" colorspace="linear"/>\n'
            '    <material name="housefloor" texrepeat="1 1" texuniform="false" metallic="0" roughness="1"><layer texture="housefloor" role="rgb"/>'
            '<layer texture="housefloor_n" role="normal"/><layer texture="housefloor_orm" role="orm"/></material>')


def scene_xml(name, tex_name, light, items, g, chairs=(), near=(0.0, 0.0), seed=0, pbr=None, rug=None, relief=False):
    tr, tg, tb = TINT[light["tint"]]
    amb = " ".join(f"{light['ambient'] * c:.2f}" for c in (tr, tg, tb))
    indoor = "regime" in light                                 # --indoor: ceiling, windows, furniture on legs ([QMt])
    geoms, windows = walls_xml(g) if indoor else ([mh.box_geom(f"wall_{i}", *r, mh.WALL_H, None, "wall") for i, r in enumerate(mh.wall_rects(g))], [])
    geoms += furniture_xml() if indoor else [mh.box_geom(f"furn_{f[5]}", *f[:4], f[4], None, "furniture") for f in mh.FURNITURE]
    plan = mh.DEMO_PLANS.get(mh.LAYOUT_SEED) or {}             # a demo room's own chairs, where they stand (subsystem-demos [qEP])
    chairs = list(chairs) + [dict(id=i, x=x, y=y, yaw=yaw, w=.44, d=.44, h=.45, back_h=.90, rgba="0.36 0.27 0.20 1", mode="tucked", moved_m=None)
                             for i, x, y, yaw in plan.get("chairs", [])]
    chair_xml = [mv.xml(c) for c in chairs]                    # (before surfaces_xml writes the meshes: the chairs' are among them)
    r_assets, r_geoms = relief_xml(name, rug, seed) if relief else ("", "")
    return f'''<?xml version="1.0" ?>
<mujoco model="scene_{name}">
  <compiler angle="radian"/>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.9 0.92 0.95" rgb2="0.6 0.65 0.7" width="256" height="256"/>
    <texture type="2d" name="housefloor" file="{tex_name}"/>
{floor_material(pbr)}
{surfaces_xml(seed) if indoor else '    <material name="wall" rgba="0.93 0.91 0.87 1"/>'}
{r_assets}
    <material name="furniture" rgba="0.45 0.38 0.32 1"/>
    <material name="ceiling" rgba="0.96 0.95 0.93 1"/>
    <material name="trim" rgba="0.95 0.95 0.93 1" specular="0.3" shininess="0.5"/>
    <material name="blinds" rgba="0.93 0.92 0.89 1"/>
    <material name="screen" rgba="0.02 0.02 0.025 1" specular="0.8" shininess="0.9"/>
    <material name="outdoors" rgba="{'0.92 0.94 1.0 1" emission="' + str(OUTDOOR_DAY) if light.get("regime") == "day" else '0.02 0.02 0.03 1" emission="0'}"/>
  </asset>
  <worldbody>
{lights_xml(light, near, windows)}    <geom name="floor" type="plane" size="{HX} {HY} 0.1" material="housefloor" contype="1" conaffinity="0" priority="1" friction="1.0"/>
{r_geoms}
{chr(10).join(geoms)}
{chr(10).join(item_xml(it) for it in items)}
{chr(10).join(chair_xml)}
  </worldbody>
  <visual>
    <headlight diffuse="{amb}" ambient="{amb}" specular="0.05 0.05 0.05"/>
  </visual>
</mujoco>
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=7); ap.add_argument("--nights", type=int, default=6)
    ap.add_argument("--per-night", type=int, default=16); ap.add_argument("--carry", type=float, default=0.3)
    ap.add_argument("--start-night", type=int, default=1, help="append nights after an existing timeline")
    ap.add_argument("--rug-night", type=int, default=RUG_NIGHT, help="night the rug is first down (1: in the baseline; later: the floor-change test)")
    ap.add_argument("--varied-from", type=int, default=5, help="night from which litter appearance varies")
    ap.add_argument("--movables", action="store_true", help="chairs/stool that move every night (not pick-up targets)")
    ap.add_argument("--out-dir", default=None, help="write the nights' truth files here instead of make_house's runs folder (an experiment's own baselines, night_tidy.sh RUN_DIR)")
    ap.add_argument("--tag", default="", help="a private copy of the house under its own names (e.g. _bench): the nights' files untouched")
    ap.add_argument("--pets", action="store_true", help="a pet's food and water bowls by the kitchen, nudged each night (never picked up)")
    ap.add_argument("--flat", dest="relief", action="store_false", help="one flat floor: no raised carpet or rug, the rug never moves ([nAH] TF8)")
    ap.add_argument("--indoor", action="store_true", help="a ceiling and windows, lit by day (sun through the windows) or night (lamp and ceiling light), [CaM]")
    ap.add_argument("--regime", choices=["day", "night", "lit"], default=None, help="every night lit as this (default: each night draws day or night)")
    ap.add_argument("--photo-floors", action="store_true", help="a photographed floor at 3 mm a pixel, with relief ([QMt]); needs tools/fetch_floors.sh")
    a = ap.parse_args()
    global PX
    if a.photo_floors:
        PX = PHOTO_PX; sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "arm"))
    mh.use_layout(a.seed)                                   # this house's floor plan ([uCP]): walls, furniture, dock, rug
    rng = random.Random(a.seed); g = mh.base_grid()
    T = a.tag                                              # a private copy of the house (the bench's), beside the nights'
    out = Path(a.out_dir) if a.out_dir else mh.OUT_DIR / f"nights{T}_s{a.seed}"; out.mkdir(parents=True, exist_ok=True)   # make_house's runs folder: one
                                                           # place to point a whole generation elsewhere (benchmark-release [PiV])
    stains = []
    for k in range(5):                                   # permanent floor marks, fixed for the whole timeline
        reg = mh.REGIONS[k % len(mh.REGIONS)]
        while True:
            x, y = rng.randint(reg[0], reg[2]), rng.randint(reg[1], reg[3])
            if not g[y][x]: break
        wx, wy = mh.cell_to_world(x, y)
        stains.append({"id": f"stain_{k}", "x": round(wx, 3), "y": round(wy, 3), "r": round(rng.uniform(.06, .14), 3)})
    rug = dict(mh.RUG, id="rug")
    carried, chairs = [], None
    if a.start_night > 1:                                   # continue from the last existing night
        prev = json.loads((out / f"nights{T}_s{a.seed}_n{a.start_night - 1}.truth.json").read_text())
        rng = random.Random(a.seed * 100 + a.start_night)
        stains = prev["stains"] + [{"id": "stain_1", "x": 0, "y": 0, "r": 0}]   # stain_1 stays cleaned
        carried = [{k: v for k, v in it.items() if k != "new_tonight"} for it in prev["items"] if rng.random() < a.carry]
        chairs = prev.get("movables")
    for n in range(a.start_night, a.nights + 1):
        present_stains = [s for s in stains if not (s["id"] == "stain_1" and n >= CLEAN_NIGHT)]
        rug_n = rug_tonight(rug, a.seed, n) if a.photo_floors and a.relief else rug   # shifted a little each night
        tex_name, pbr = f"house_floor{T}_s{a.seed}_n{n}.png", None
        if a.photo_floors:
            tex, tn, to = photo_floor_texture(random.Random(a.seed * 1000 + n), present_stains, rug_n if n >= a.rug_night else None, a.seed)
            pbr = (f"house_floor{T}_s{a.seed}_n{n}_n.png", f"house_floor{T}_s{a.seed}_n{n}_orm.png")
            half = (tn.width // 2, tn.height // 2)          # relief at 6 mm: a quarter of the GPU memory, and finer than
            tn.resize(half, Image.BILINEAR).save(TEX_DIR / pbr[0], compress_level=1)   # the head camera resolves it
            to.resize(half, Image.BILINEAR).save(TEX_DIR / pbr[1], compress_level=1)
        else:
            tex = floor_texture(random.Random(a.seed * 1000 + n), present_stains, rug if n >= a.rug_night else None, a.seed)
        tex.save(TEX_DIR / tex_name, compress_level=1 if a.photo_floors else 6)
        taken = set()
        if a.movables:
            chairs = mv.place(random.Random(a.seed * 7919 + n), g, chairs); taken |= mv.taken_cells(chairs)
        items = [dict(it) for it in carried]
        for it in items:
            taken.add(tuple(it["cell"]))
        maker = new_item_varied if n >= a.varied_from else new_item
        items += [maker(rng, i, n, g, taken) for i in range(a.per_night)]
        light = lighting(rng)                                 # (drawn as before, so items and every later draw stay put)
        light = {**light, **regime(a.seed, n, a.regime)} if a.indoor else light
        name = f"nights{T}_s{a.seed}_n{n}"
        (mh.DIMOS_DATA / f"scene_{name}.xml").write_text(scene_xml(name, tex_name, light, items, g, chairs if a.movables else (),
                                                                    near=mh.cell_to_world(*mh.START), seed=a.seed, pbr=pbr,
                                                                    rug=rug_n if n >= a.rug_night else None, relief=a.photo_floors and a.relief))   # one house, its nights
        sx, sy = mh.cell_to_world(*mh.START)
        truth = {"room": name, "night": n, "start_pos": [round(sx, 3), round(sy, 3)], "lighting": light,
                 "stains": present_stains, "stain_cleaned": ["stain_1"] if n >= CLEAN_NIGHT else [],
                 "rug": rug_n if n >= a.rug_night else None, "rug_new_tonight": n == a.rug_night > 1,
                 "relief_m": {"carpet": carpet_m(a.seed), "rug": RUG_M} if a.photo_floors and a.relief else None,
                 "items": [{**it, "new_tonight": it["first_night"] == n} for it in items],
                 "movables": chairs if a.movables else [],
                 "fixtures": pet_bowls_tonight(a.seed, n) if a.pets else []}
        (out / f"{name}.truth.json").write_text(json.dumps(truth, indent=1))
        carried = [it for it in items if rng.random() < a.carry]      # some toys are left out for another night
        kinds = {}
        for it in items: kinds[it["kind"]] = kinds.get(it["kind"], 0) + 1
        print(f"night {n}: {len(items)} items {kinds} carried_in={sum(not t['new_tonight'] for t in truth['items'])} "
              f"stains={len(present_stains)} rug={'yes' if truth['rug'] else 'no'} light={light['sun']}/{light['ambient']} {light['tint']} lamp={light['lamp_on']}")
    print(f"truth: {out}")


if __name__ == "__main__":
    main()
