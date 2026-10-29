"""Local floors at wrist-camera resolution (grasp spec [N4z]): a patch covering the arm's reach, generated from a seed
each trial, so the look-closer step is tuned against many floors rather than one photo.

The wrist camera at ~32 cm resolves ~0.6 mm/px; the whole-floor carpet texture is 5 mm/px, a smear at that range. This
patch is 1.3 m square at 2048 px (0.63 mm/px), laid just above the coarse floor under the robot. Floors: cut pile, loop
pile (berber), patterned rug, wood planks, tile. Distractors: walked-on patches (pile brightness), stains, scuff marks,
lint specks. All sizes and colour ranges here are estimates of ordinary homes, not measurements.

Every floor repeats seamlessly (noise wraps; tile, plank and pattern spacings fit a whole number of times into the
patch; stains wrap round its edges), so the surround cells repeat it by translation with no visible joins ([CaM]).
A rug lies on another floor: the patch is that floor (so it meets the surround seamlessly), with a rug of its own size
drawn on it, patterned, bordered and edged with a faint shadow, so it reads as a rug and not as a texture seam."""
import csv, functools, io, zipfile
from pathlib import Path
import numpy as np, mujoco, cv2

PX = 2048; SIZE_M = 1.3; MPP = SIZE_M / PX
SHADE_FADE_M = 0.2               # view-dependent pile shading fades out over the patch's last 20 cm: the surround is not
                                 # re-shaded per camera, and a shaded patch read as a lighter or darker square
SUR_PX = PX                      # surround cells: the patch at full resolution, repeated (it tiles seamlessly); at 2.5 mm/px
                                 # the 1 mm pile speckle vanished, and the patch read as a speckled square with a straight edge
SUR_CELLS = [(i, j) for i in range(-3, 4) for j in range(-3, 4) if (i, j) != (0, 0)]   # 9.1 m square around the patch
FLOORS = ("cut_pile", "loop_pile", "wood", "tile")
KINDS = FLOORS + ("rug",)        # "rug": any floor with a rug on it
RUG_P = 0.4                      # share of floors with a rug on them (est.; many rooms have one)
PHOTO_CSV = Path(__file__).resolve().parent / "floor_photos.csv"          # photographed floors: size, kind, train/held out
PHOTO_ZIPS = Path(__file__).resolve().parents[2] / "models" / "textures" / "floors" / "zips"   # tools/fetch_floors.sh
SPLIT = "train"                  # which photographed floors to draw from; "heldout" only for judging (eval_policy --floors)
SCALE_JITTER = 0.15              # each trial rescales the photo by up to this much either way (est.; floors differ)
FLAT_N = np.array([128, 128, 255], np.uint8)                              # a normal map pixel for "straight up"
NX_SIGN, NY_SIGN = 1, 1          # how world +x, +y tilts are written into the normal map's R, G (set by tests/test_floor_photos.py)


def photos(split=None, kinds=None):
    """the photographed floors of one split (SPLIT unless given), optionally only some kinds (carpet, rug, wood, tile)"""
    rows = [r for r in csv.DictReader(l for l in open(PHOTO_CSV) if not l.startswith("#"))]
    return [r for r in rows if r["split"] == (split or SPLIT) and (kinds is None or r["kind"] in kinds)]


@functools.lru_cache(maxsize=12)
def _photo_maps(fid):
    """one photographed floor's maps, as an array laid out like every image here (column +x, row +y): colour 0-1, the
    surface normal in world axes (nx, ny, nz), roughness 0-1, ambient occlusion 0-1"""
    z = zipfile.ZipFile(PHOTO_ZIPS / f"{fid}.zip")
    def img(suffix, mode):
        n = [x for x in z.namelist() if x.endswith(suffix)]
        return np.asarray(__import__("PIL.Image", fromlist=["Image"]).open(io.BytesIO(z.read(n[0]))).convert(mode)) if n else None
    col = img("_Color.jpg", "RGB").astype(np.float32) / 255
    nm = img("_NormalGL.jpg", "RGB").astype(np.float32) / 127.5 - 1         # OpenGL normal map: R toward image right,
    nrm = np.stack([nm[..., 0], -nm[..., 1], nm[..., 2]], -1)              # G toward image up (= row -1 here, so -y)
    r = img("_Roughness.jpg", "L"); ao = img("_AmbientOcclusion.jpg", "L")
    rough = r.astype(np.float32) / 255 if r is not None else np.full(col.shape[:2], 0.8, np.float32)
    occ = ao.astype(np.float32) / 255 if ao is not None else np.ones(col.shape[:2], np.float32)
    return col, nrm, rough, occ


def _dihedral(rng, maps):
    """turn and flip the maps together (any of the 8 ways a square can lie), rotating the normals with them"""
    col, nrm, rough, occ = maps
    if rng.random() < 0.5:                                                  # transpose: x and y swap
        col, rough, occ = col.transpose(1, 0, 2), rough.T, occ.T; nrm = nrm.transpose(1, 0, 2)[..., [1, 0, 2]]
    if rng.random() < 0.5:                                                  # mirror in x
        col, rough, occ = col[:, ::-1], rough[:, ::-1], occ[:, ::-1]; nrm = nrm[:, ::-1] * np.array([-1, 1, 1], np.float32)
    if rng.random() < 0.5:                                                  # mirror in y
        col, rough, occ = col[::-1], rough[::-1], occ[::-1]; nrm = nrm[::-1] * np.array([1, -1, 1], np.float32)
    return col, nrm, rough, occ


def _photo_patch(rng, row):
    """one photographed floor laid over the patch at its real size (x a trial's jitter), a whole number of times so the patch
    still repeats seamlessly, turned and flipped: (albedo, normals, roughness, occlusion), each PX square"""
    size = float(row["size_m"]) * rng.uniform(1 - SCALE_JITTER, 1 + SCALE_JITTER)
    n = max(1, round(SIZE_M / size))                                        # copies across the patch (the size moves to fit)
    col, nrm, rough, occ = _dihedral(rng, _photo_maps(row["id"]))
    def lay(a, interp=cv2.INTER_AREA):
        one = cv2.resize(np.ascontiguousarray(a), (PX // n + 1, PX // n + 1), interpolation=interp)
        t = np.tile(one, (n, n) + (1,) * (a.ndim - 2))
        return cv2.resize(t, (PX, PX), interpolation=interp)              # the whole tiling rescaled: it still repeats
    nrm = lay(nrm); nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True) + 1e-6
    col = lay(col) * rng.uniform(0.9, 1.1)                                  # exposure-like variation (est.)
    return col, nrm, lay(rough), lay(occ)


@functools.lru_cache(maxsize=2)
def _photo_patch_cached(seed, fid):
    """the trial's photographed patch, made once: the surround repeats it (without the marks), so it asks again"""
    row = next(r for split in ("train", "heldout") for r in photos(split) if r["id"] == fid)
    return _photo_patch(np.random.default_rng(seed + 37), row)


def _encode(nrm, rough, occ):
    """normals and roughness/occlusion as the textures MuJoCo's normal and ORM layers take"""
    n = nrm * np.array([NX_SIGN, NY_SIGN, 1], np.float32)
    nrgb = np.clip((n * 0.5 + 0.5) * 255 + 0.5, 0, 255).astype(np.uint8)
    orm = np.stack([occ, rough, np.zeros_like(rough)], -1)
    return nrgb, np.clip(orm * 255 + 0.5, 0, 255).astype(np.uint8)


def _noise(rng, cells, px=PX):
    """smooth noise with features about px/cells wide, zero mean, unit-ish std; repeats seamlessly every px"""
    n = np.tile(rng.normal(0, 1, (cells, cells)).astype(np.float32), (3, 3))          # three periods, resized, middle kept:
    return cv2.resize(n, (3 * px, 3 * px), interpolation=cv2.INTER_CUBIC)[px:2 * px, px:2 * px]   # its edges match


def _wblur(img, sigma):
    """Gaussian blur that wraps round the edges (so the result still tiles)"""
    p = int(4 * sigma) + 1
    return cv2.GaussianBlur(np.pad(img, p, mode="wrap"), (0, 0), sigma)[p:-p, p:-p]


def _fit(period):
    """the nearest period that fits a whole number of times into the patch"""
    return SIZE_M / max(1, round(SIZE_M / period))


def _wrap(d):
    """offsets on the patch taken the short way round, as if it repeated"""
    return (d + SIZE_M / 2) % SIZE_M - SIZE_M / 2


def _octaves(rng, sizes_m, weights):
    return sum(w * _noise(rng, max(2, int(SIZE_M / s))) for s, w in zip(sizes_m, weights))


def _hsv(h, s, v):
    c = cv2.cvtColor(np.uint8([[[h * 180, s * 255, v * 255]]]), cv2.COLOR_HSV2RGB)[0, 0]
    return c.astype(np.float32) / 255


def _base_colour(rng, kind):
    if kind == "wood": return _hsv(rng.uniform(0.05, 0.1), rng.uniform(0.35, 0.7), rng.uniform(0.3, 0.75))
    if rng.random() < 0.7:                                                  # most carpets and tiles are neutrals
        return _hsv(rng.uniform(0.04, 0.14), rng.uniform(0.05, 0.3), rng.uniform(0.35, 0.85))
    return _hsv(rng.random(), rng.uniform(0.15, 0.55), rng.uniform(0.25, 0.75))


def _pile(rng, base, fleck=0.0):
    """carpet pile: fibre-tip speckle at ~1 mm, tufts at a few mm, pile variation at cm scale"""
    speck = _wblur(rng.normal(0, 1, (PX, PX)).astype(np.float32), 0.8)
    tone = 1 + 0.10 * speck + 0.05 * _octaves(rng, (0.004, 0.02, 0.1), (0.6, 0.3, 0.3))
    img = base[None, None] * tone[..., None]
    if fleck > 0:                                                           # berber-style coloured flecks
        f = (_wblur(rng.random((PX, PX)).astype(np.float32), 1.5) > 0.5 + 0.05 * (1 - fleck))
        img[f] = img[f] * rng.uniform(0.5, 1.4, 3)
    return img


def _rug(img, rrng, focus, xx, yy):
    """a rug lying on the floor: its own size (40-100 cm), turn, pattern, contrasting border and a thin edge shadow. With
    a focus (the item, patch metres), 7 rugs in 10 have an edge within 15 cm of it, either side, at any angle, so the
    wrist camera meets a sharp edge near what it is looking for without it always lining up; the rest lie anywhere"""
    near_item = bool(rrng.random() < 0.7)                                         # 30%: anywhere, not keyed to the item
    for _ in range(40):
        rw, rh = rrng.uniform(0.4, 1.0, 2); th = rrng.uniform(0, np.pi); ct, st = np.cos(th), np.sin(th)
        ax = np.array([[ct, st], [-st, ct]])                                          # rows: the rug's own x and y axes
        if focus is None or near_item is False: c0 = rrng.uniform(0.3, SIZE_M - 0.3, 2)
        else:
            k = rrng.integers(4); n = ax[k % 2] * (1 if k < 2 else -1); h = (rw if k % 2 == 0 else rh) / 2   # an edge and its
            tang = ax[1 - k % 2]; along = (rh if k % 2 == 0 else rw) / 2                                   # outward normal
            c0 = np.asarray(focus) - n * (h + rrng.uniform(-0.15, 0.15)) - tang * rrng.uniform(-0.7, 0.7) * along
        corners = c0 + np.array([[sx * rw / 2, sy * rh / 2] for sx in (-1, 1) for sy in (-1, 1)]) @ ax
        if corners.min() >= 0.02 and corners.max() <= SIZE_M - 0.02: break         # wholly on the patch (the surround has none)
    rx = (xx - c0[0]) * ct + (yy - c0[1]) * st; ry = -(xx - c0[0]) * st + (yy - c0[1]) * ct
    ex, ey = np.abs(rx), np.abs(ry); rugm = (ex < rw / 2) & (ey < rh / 2); edge = np.minimum(rw / 2 - ex, rh / 2 - ey)
    sat = rrng.uniform(0.1, 0.6)                                                  # most rugs are muted; some are bold (est.)
    cols = [_hsv(rrng.random(), sat * rrng.uniform(0.6, 1.2), rrng.uniform(0.25, 0.8)) for _ in range(rrng.integers(2, 5))]
    style = rrng.integers(3); period = rrng.uniform(0.06, 0.2)
    if style == 0: idx = np.floor(rx / period) % len(cols)                                      # stripes
    elif style == 1: idx = (np.floor(rx / period) + np.floor(ry / period)) % len(cols)          # checks
    else: idx = np.floor(np.maximum(ex / rw, ey / rh) * 2 * rrng.integers(3, 7) + 0.1 * _noise(rrng, 8)) % len(cols)   # rings
    rug = np.stack(cols)[idx.astype(int)]
    bw = rrng.uniform(0.03, 0.08); rug[edge < bw] = _hsv(rrng.random(), rrng.uniform(0.3, 0.7), rrng.uniform(0.15, 0.5))   # border
    rug = rug * _pile(rrng, np.ones(3, np.float32)); out = img.copy(); out[rugm] = rug[rugm]
    shadow = (~rugm) & (ex < rw / 2 + 0.004) & (ey < rh / 2 + 0.004); out[shadow] *= 0.7            # thin shadow at its edge
    return out, rugm


def generate(seed, kind=None, marks=True, rug=None, focus=None):
    """a floor for this seed: (albedo, label, pile direction map or None, sheen). kind picks the floor ("rug" = any floor
    with a rug); rug=None lets the seed decide (RUG_P), False leaves it out; focus (patch metres) is where the item is,
    and a rug's edge is laid through its neighbourhood. marks=False leaves out stains, scuffs and lint (the surround
    repeats the patch, and repeated stains would show as a grid). The same seed gives the same floor whatever else is
    asked for: the rug and the floor type draw from their own streams."""
    rng = np.random.default_rng(seed); pick = rng.integers(len(FLOORS))          # always drawn, so the stream never shifts
    if kind == "rug": kind, rug = None, True
    rrng = np.random.default_rng(seed + 7919); has_rug = bool(rrng.random() < RUG_P) if rug is None else bool(rug)
    yy, xx = np.mgrid[0:PX, 0:PX].astype(np.float32) * MPP
    pbr = None                                                              # (normals, roughness, occlusion) of a photographed floor
    if kind is None or kind.startswith("photo"):                            # photographed ([N4z]): "photo", "photo_wood", "photo:<id>"
        if kind and kind.startswith("photo:"):
            row = next(r for split in ("train", "heldout") for r in photos(split) if r["id"] == kind[6:])
        else:
            want = None if kind in (None, "photo") else [kind.split("_", 1)[1]]
            rows = photos(kinds=want or ["carpet", "wood", "tile"]); row = rows[np.random.default_rng(seed + 31).integers(len(rows))]
        img, *pbr = [a.copy() for a in _photo_patch_cached(seed, row["id"])]; kind = f"{row['kind']}:{row['id']}"
    else:
        base = _base_colour(rng, kind)
    if pbr is not None: pass
    elif kind == "cut_pile":
        img = _pile(rng, base)
    elif kind == "loop_pile":
        p = _fit(rng.uniform(0.004, 0.007)); q = _fit(p * 1.3); u, v = (xx, yy) if rng.random() < 0.5 else (yy, xx)   # rows along x or y
        loops = 0.5 + 0.5 * np.sin(2 * np.pi * u / p) * np.sin(2 * np.pi * v / q)
        img = _pile(rng, base, fleck=0.6) * (0.85 + 0.3 * loops[..., None])
    elif kind == "wood":
        w = _fit(rng.uniform(0.08, 0.15)); L = _fit(rng.uniform(0.6, 1.2)); u, v = (xx, yy) if rng.random() < 0.5 else (yy, xx)
        n_pl, n_seg = round(SIZE_M / w), round(SIZE_M / L); plank = np.floor(v / w) % n_pl
        shift = (plank * 0.37) % 1.0 * L; seg_ = np.floor((u + shift) / L) % n_seg      # planks and their joints repeat with the patch
        prng = np.random.default_rng(seed + 1); tint = prng.uniform(0.8, 1.2, 4096)
        grain = np.sin(2 * np.pi * (v / _fit(0.004) + 3 * _noise(rng, 20))) * 0.06 + 0.04 * _noise(rng, 200)
        seam = ((v % w) < 0.0015) | (((u + shift) % L) < 0.0015)
        img = base[None, None] * (tint[((plank * 37 + seg_) % 4096).astype(int)] + grain)[..., None]; img[seam] *= 0.45
    else:                                                                   # tile
        t = _fit(rng.choice([0.3, 0.45, 0.6])); g = rng.uniform(0.003, 0.006); nt = round(SIZE_M / t)
        grout = ((xx % t) < g) | ((yy % t) < g); tid = ((np.floor(xx / t) % nt) * 13 + np.floor(yy / t) % nt).astype(int)
        tint = np.random.default_rng(seed + 2).uniform(0.93, 1.07, 4096)[tid % 4096]
        img = base[None, None] * (tint + 0.03 * _octaves(rng, (0.002, 0.03), (0.5, 0.5)))[..., None]
        img[grout] = _hsv(rng.uniform(0, 0.15), 0.1, rng.uniform(0.3, 0.8))
    rug_mask = None
    if has_rug and pbr is not None:
        drawn, rug_mask = _rug(img.copy(), rrng, focus, xx, yy)
        rows = photos(kinds=["rug", "carpet"]); rrow = rows[rrng.integers(len(rows))]
        rcol, rn, rr, ro = _photo_patch(np.random.default_rng(seed + 41), rrow)
        edge = np.where(rug_mask, 1.0, np.clip(drawn.mean(-1) / np.maximum(img.mean(-1), 1e-3), 0.3, 1.0))   # its shadow on the floor
        img = np.where(rug_mask[..., None], rcol, img * edge[..., None])
        pbr = [np.where(rug_mask[..., None], rn, pbr[0]), np.where(rug_mask, rr, pbr[1]), np.where(rug_mask, ro, pbr[2])]
        kind = f"{kind}+rug:{rrow['id']}"
    elif has_rug: img, rug_mask = _rug(img, rrng, focus, xx, yy)
    nap = None
    if kind in ("cut_pile", "loop_pile"):                                   # pile lay: one main direction, walked-on patches flipped
        main = rng.uniform(0, 2 * np.pi); walked = _noise(rng, 5) > rng.uniform(0.3, 0.9)
        nap = (main + 0.25 * _noise(rng, 12) + np.pi * walked).astype(np.float32)
        img *= (1 + rng.uniform(0.0, 0.04) * np.tanh(2 * _noise(rng, 5)))[..., None]
    if has_rug and nap is None: nap = (rrng.uniform(0, 2 * np.pi) + 0.25 * _noise(rrng, 12)).astype(np.float32)   # the rug's lay
    for _ in range(rng.integers(0, 4) if marks else 0):                     # stains: irregular, a few to 15 cm, darker or tinted,
        c = rng.uniform(0.25, SIZE_M - 0.25, 2); r = rng.uniform(0.02, 0.08)   # clear of the edges (the surround has none)
        blob = (np.hypot(_wrap(xx - c[0]), _wrap(yy - c[1])) + 0.3 * r * _noise(rng, 30)) < r
        img[blob] *= rng.uniform(0.6, 0.9) * rng.uniform(0.85, 1.1, 3)
    for _ in range(rng.integers(0, 4) if marks else 0):                     # scuffs: thin darker strokes, clear of the edges
        p0 = rng.uniform(0.25, SIZE_M - 0.25, 2); a = rng.uniform(0, np.pi); L = rng.uniform(0.03, 0.2)
        ex, ey = _wrap(xx - p0[0]), _wrap(yy - p0[1]); d_ = np.abs(-ex * np.sin(a) + ey * np.cos(a)); along = ex * np.cos(a) + ey * np.sin(a)
        img[(d_ < 0.0012) & (along > 0) & (along < L)] *= 0.7
    n_lint = rng.integers(0, 40) if marks else 0                            # lint and crumbs: 1-3 mm specks, below what we pick
    for x_, y_, r_ in zip(rng.integers(8, PX - 8, n_lint), rng.integers(8, PX - 8, n_lint), rng.integers(1, 4, n_lint)):
        cv2.circle(img, (int(x_), int(y_)), int(r_), tuple(float(v) for v in rng.uniform(0.2, 1.0, 3)), -1)
    sheen = {"cut_pile": 0.25, "loop_pile": 0.12}.get(kind, 0.0) * rng.uniform(0.5, 1.2)   # strength est.; calibrate from real photo pairs
    if has_rug: sheen = np.where(rug_mask, 0.2 * rrng.uniform(0.5, 1.2), sheen).astype(np.float32)   # the rug's own pile
    if pbr is None: kind = kind + "+rug" if has_rug else kind
    return np.clip(img, 0, 1).astype(np.float32), kind, nap, sheen, None if pbr is None else _encode(*pbr)


_NAPCS = {}


def shade(albedo, nap, sheen, cam_pos, centre_xy, coarse=256):
    """what the camera sees from cam_pos: pile reads brighter looking against its lay and darker looking with it, more so
    at low angles; straight down it barely changes. The view terms vary smoothly across the patch, so they are worked
    out on a coarse grid and enlarged (fast enough for every recorded frame); the pile direction stays full resolution.
    The effect fades out over the patch's last SHADE_FADE_M (the surround is not re-shaded)."""
    if nap is None or not np.any(sheen): return albedo
    key = id(nap)
    if _NAPCS.get("key") != key:                                                   # per floor: pile direction and edge fade
        g = (np.arange(PX, dtype=np.float32) + 0.5) * MPP - SIZE_M / 2
        edge = np.minimum(SIZE_M / 2 - np.abs(g)[None, :], SIZE_M / 2 - np.abs(g)[:, None]); t_ = np.clip(edge / SHADE_FADE_M, 0, 1)
        w = sheen * t_ ** 2 * (3 - 2 * t_)
        _NAPCS.update(key=key, c=(np.cos(nap) * w).astype(np.float32), s=(np.sin(nap) * w).astype(np.float32))
    gc = (np.arange(coarse, dtype=np.float32) + 0.5) * (SIZE_M / coarse) - SIZE_M / 2
    X = centre_xy[0] + gc[None, :]; Y = centre_xy[1] + gc[:, None]                 # texel world positions (row = y)
    dx = cam_pos[0] - X; dy = cam_pos[1] - Y; horiz = np.hypot(dx, dy) + 1e-9
    tilt = horiz / np.hypot(horiz, cam_pos[2])                                      # sin of the view's angle from vertical
    A = cv2.resize((tilt * dx / horiz).astype(np.float32), (PX, PX), interpolation=cv2.INTER_LINEAR)
    B = cv2.resize((tilt * dy / horiz).astype(np.float32), (PX, PX), interpolation=cv2.INTER_LINEAR)
    f = 1 + A * _NAPCS["c"] + B * _NAPCS["s"]                                       # tilt * cos(view direction - pile direction)
    return albedo * f[..., None]


def _pbr_layers(spec, mt, name, px):
    """the material's normal and ORM layers (filled per trial; flat and matte until then). The classic renderer ignores
    them; Filament shades with them ([UQK])"""
    for suffix, role, rgb in (("_n", mujoco.mjtTextureRole.mjTEXROLE_NORMAL, [0.5, 0.5, 1.0]),
                              ("_orm", mujoco.mjtTextureRole.mjTEXROLE_ORM, [1.0, 0.8, 0.0])):
        t = spec.add_texture(); t.name = name + suffix; t.type = mujoco.mjtTexture.mjTEXTURE_2D; t.builtin = mujoco.mjtBuiltin.mjBUILTIN_FLAT
        t.width = px; t.height = px; t.rgb1 = rgb; t.nchannel = 3; mt.textures[role] = name + suffix


def add_to_spec(spec):
    """a floor patch geom with its own 2048 px texture; place and fill it with apply()"""
    t = spec.add_texture(); t.name = "local_floor"; t.type = mujoco.mjtTexture.mjTEXTURE_2D
    t.builtin = mujoco.mjtBuiltin.mjBUILTIN_FLAT; t.width = PX; t.height = PX; t.rgb1 = [0.5, 0.5, 0.5]; t.nchannel = 3
    mt = spec.add_material(); mt.name = "local_floor"; mt.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "local_floor"
    mt.texrepeat = [1, 1]; mt.texuniform = False; _pbr_layers(spec, mt, "local_floor", PX)
    g = spec.worldbody.add_geom(); g.name = "local_floor"; g.type = mujoco.mjtGeom.mjGEOM_PLANE; g.size = [SIZE_M / 2, SIZE_M / 2, 0.01]
    g.pos = [0, 0, 0.001]; g.material = "local_floor"; g.contype = 0; g.conaffinity = 0
    t = spec.add_texture(); t.name = "surround"; t.type = mujoco.mjtTexture.mjTEXTURE_2D      # the same floor out to everything
    t.builtin = mujoco.mjtBuiltin.mjBUILTIN_FLAT; t.width = SUR_PX; t.height = SUR_PX; t.rgb1 = [0.5, 0.5, 0.5]; t.nchannel = 3   # a camera
    mt = spec.add_material(); mt.name = "surround"; mt.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "surround"                # can see
    mt.texrepeat = [1, 1]; mt.texuniform = False; _pbr_layers(spec, mt, "surround", SUR_PX)                                     # ([CaM])
    for i, j in SUR_CELLS:                                   # a ring of patch-sized cells beside it, never under it: no two
        g = spec.worldbody.add_geom(); g.name = f"surround_{i}_{j}"; g.type = mujoco.mjtGeom.mjGEOM_PLANE   # floors overlap,
        g.size = [SIZE_M / 2, SIZE_M / 2, 0.01]; g.pos = [i * SIZE_M, j * SIZE_M, 0.001]                     # so none flickers
        g.material = "surround"; g.contype = 0; g.conaffinity = 0

_CUR = {}


def _upload(m, img, renderers, viewer, name="local_floor", filament=True):
    tid = m.texture(name).id; a = m.tex_adr[tid]
    u8 = (img if img.dtype == np.uint8 else np.clip(img * 255, 0, 255).astype(np.uint8))[::-1]   # MuJoCo lays texture
    # rows along -y; flipped here so row i of every image built in this file is +y (i * MPP from the patch's -y edge),
    # as the pile shading, rug placement and stains assume (measured 25 Sep 2026: unflipped, all were mirrored in y)
    m.tex_data[a:a + u8.size] = u8.reshape(-1)
    for r in renderers:
        if r is None: continue
        r._gl_context.make_current(); mujoco.mjr_uploadTexture(m, r._mjr_context, tid)
    if viewer is not None: viewer.update_texture(tid)
    if filament:                                                           # and the Filament cameras' copies ([Hsz]); not for
        import filament_cameras as fc; fc.texture_changed(m, tid)           # the classic-only per-view pile shading


def _surround(kind, seed):
    """the textures for the surround cells: this floor without its stains or rug (it tiles): (albedo, (normal, ORM) or None)"""
    base = kind.split("+")[0]
    g = generate(seed, "photo:" + base.split(":")[1] if ":" in base else base, marks=False, rug=False)
    return g[0], g[4]


def _upload_pbr(m, pbr, renderers, viewer, name):
    """a floor's normal and ORM textures (flat and matte for the generated floors), if the model has them"""
    if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TEXTURE, name + "_n") < 0: return
    n, orm = pbr if pbr is not None else (np.broadcast_to(FLAT_N, (PX, PX, 3)), np.broadcast_to(np.array([255, 204, 0], np.uint8), (PX, PX, 3)))
    _upload(m, np.ascontiguousarray(n), renderers, viewer, name + "_n"); _upload(m, np.ascontiguousarray(orm), renderers, viewer, name + "_orm")


def apply(m, seed, centre_xy, renderers=(), viewer=None, kind=None, view_from=None, focus_xy=None):
    """generate a floor for this seed, centre the patch at centre_xy, and upload it to every renderer and the viewer.
    focus_xy (world): where the item is, so a rug's edge lands near it"""
    focus = None if focus_xy is None else np.asarray(focus_xy) - np.asarray(centre_xy) + SIZE_M / 2
    albedo, kind, nap, sheen, pbr = generate(seed, kind, focus=focus); gid = m.geom("local_floor").id; m.geom_pos[gid][:2] = centre_xy
    if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "surround_1_0") >= 0:      # (models built before the surround have none)
        for i, j in SUR_CELLS: m.geom_pos[m.geom(f"surround_{i}_{j}").id][:2] = np.asarray(centre_xy) + np.array([i, j]) * SIZE_M
        s_alb, s_pbr = _surround(kind, seed); _upload(m, s_alb, renderers, viewer, "surround"); _upload_pbr(m, s_pbr, renderers, viewer, "surround")
    _CUR.update(m=m, albedo=albedo, nap=nap, sheen=sheen, centre=np.array(centre_xy, float), last_by={})
    _upload(m, shade(albedo, nap, sheen, view_from, centre_xy) if view_from is not None else albedo, renderers, viewer)
    _upload_pbr(m, pbr, renderers, viewer, "local_floor")
    return kind


def view_from(m, cam_pos, renderers=(), viewer=None, min_move=0.01):
    """re-shade the current floor for a camera at cam_pos (call before rendering that camera)"""
    if _CUR.get("m") is not m or _CUR["nap"] is None: return    # the model itself, not its id: an address is reused (29 Sep)
    key = tuple(id(r) for r in renderers); last = _CUR.setdefault("last_by", {}).get(key)   # each renderer holds its own shading
    if last is not None and np.linalg.norm(cam_pos - last) < min_move: return
    _CUR["last_by"][key] = np.array(cam_pos, float)
    _upload(m, shade(_CUR["albedo"], _CUR["nap"], _CUR["sheen"], cam_pos, _CUR["centre"]), renderers, viewer, filament=False)
