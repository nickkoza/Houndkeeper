#!/usr/bin/env python3
"""The house twin ([lQ1]): a MuJoCo house built from a run dir's own outputs, as the replay path ([V35]) or the sim's
baseline nights leave them, and nothing of a sim house's truth ([UdA]):
  - what stands up: each night's lidar, levelled on its own floor and moved into the house's frame by the shape solve's
    corrections (house_shape.py), binned into 5 cm columns; a column is a box from the floor, or a slab where the lidar
    camera saw the floor under it (a table top); a pocket wholly enclosed by columns with no seen floor in it is the
    inside of something solid (the lidar returns only from faces), filled; tops cut by the map's own height limit go up to a stated ceiling (--ceiling:
    the lidar never sees the top of a wall); cells nobody mapped get nothing ([TgM]: absent, not guessed);
  - the floor ([POG]): the nights' floor pictures with the lighting taken out by each spot's trimmed mean of views ([evl]:
    --pictures, floor_mosaic2.py --combine trimmed), night 2's laid on night 1's patch by patch the way the ground
    truthing does it (floor_truth2.py: 60 cm patches, masked edge correlation, houndkeeper.floor_register.snap), so the
    seams line up, then blended into one texture over the house's floor; floor never seen is one flat marker grey;
  - the lights: the procedural houses' lit regime (make_nights: the same fixtures, strengths, bounce and headlight),
    a fixture over each ~4 m of mapped floor, as the generator puts one over each part of the open plan ([TgM]).
The general part (heights from points, boxes, the scene with its texture) is DimOS's: simulation/mujoco/scene_from_map.py
in the fork ([4TQ]); this is the house's part. Writes <out>/<name>.xml with its texture, twin.json (what went in, what
came out) and twin.npz (the floor at 1 cm and its seen mask, for the scorer). A twin of a real house is a picture of its
floor: it is written under data/ only ([au3]). --install puts it where DimOS's sim finds rooms (--mujoco-room <name>).
Usage: house_twin.py <run dir> [--out data/twin/<name>] [--name twin] [--pictures n1.npz n2.npz] [--ceiling 2.4]
       [--px 0.01] [--install]   (dimos venv; ~1 min)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))
import argparse, importlib.util, json, math, shutil, time
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage as ndi

HK = Path(__file__).resolve().parents[2]
HX, HY, RES = 8.4, 4.9, 0.01                  # the floor pictures' grid (floor_mosaic2's), row 0 at +y
COL = 0.05                                    # the columns (m)
PATCH, PAD, MAXS = 60, 12, 8                  # registration: floor_truth2's 60 cm patches, searched +-8 cm
MIN_SCORE, MIN_LEAD = 0.3, 0.05               # a patch's match counted: floor_truth2's "clearly best"
FIELD_SIGMA = 0.6                             # m: the warp field's smoothing between registered patches
FEATHER = 0.30                                # m: two nights blended over this much from either's edge (no seam)
LIGHT_TILE = 5.6                              # m: a ceiling fixture over each tile of mapped floor (>= LIGHT_MIN_M2 of it):
LIGHT_MIN_M2 = 4.0                            # ~33 m2 a fixture, as the lit regime's five over the 165 m2 open plan
REACH_LOSS = 0.25                             # m: a top this near the map's own top may be cut by it (the lidar's reach
                                              # ends lower on a far wall: house 70's walls ended 1.30-1.50 m, 3 Oct)
UNDER_SEEN = 0.5                              # a column is a slab only where the floor picture saw this share of the
                                              # floor under it ([TgM]: a sofa's seat is not a table top)
TRUTH_NAMES = (".truth.json", ".traj.csv", "house_floor_s")   # what the builder must never open ([UdA])


def scene_lib():
    """DimOS's scene_from_map: the fork's, or the twin branch's worktree until the fork carries it (feat/real-to-sim-scene)"""
    try:
        from dimos.simulation.mujoco import scene_from_map
        return scene_from_map
    except ImportError:
        from houndkeeper import workspace
        p = workspace.sibling("dimos-twin") / "dimos" / "simulation" / "mujoco" / "scene_from_map.py"
        spec = importlib.util.spec_from_file_location("scene_from_map", p); m = importlib.util.module_from_spec(spec)
        _sys.modules["scene_from_map"] = m; spec.loader.exec_module(m); return m


def opened(path):
    """a path the builder reads: refused if it is a sim house's truth ([UdA])"""
    p = Path(path)
    if any(t in p.name for t in TRUTH_NAMES): raise SystemExit(f"refusing to read sim truth: {p}")
    return p


def inputs(run, pictures=None):
    """the run dir's outputs the twin is built from: {dbs, corrections, pictures}"""
    run = Path(run); ff = json.loads(opened(run / "floor_frame.json").read_text())
    n = len(ff["dbs"])
    return {"dbs": [str(opened(d)) for d in ff["dbs"]],
            "corrections": [str(opened(run / "shape" / f"corrections_{k}.npz")) for k in range(1, n + 1)],
            "settle": str(opened(run / "shape" / "settle.npz")),
            "pictures": [str(opened(p)) for p in (pictures or [run / f"mosaic_n{k}.npz" for k in range(1, n + 1)])]}


def house_points(dbs, corrections, every=2):
    """every night's lidar in the house's frame, heights from each scan's own floor (as tall_layer.py --corrections)"""
    from dimos.memory.store.sqlite import SqliteStore
    from houndkeeper.localize import Localizer, level
    out = []
    for db, cp in zip(dbs, corrections):
        z = np.load(cp); loc = Localizer(times=list(z["times"]), Ts=list(z["Ts"]))
        st = SqliteStore(path=db); st.start()
        try:
            for k, o in enumerate(st.stream("lidar")):
                if k % every: continue
                p = o.data.as_numpy(); P = level(np.asarray(p[0] if isinstance(p, tuple) else p, float)[:, :3])
                C = loc.correction(o.ts); out.append(np.c_[P[:, :2] @ C[:2, :2].T + C[:2, 2], P[:, 2]])
        finally:
            st.stop()
    return np.vstack(out)


def start_pose(settle):
    """where night 1 began, in the house's frame: its first keyframe as the shape solve placed it (x, y)"""
    z = np.load(settle); C = z["passes"][-1][0]; b = z["believed"][0]
    return [round(float(v), 3) for v in C[:2, :2] @ b + C[:2, 2]]


def grey(rgb): return rgb.astype(np.float32).mean(-1)


def register(ref, ref_seen, mov, mov_seen):
    """mov's picture laid on ref's: each 60 cm patch of mov matched on ref by masked edge correlation within +-8 cm
    (floor_register.snap, as floor_truth2.py matches a picture to the true floor), the clear matches smoothed into a
    field over the house. Returns (rows, cols) shift fields (cells; mov[r, c] lies at ref[r + dr, c + dc]) and stats"""
    from houndkeeper import floor_register as fr
    g, t = grey(mov), grey(ref); H, W = g.shape; C, S = [], []
    for r in range(PAD, H - PATCH - PAD, PATCH):
        for c in range(PAD, W - PATCH - PAD, PATCH):
            sl = (slice(r, r + PATCH), slice(c, c + PATCH)); wl = (slice(r - PAD, r + PATCH + PAD), slice(c - PAD, c + PATCH + PAD))
            if mov_seen[sl].mean() < 0.9 or ref_seen[wl].mean() < 0.5: continue
            F = np.zeros((PATCH + 2 * PAD,) * 2, np.float32); Fm = np.zeros(F.shape, bool)
            F[PAD:-PAD, PAD:-PAD] = g[sl]; Fm[PAD:-PAD, PAD:-PAD] = mov_seen[sl]
            s_ = fr.snap(F, Fm, t[wl], ref_seen[wl], maxs=MAXS, turns_deg=(0.0,), lead_cells=4, turn_edge_locks=True)
            if s_ is None or s_[3] < MIN_SCORE or s_[4] < MIN_LEAD: continue
            C.append((r + PATCH / 2, c + PATCH / 2)); S.append((s_[1], s_[2]))
    if not S: return np.zeros((H, W)), np.zeros((H, W)), {"patches": 0}
    C, S = np.array(C), np.array(S, float)
    med = np.median(S, 0); keep = np.hypot(*(S - med).T) <= 4.0   # a patch far off the rest: a false match (est.)
    C, S = C[keep], S[keep]
    sig = FIELD_SIGMA / RES; acc = np.zeros((3, H, W), np.float32)
    for (r, c), (dr, dc) in zip(C.astype(int), S): acc[:, r, c] += (1.0, dr, dc)
    acc = np.stack([ndi.gaussian_filter(a, sig, mode="constant") for a in acc])
    w = acc[0]; far = w < 1e-6 * acc[0].max()                     # no patch within reach: the median shift
    DR = np.where(far, med[0], acc[1] / np.maximum(w, 1e-12)); DC = np.where(far, med[1], acc[2] / np.maximum(w, 1e-12))
    rel = np.hypot(*(S - np.median(S, 0)).T)
    return DR, DC, {"patches": int(len(S)), "dropped": int((~keep).sum()), "shift_mm_median": round(float(np.hypot(*med) * 10), 1),
                    "spread_mm_median": round(float(np.median(rel) * 10), 1), "spread_mm_p90": round(float(np.percentile(rel, 90) * 10), 1)}


def warp(img, seen, DR, DC):
    """img moved by the field: out[r, c] = img[r - dr, c - dc]"""
    R, Cc = np.mgrid[:seen.shape[0], :seen.shape[1]].astype(np.float32); co = [R - DR, Cc - DC]
    out = np.stack([ndi.map_coordinates(img[..., k].astype(np.float32), co, order=1, cval=0) for k in range(3)], -1)
    return out, ndi.map_coordinates(seen.astype(np.float32), co, order=0, cval=0) > 0.5


def floor_texture(pictures):
    """[POG] one floor from the nights' pictures: night 1 as it is, every later night registered to it, matched to its
    brightness over the floor both saw (one gain per colour), and the nights blended, each weighed by how far inside its
    own seen floor a spot lies (up to FEATHER): no seam where one night's picture ends. Returns (rgb, seen, stats)"""
    zs = [np.load(p) for p in pictures]
    ref = zs[0]["rgb"].astype(np.float32); ref_seen = np.isfinite(zs[0]["dist"].astype(np.float32))
    imgs, masks, stats = [ref], [ref_seen], []
    for z in zs[1:]:
        mov = z["rgb"]; ms = np.isfinite(z["dist"].astype(np.float32))
        DR, DC, st = register(ref.astype(np.uint8), ref_seen, mov, ms); img, ms = warp(mov, ms, DR, DC)
        both = ms & ref_seen
        if both.sum() > 10000:
            gain = ref[both].mean(0) / np.maximum(img[both].mean(0), 1); img = img * gain; st["gain"] = [round(float(x), 3) for x in gain]
        imgs.append(img); masks.append(ms); stats.append(st)
    wts = [np.minimum(ndi.distance_transform_edt(m) * RES, FEATHER) * m for m in masks]
    wsum = np.sum(wts, 0); seen = np.any(masks, 0)
    rgb = np.sum([w[..., None] * i for w, i in zip(wts, imgs)], 0) / np.maximum(wsum, 1e-9)[..., None]
    rgb = np.where(seen[..., None], np.clip(rgb + 0.5, 0, 255), 0).astype(np.uint8)
    return rgb, seen, stats


def twin_lights(seen, ceiling):
    """the procedural houses' lit regime (make_nights.regime(.., 'lit'), lights_xml's fixtures and bounce, its headlight),
    a fixture over each LIGHT_TILE of mapped floor at its middle: [(Light), ...], headlight colours"""
    import make_nights as mn
    lib = scene_lib(); light = mn.regime(0, 1, "lit")                # the lit regime is every house's own: no draw
    tr, tg, tb = mn.TINT[light["tint"]]; c3 = lambda k: (k * tr, k * tg, k * tb)
    k = light["ceiling"] * mn.LIT_CEILING; zc = ceiling - 0.05; out = []
    H, W = seen.shape; t = int(LIGHT_TILE / RES)
    for r in range(0, H, t):
        for c in range(0, W, t):
            m = seen[r:r + t, c:c + t]
            if m.sum() * RES * RES < LIGHT_MIN_M2: continue
            rr, cc = np.argwhere(m).mean(0); x, y = -HX + (c + cc + 0.5) * RES, HY - (r + rr + 0.5) * RES; i = len(out) // 2
            out.append(lib.Light(f"ceiling{i}_0", (x, y, zc), c3(k), cutoff=85, exponent=1, attenuation=(1, 0.05, 0.05), bulbradius=0.12))
            out.append(lib.Light(f"ceiling{i}_bounce", (x, y, ceiling - 0.3), c3(k * mn.BOUNCE), cutoff=88, exponent=0,
                                 attenuation=(1, 0.02, 0.01), bulbradius=mn.BOUNCE_BULB))
    amb = c3(light["ambient"])
    return out, (amb, amb)


def build(run, out, name="twin", pictures=None, ceiling=2.4, px=RES, every=2):
    lib = scene_lib(); t0 = time.time()
    src = inputs(run, pictures)
    P = house_points(src["dbs"], src["corrections"], every)
    top_cut = float(np.percentile(P[:, 2], 99.9))                 # the map's own height limit, read off its returns
    grid = lib.HeightGrid.from_points(P, COL, floor_z=0.0, min_height=0.10, origin=(-HX, -HY), shape=(int(2 * HY / COL), int(2 * HX / COL)))
    rgb, seen, reg = floor_texture(src["pictures"])
    k = int(COL / RES); under = seen.reshape(seen.shape[0] // k, k, seen.shape[1] // k, k).mean((1, 3))[::-1] >= UNDER_SEEN
    boxes = lib.boxes_from_heights(grid, floor_seen=under, clipped_above=top_cut - REACH_LOSS, extend_to=ceiling)
    tex = Image.fromarray(rgb)
    if px != RES: tex = tex.resize((round(2 * HX / px), round(2 * HY / px)), Image.BILINEAR)
    sm = Image.fromarray(seen).resize(tex.size, Image.NEAREST)
    floor = lib.FloorPicture(rgb=np.asarray(tex)[::-1], seen=np.asarray(sm)[::-1], resolution=px, origin=(-HX, -HY))
    lights, head = twin_lights(seen, ceiling)
    scene = lib.build_scene(boxes, floor, name=name, lights=lights, ceiling=ceiling, headlight=head)
    out = Path(out); xml = scene.write(out)
    np.savez_compressed(out / "twin.npz", rgb=rgb, seen=seen)
    foot = np.isfinite(grid.top)
    man = {"name": name, "built": time.strftime("%Y-%m-%dT%H:%M"), "from": src, "run": str(run),
           "reads": "the run's recordings, shape corrections and floor pictures only ([UdA])",
           "start_pos": start_pose(src["settle"]), "ceiling_m": ceiling, "map_top_m": round(top_cut, 3),
           "boxes": len(boxes), "slabs": sum(b.z0 > 0 for b in boxes),
           "mapped_m2": round(sum((b.x1 - b.x0) * (b.y1 - b.y0) for b in boxes), 1), "filled_m2": round(sum((b.x1 - b.x0) * (b.y1 - b.y0) for b in boxes) - float(foot.sum() * COL * COL), 1),
           "floor_seen_m2": round(float(seen.sum() * RES * RES), 1), "floor_unmapped_share": round(float(1 - seen.mean()), 3),
           "texture_px_m": px, "registration": reg, "lights": len(lights), "seconds": round(time.time() - t0)}
    (out / "twin.json").write_text(json.dumps(man, indent=1))
    return xml, man


def private_out(out):
    """[au3]: a twin goes under data/ (or a scratch dir outside the repo); never into sim/, the repo's code or the sim's runs"""
    o = Path(out).resolve()
    if o.is_relative_to(HK) and not o.is_relative_to(HK / "data"):
        raise SystemExit(f"refusing to write a twin into the repo outside data/ ({o}): a twin's floor is a picture of the house")
    return o


def install(xml, name):
    """the twin where DimOS's sim finds rooms: scene_<name>.xml beside the others, its texture in the sim's one texture folder"""
    import make_nights as mn
    xml = Path(xml); tex = xml.parent / f"{name}_floor.png"
    shutil.copy(tex, mn.TEX_DIR / tex.name); shutil.copy(xml, mn.mh.DIMOS_DATA / f"scene_{name}.xml")
    return mn.mh.DIMOS_DATA / f"scene_{name}.xml"


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--out", default=None)
    ap.add_argument("--name", default="twin"); ap.add_argument("--pictures", nargs="+", default=None,
                                                               help="the nights' floor pictures (default the run's mosaic_n<k>.npz; [POG]: trimmed ones)")
    ap.add_argument("--ceiling", type=float, default=2.4, help="m: where walls the lidar saw to its top end (a stated height)")
    ap.add_argument("--px", type=float, default=RES, help="the texture's metres a pixel (0.003: the procedural floors')")
    ap.add_argument("--install", action="store_true", help="copy into DimOS's sim data (scene_<name>.xml)")
    a = ap.parse_args()
    out = private_out(a.out or HK / "data" / "twin" / a.name)
    xml, man = build(a.run, out, a.name, a.pictures, a.ceiling, a.px)
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(xml))                    # it loads (parsed only)
    print(f"twin {a.name}: {man['boxes']} boxes ({man['slabs']} slabs), {man['mapped_m2']} m2 mapped, floor {man['floor_seen_m2']} m2 "
          f"seen ({man['floor_unmapped_share']:.0%} of the plane unmapped), {man['lights']} lights, {m.ngeom} geoms; "
          f"night 2 on night 1: {man['registration']}; {man['seconds']} s -> {xml}")
    if a.install: print(f"installed: {install(xml, a.name)} (dimos --mujoco-room {a.name} --mujoco-start-pos {man['start_pos'][0]},{man['start_pos'][1]})")


if __name__ == "__main__":
    main()
