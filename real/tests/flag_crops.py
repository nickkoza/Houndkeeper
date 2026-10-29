#!/usr/bin/env python3
"""The flags' crops for the phone cockpit's Review page ([71R]; decisions [Vd3], [oWO]): after a real night, one crop
per candidate in its candidates.json, so whichever of them the user's marks leave as flags has its picture.

Each candidate's crop is cut from the night's recording (night/recording.txt): the head camera's frame nearest in time
to the candidate's decision that sees its floor point (the light check's own choice of view, cascade.pick_views_by_time,
placed as the bench places it: baselines.frames.head_camera), a square around the point about twice the light check's
span, so the user sees a little floor around it. Every crop goes through the people screen before it is kept
(tools/screen_people.py, as the morning log's real nights' do, [hKP]); a crop it does not clear is never written. Only
a recording the catalog has cleared for its frames gives crops at all ([R1X]); otherwise none, and the index says why.
The crops stay on the laptop beside the night (review_crops/: <candidate id>.jpg and index.json); the project service
shows them to the user's phone through the relay ([Vd3]).

Usage: flag_crops.py <night dir>   (dimos venv python; CPU is enough: CUDA_VISIBLE_DEVICES= nice -n 15)"""
import argparse
import io
import json
import math
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src")); sys.path.insert(0, str(HK / "tools"))

OUT = "review_crops"
GROW = 2.0            # the crop's side against the light check's (recheck.crop_box): more floor round the point
PX = 240              # the saved crop's side (the phone shows it at about 160 css px)
QUALITY = 75


def pick(cands, poses, K, D, model, wh):
    """each candidate's (id, x_robot, y_robot: odometry's frame; ts: its decision) views, nearest in time first:
    [[(pose index, u, v, range)]]. poses: [(frame index, ts, T world <- optical)]"""
    from baselines import cascade as cs
    pts = [(float(c["x_robot"] if c.get("x_robot") is not None else c["x"]),
            float(c["y_robot"] if c.get("y_robot") is not None else c["y"]), float(c.get("ts") or 0.0)) for c in cands]
    return cs.pick_views_by_time(pts, poses, int(wh[0]), int(wh[1]), K, D, model)


def crops_for(cands, poses, image_at, K, D, model, wh, screen, views=None):
    """{candidate id: {"img": RGB crop or None, "ts", "box", "range_m"; or "withheld"/"none": why there is none}}, cut
    from each candidate's first view (pick's, unless views are given). image_at(frame index) -> the RGB frame;
    screen(crop) -> True only when clear of people"""
    from baselines import recheck as rk
    w, h = int(wh[0]), int(wh[1]); fx = float(np.asarray(K, float).reshape(3, 3)[0, 0])
    views = pick(cands, poses, K, D, model, wh) if views is None else views
    out = {}
    for c, vs in zip(cands, views):
        if not vs: out[c["id"]] = {"img": None, "none": "no frame saw its place"}; continue
        i, u, v, r = vs[0]
        x0, y0, s = rk.crop_box(u, v, r, fx, w, h)
        s2 = int(min(round(s * GROW), w, h)); cx, cy = x0 + s / 2, y0 + s / 2
        X0 = int(min(max(round(cx - s2 / 2), 0), w - s2)); Y0 = int(min(max(round(cy - s2 / 2), 0), h - s2))
        img = np.ascontiguousarray(np.asarray(image_at(poses[i][0]))[Y0:Y0 + s2, X0:X0 + s2])
        meta = {"ts": float(poses[i][1]), "box": [X0, Y0, X0 + s2, Y0 + s2], "range_m": round(float(r), 2)}
        if not screen(img): out[c["id"]] = dict(meta, img=None, withheld="the people screen did not clear it"); continue
        out[c["id"]] = dict(meta, img=img)
    return out


def jpeg(img, px=PX, quality=QUALITY) -> bytes:
    from PIL import Image
    im = Image.fromarray(np.ascontiguousarray(img))
    if im.size[0] > px: im = im.resize((px, px), Image.BILINEAR)
    b = io.BytesIO(); im.save(b, "JPEG", quality=quality, optimize=True); return b.getvalue()


def _cleared(db):
    """the recording's kind if the catalog has cleared it for its frames ([R1X]); raises PermissionError otherwise"""
    from houndkeeper import recordings
    rid = recordings.run_id_of(Path(db))
    if not rid: raise PermissionError("the recording is not in the catalog")
    try: recordings.require_screened(rid); return recordings.get(rid)["kind"]
    except SystemExit as e: raise PermissionError(str(e)) from None


def owl_screen(img) -> bool:
    import screen_people
    return not screen_people.people_in(img)


def make(night_dir, screen=None):
    """the night's review_crops/ (each candidate's crop, people-screened) and its index.json; returns the index"""
    d = Path(night_dir); out = d / OUT; out.mkdir(exist_ok=True)
    cands = json.loads((d / "candidates.json").read_text()).get("candidates") or []
    idx = {"note": "", "crops": {}}
    try:
        db = Path((d / "night" / "recording.txt").read_text().strip())
        kind = _cleared(db)
    except (OSError, PermissionError) as e:
        idx["note"] = f"Pictures withheld: {e}"
        (out / "index.json").write_text(json.dumps(idx, indent=1)); return idx
    screen = screen or ((lambda img: True) if kind == "sim" else owl_screen)   # a sim house holds no people
    from dimos.memory.store.sqlite import SqliteStore
    import floor_mosaic2 as fm
    from baselines import frames as fr, recheck as rk
    st = SqliteStore(path=str(db)); st.start()
    try:
        pose, T_bo, lag, _, ci = fr.head_camera(st, d.parent if (d.parent / "head_mount.json").exists() else d, str(db))
        poses = []
        for k, o in enumerate(st.stream("color_image")):
            if k % rk.EVERY: continue
            P = pose(o.ts - lag)
            if P is not None: poses.append((k, float(o.ts), P @ T_bo))
        K, D, model, wh = np.array(ci.K, float).reshape(3, 3), list(ci.D or []), ci.distortion_model or "", (ci.width, ci.height)
        views = pick(cands, poses, K, D, model, wh)
        want = {poses[vs[0][0]][0] for vs in views if vs}
        frames = {k: fm.arr(o.data) for k, o in enumerate(st.stream("color_image")) if k in want}   # one pass
        got = crops_for(cands, poses, frames.__getitem__, K, D, model, wh, screen, views)
    finally:
        st.stop()
    for cid, e in got.items():
        img = e.pop("img")
        if img is not None: (out / f"{cid}.jpg").write_bytes(jpeg(img)); e["file"] = f"{cid}.jpg"
        idx["crops"][cid] = e
    withheld = sum("withheld" in e for e in idx["crops"].values())
    if withheld: idx["note"] = f"{withheld} picture(s) withheld by the people screen."
    (out / "index.json").write_text(json.dumps(idx, indent=1))
    return idx


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0]); ap.add_argument("night")
    idx = make(ap.parse_args(argv).night)
    n = sum("file" in e for e in idx["crops"].values())
    print(f"{n} of {len(idx['crops'])} candidates have a crop" + (f"; {idx['note']}" if idx["note"] else ""))


if __name__ == "__main__":
    main()
