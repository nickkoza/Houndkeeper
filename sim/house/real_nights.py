#!/usr/bin/env python3
"""Can a recording build a floor map with no sim truth ([V35])? What the map-building steps read from a recording
(floor_mosaic2.py, tall_layer.py, baseline_views.py, house_frame.py, house_shape.py, calibrate_mount.py), checked before
real_nights.sh spends minutes on it: the streams, the camera's transforms, and how much of the night's path lies on the
floor grid those steps paint (a fixed HX x HY canvas about odometry's origin: the sim's house fits it by construction, a
real house started from its dock may not). Prints what is missing and exits 1 when a step could not run.
Usage: real_nights.py <memory.db> [<memory.db> ...]   (dimos venv)"""
import argparse, sys

import numpy as np

from floor_mosaic2 import HX, HY        # the floor grid every map-building step paints (tall_layer.py and confirm_stream.py share it)

STREAMS = {   # stream -> the steps that read it
    "color_image": "floor picture, views, mount", "camera_info": "floor picture, views, mount",
    "tf": "every step (poses and the camera's place on the body)", "odom": "frame lag, house frame",
    "lidar": "tall layer, house frame and shape", "global_costmap": "floor picture (structures, before the stitch)",
    "global_map": "floor picture (lines of sight, before the stitch)"}
TF_PAIRS = [("world", "base_link"), ("base_link", "camera_link"), ("camera_link", "camera_optical")]
MIN_ON_CANVAS = 0.98                         # of the night's poses on the floor grid; less and the picture is cut off (est.)


def problems(streams, tf_pairs, on_canvas):
    """what stops a recording building its floor map: [(severity, text)], severity 'missing' (a step cannot run) or
    'warn' (it runs but loses floor); streams and tf_pairs are what the recording holds, on_canvas its poses' share"""
    out = [("missing", f"no '{s}' stream ({use})") for s, use in STREAMS.items() if s not in streams]
    out += [("missing", f"no {a} -> {b} transform (tf)") for a, b in TF_PAIRS if (a, b) not in tf_pairs]
    if on_canvas is not None and on_canvas < MIN_ON_CANVAS:
        out.append(("warn", f"only {on_canvas:.0%} of the night's path lies on the floor grid ({2 * HX:.1f} x {2 * HY:.1f} m "
                            f"about odometry's origin): floor beyond it is not mapped"))
    return out


def canvas_share(xy, hx=HX, hy=HY, margin=0.0):
    """share of positions (N x 2, m, odometry's frame) on the floor grid, `margin` m inside its edge"""
    xy = np.asarray(xy, float).reshape(-1, 2)
    if not len(xy): return 0.0
    return float(((np.abs(xy[:, 0]) <= hx - margin) & (np.abs(xy[:, 1]) <= hy - margin)).mean())


def survey(db):
    """(streams, tf pairs, share of odometry poses on the canvas) of one recording"""
    from dimos.memory.store.sqlite import SqliteStore
    st = SqliteStore(path=db); st.start()
    try:
        streams = set(st.list_streams()); pairs = set()
        if "tf" in streams:
            for o in st.stream("tf"):
                for t in getattr(o.data, "transforms", [o.data]): pairs.add((t.frame_id, t.child_frame_id))
                if all(p in pairs for p in TF_PAIRS): break
        xy = [(o.data.position.x, o.data.position.y) for o in st.stream("odom")] if "odom" in streams else []
    finally:
        st.stop()
    return streams, pairs, (canvas_share(xy) if xy else None)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("dbs", nargs="+"); a = ap.parse_args(); bad = False
    for k, db in enumerate(a.dbs, 1):
        streams, pairs, share = survey(db); ps = problems(streams, pairs, share)
        bad |= any(s == "missing" for s, _ in ps)
        print(f"night {k}: {db}: " + ("ready" if not ps else "; ".join(f"{s}: {t}" for s, t in ps))
              + ("" if share is None else f" ({share:.0%} of its path on the floor grid)"), flush=True)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
