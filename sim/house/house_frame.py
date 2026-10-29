#!/usr/bin/env python3
"""One frame for a house's nights ([KkQ]): each night's lidar, with its odometry poses baked in (DimOS's
`dimos map pose-fill`), run through DimOS's pose graph (`dimos/mapping/loop_closure/pgo.py`) as one stream, night after
night, so revisited places tie the nights together as well as each night to itself. Writes each night's correction
(times, 3x3 SE(2) world <- believed) for `floor_mosaic2.py --corrections` and `calibrate_mount.py --corrections`.
Measured 27 Sep on house 70's two baseline nights against the sim's true path: the nights within 33 mm of each other
where both passed (odometry: 308), night 1 within 15-20 mm of its true path (odometry: 65).
Each loop closure DimOS proposes is verified before it enters the graph ([dmT], decision [fxk]): the robot's own scans
around its two keyframes must agree with the alignment it asks (loop_verify.py, through DimOS's `loop_verifier`); one
they do not confirm is left out, and every verdict is kept in verify.json. --verify none runs DimOS as it ships.
Usage: house_frame.py <memory.db> [<memory.db> ...] --out <dir> [--verify lidar|none] (dimos venv; ~6-10 min for two
nights, verified; run it with OMP_NUM_THREADS=1 for a repeatable graph)"""
import argparse, math, subprocess, sys, tempfile, time
from pathlib import Path

import numpy as np

from dimos.mapping.loop_closure.pgo import PGO
from dimos.memory.store.sqlite import SqliteStore

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src")); from houndkeeper import workspace   # noqa: E402  (decision [cy4])
DIMOS = workspace.dimos()
PGO_SETTINGS = {"odom_rot_var": 4e-4, "odom_trans_var_xy": 1e-3, "submap_resolution": 0.05, "key_pose_delta_trans": 0.25,
                "key_pose_delta_deg": 5.0, "loop_search_radius": 2.0, "loop_rot_var": 0.05}   # indoor, cm-level: 5 cm submaps,
# a keyframe each 25 cm, odometry trusted less than DimOS's outdoor defaults (tuned 27 Sep: 65 -> 20 mm). Loops back at
# DimOS's own trust and reach (1 Oct): at 0.01 and 3 m, false loop closures on houses 72 and 73 bent the map 2-4.9 m
# (p95); at these, 31-43 cm, and every house's floor pictures agreed within 0.1-4.4% (house_shape.py then straightens it)
VERIFY = ("lidar", "none")   # the stitch's verifier: the scans around each loop ([fxk]); the floor's fallback needs views
# painted before the frame exists, so it stays the bench's (joint_bench.py --verify either)


def verifier(dbs, mode, cache):
    """loop_verify.LoopVerifier over the nights' odometry and house_shape's keyframes, or None for DimOS as it ships"""
    if mode == "none": return None
    import joint_graph as jg, loop_verify as lv
    prob = jg.load_problem(dbs, cache=cache); return lv.LoopVerifier(prob.tracks, prob.kf, None, mode)


def posed(db: str, tmp: Path) -> Path:
    """a copy of the recording's lidar with each scan's odometry pose baked in"""
    out = tmp / (Path(db).parent.name + "_posed.db")
    subprocess.run([str(DIMOS / ".venv" / "bin" / "dimos"), "map", "pose-fill", db, "--out", str(out), "--streams", "lidar,odom,tf"],
                   check=True, capture_output=True, cwd=DIMOS)
    return out


def se2_of(M):
    th = math.atan2(M[1, 0], M[0, 0]); c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, M[0, 3]], [s, c, M[1, 3]], [0, 0, 1.0]])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("dbs", nargs="+"); ap.add_argument("--out", required=True)
    ap.add_argument("--pgo", nargs="*", default=[], help="settings over PGO_SETTINGS, as key=value (e.g. loop_search_radius=1.5)")
    ap.add_argument("--verify", default="lidar", choices=VERIFY, help="each proposed loop checked by the robot's scans ([dmT]); none: DimOS as it ships")
    a = ap.parse_args(); settings = dict(PGO_SETTINGS, **{k: float(v) for k, v in (x.split("=") for x in a.pgo)}); out = Path(a.out); out.mkdir(parents=True, exist_ok=True); t0 = time.time()
    with tempfile.TemporaryDirectory() as td:
        ver = verifier(a.dbs, a.verify, Path(td))
        stores = [SqliteStore(path=str(posed(db, Path(td)))) for db in a.dbs]
        for st in stores: st.start()
        obs = [[o for o in st.streams.lidar if o.pose_tuple is not None] for st in stores]
        graph = None                                   # one graph over the nights in turn (a staged handover, night 1
        for g in PGO(loop_verifier=ver, **settings)(iter([o for ob in obs for o in ob])): graph = g.data   # corrected first and later
        for k, ob in enumerate(obs):                   # nights started on its dock, was worse: 179/91 mm against 91/73, 27 Sep)
            np.savez(out / f"corrections_{k + 1}.npz", times=np.array([o.ts for o in ob]),
                     Ts=np.array([se2_of(graph.correction_at(o.ts).to_matrix()) for o in ob]), db=a.dbs[k])
        for st in stores: st.stop()
    import json as _json                                       # its health, for the pipeline report ([63R])
    refused = sum(not r["accepted"] for r in ver.record) if ver else None
    (out / "summary.json").write_text(_json.dumps({"nights": len(a.dbs), "keyframes": len(graph.keyframes), "loop_closures": len(graph.loops),
                                                   "verify": a.verify, "loops_refused": refused}))
    if ver: (out / "verify.json").write_text(_json.dumps({"mode": a.verify, "candidates": ver.record}, indent=0))
    print(f"house frame: {len(a.dbs)} nights, {len(graph.keyframes)} keyframes, {len(graph.loops)} loop closures"
          + (f" ({refused} refused by the verifier)" if ver else " (unverified)") + f", {time.time() - t0:.0f} s -> {out}/corrections_<n>.npz")


if __name__ == "__main__":
    main()
