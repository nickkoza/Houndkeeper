#!/usr/bin/env python3
"""The house frame's gate ([Tu2], decisions [myl] and [UpK]): each night's correction (house frame <- the night's own
odometry) is refused when it turns or moves the robot too far from where odometry put it, at the dock where every night
starts and over the night. A night starts on the dock, so its odometry starts in the dock's frame: a correction far from identity
there is the pose graph or the shape solve gone wrong, not drift. Over the night odometry drifts a few degrees, and
that heading drift moves the robot in proportion to how far it has gone from the dock, so the move allowed grows with
that distance ([UpK]); a correction past that has the map turned or folded (house 72's re-recorded baselines, 3 Oct
2026: about -90 deg and 7.8 m, which every later picture and the live tidy then read).
Reads only the shape solve's own outputs (corrections_<n>.npz and settle.npz's believed keyframe positions): the
robot's lidar and odometry, never the sim's truth.
Usage: frame_gate.py <shape dir> [--out gate.json]   exit 1, naming each night and how far, on a refusal"""
import argparse, json, math, sys
from pathlib import Path

import numpy as np

# The limits ([myl]): the tuning houses' own shape corrections as of 4 Oct 2026 (houses 70, 71, 73, 76, 77, 82-93,
# with 78, 95 and 96; tests/test_frame_gate.py checks every one passes), every one a working map:
#   at the dock: turns to 3.29 deg (house 88, night 2), moves to 0.061 m (house 83, night 2)
#   over the night: turns to 10.46 deg (house 83, night 1), moves to 0.627 m (house 83, night 1)
# House 72's broken stitch: 89.85 deg and 3.78 m at night 2's dock, 94.7 deg and 15.0 m over night 1.
# Over the night the move allowed at a keyframe is max(NIGHT_M, NIGHT_M_BASE + NIGHT_M_PER_M x its distance from the
# dock), by odometry ([UpK], 13 Oct 2026): house 76's 1280 night 2 was a right frame (3 cm median against truth) that
# moved 1.02 m from odometry 10.8 m from the dock after a 20 m run with 4-5.7 deg of heading drift. 0.08 m a metre
# passes all 106 archived tuning-house nights (closest house 71's night 2, 0.25 m under) and still refuses every
# planted turn of that night about the dock that 1.0 m flat refused; a steeper slope let 10-12 deg through.
DOCK_DEG, DOCK_M = 5.0, 0.3
NIGHT_DEG, NIGHT_M = 15.0, 1.0
NIGHT_M_BASE, NIGHT_M_PER_M = 0.3, 0.08


def excursions(times, Ts, key_times, believed):
    """how far one night's correction takes the robot from its odometry pose, at each keyframe: (turn deg, move m)
    arrays, the first at the dock. Ts: (n, 3, 3) SE(2) house <- odometry at `times`; believed: (k, 2) odometry
    positions at `key_times`"""
    times, Ts = np.asarray(times, float), np.asarray(Ts, float)
    key_times, B = np.asarray(key_times, float), np.asarray(believed, float).reshape(-1, 2)
    i = np.clip(np.searchsorted(times, key_times), 1, len(times) - 1) if len(times) > 1 else np.zeros(len(key_times), int)
    if len(times) > 1:                                         # the nearer of the two corrections around each keyframe
        i = np.where(np.abs(times[i - 1] - key_times) <= np.abs(times[i] - key_times), i - 1, i)
    C = Ts[i]
    turn = np.degrees(np.abs(np.arctan2(C[:, 1, 0], C[:, 0, 0])))
    move = np.linalg.norm(np.einsum("nij,nj->ni", C[:, :2, :2], B) + C[:, :2, 2] - B, axis=1)
    return turn, move


def move_limit(dist):
    """the over-the-night move allowed at keyframes `dist` metres from the dock by odometry ([UpK])"""
    return np.maximum(NIGHT_M, NIGHT_M_BASE + NIGHT_M_PER_M * np.asarray(dist, float))


def judge(turn, move, dist=None):
    """a night's verdict from its excursions: a dict of the figures and the reasons it is refused (empty: it passes).
    dist: each keyframe's distance from the dock by odometry (none given: the flat NIGHT_M everywhere)"""
    turn, move = np.asarray(turn, float), np.asarray(move, float)
    lim = move_limit(np.zeros(len(move)) if dist is None else dist)
    w = int(np.argmax(move - lim))                              # the keyframe nearest its limit, or furthest past it
    r = {"dock_turn_deg": float(turn[0]), "dock_move_m": float(move[0]), "max_turn_deg": float(turn.max()),
         "max_move_m": float(move.max()), "at_max_turn_kf": int(np.argmax(turn)), "at_max_move_kf": int(np.argmax(move)),
         "tightest_kf": w, "tightest_move_m": float(move[w]), "tightest_limit_m": float(lim[w]),
         "tightest_dist_m": float(0.0 if dist is None else np.asarray(dist, float)[w])}
    why = []
    if turn[0] > DOCK_DEG: why.append(f"turned {turn[0]:.1f} deg at the dock (limit {DOCK_DEG:g})")
    if move[0] > DOCK_M: why.append(f"moved {move[0]:.2f} m at the dock (limit {DOCK_M:g})")
    if turn.max() > NIGHT_DEG: why.append(f"turned {turn.max():.1f} deg over the night, keyframe {r['at_max_turn_kf']} (limit {NIGHT_DEG:g})")
    if move[w] > lim[w]: why.append(f"moved {move[w]:.2f} m over the night, keyframe {w}, {r['tightest_dist_m']:.1f} m from the dock "
                                    f"(limit {lim[w]:.2f} m there)")
    r["refused"] = why
    return r


def check(shape_dir):
    """every night's verdict in a shape solve's output directory: {night: judge(...)}"""
    d = Path(shape_dir); z = np.load(d / "settle.npz"); night = np.asarray(z["night"])
    out = {}
    for k in sorted(int(n) for n in np.unique(night)):
        c = np.load(d / f"corrections_{k}.npz"); m = night == k
        B = np.asarray(z["believed"])[m].reshape(-1, 2)
        out[k] = judge(*excursions(c["times"], c["Ts"], np.asarray(z["times"])[m], B), np.linalg.norm(B - B[0], axis=1))
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("shape_dir"); ap.add_argument("--out", default=None)
    a = ap.parse_args(); v = check(a.shape_dir)
    for k, r in v.items():
        print(f"house frame gate: night {k}: at the dock {r['dock_turn_deg']:.2f} deg, {r['dock_move_m']:.3f} m; over the night "
              f"up to {r['max_turn_deg']:.2f} deg, {r['max_move_m']:.3f} m; nearest its move limit at keyframe {r['tightest_kf']}, "
              f"{r['tightest_move_m']:.2f} m of {r['tightest_limit_m']:.2f} m allowed {r['tightest_dist_m']:.1f} m from the dock" + (": REFUSED, " + "; ".join(r["refused"]) if r["refused"] else ""))
    bad = [k for k, r in v.items() if r["refused"]]
    if bad:
        print(f"house frame REFUSED: night(s) {bad} placed far from their own odometry; the pose graph or the shape solve "
              f"went wrong (a false loop closure, or a turned match), and every picture would be built in a turned frame")
        return 1
    if a.out:
        Path(a.out).write_text(json.dumps({"limits": {"dock_deg": DOCK_DEG, "dock_m": DOCK_M, "night_deg": NIGHT_DEG, "night_m": NIGHT_M,
                                                      "night_m_base": NIGHT_M_BASE, "night_m_per_m": NIGHT_M_PER_M},
                                           "nights": {str(k): r for k, r in v.items()}}, indent=1))
    print("house frame gate: passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
