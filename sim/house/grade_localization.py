#!/usr/bin/env python3
"""How well the robot knows where it is ([KkQ]), graded against the sim's truth: the recording's odometry, and the
lidar localizer's poses, each against the robot's true path (the sim's scoring-only log, DIMOS_SIM_TRUTH_TRAJ). The
house frame is the robot's own (its first night's start), so one rigid offset for the whole night is removed first
(the best 2D fit); what's left is what hurts the floor map: error that changes over the night.
Usage: grade_localization.py <memory.db> <traj.csv> [--house house.npy] (dimos venv)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import argparse, math

import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from houndkeeper.localize import Localizer, scans_from, se2_params


def rigid_fit(a, b):
    """the 2D rotation and shift laying points a onto b (least squares): (R, t)"""
    ma, mb = a.mean(0), b.mean(0); H = (a - ma).T @ (b - mb); th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]]); return R, mb - R @ ma, th


def grade(name, est_xy, est_yaw, true_xy, true_yaw):
    R, t, th = rigid_fit(est_xy, true_xy); e = est_xy @ R.T + t - true_xy
    dy = np.degrees((est_yaw + th - true_yaw + np.pi) % (2 * np.pi) - np.pi); dy -= np.median(dy)
    d = np.hypot(*e.T)
    print(f"{name:10s} position error median {np.median(d) * 1000:5.1f} mm, p95 {np.percentile(d, 95) * 1000:5.1f} mm, max {d.max() * 1000:5.1f} mm; "
          f"heading median {np.median(np.abs(dy)):.2f} deg, p95 {np.percentile(np.abs(dy), 95):.2f} deg  (offset removed: {np.hypot(*t) * 1000:.0f} mm, {math.degrees(th):.2f} deg)")


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("db"); ap.add_argument("traj"); ap.add_argument("--house", default=None)
    a = ap.parse_args()
    T = np.genfromtxt(a.traj, delimiter=",", names=True)
    st = SqliteStore(path=a.db); st.start(); scans = scans_from(st)
    loc = Localizer(np.load(a.house) if a.house else None).run(scans)
    t = np.array(loc.times); ok = (t > T["wall_t"][0]) & (t < T["wall_t"][-1]); t = t[ok]
    it = lambda col: np.interp(t, T["wall_t"], T[col])
    true_xy = np.c_[it("true_x"), it("true_y")]; true_yaw = np.unwrap(np.interp(t, T["wall_t"], np.unwrap(T["true_yaw"])))
    odom_xy = np.c_[it("odom_x"), it("odom_y")]; odom_yaw = np.interp(t, T["wall_t"], np.unwrap(T["odom_yaw"]))
    Cs = [loc.correction(x) for x in t]
    loc_xy = np.array([C[:2, :2] @ p + C[:2, 2] for C, p in zip(Cs, odom_xy)]); loc_yaw = odom_yaw + np.array([se2_params(C)[2] for C in Cs])
    print(f"{len(t)} scans over {t[-1] - t[0]:.0f} s; true path {np.sum(np.hypot(*np.diff(true_xy, axis=0).T)):.1f} m")
    grade("odometry", odom_xy, odom_yaw, true_xy, true_yaw)
    grade("localized", loc_xy, loc_yaw, true_xy, true_yaw)


if __name__ == "__main__":
    main()
