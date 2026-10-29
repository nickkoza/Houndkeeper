#!/usr/bin/env python3
"""The offline pose ablation ([KkQ]): a recording's floor tool run with the robot's poses moved to where the sim says it
truly was (the sim's traj.csv), to measure what pose error costs without a new run. An analysis tool, never the
robot's: the robot's tools take the moved poses only as an injected pose_hook, which nothing on the robot sets ([1B2]).
Usage: true_path.py <traj.csv> floor_mosaic2|calibrate_mount <that tool's arguments>   (dimos venv python)
       (the confirmer's replay: score_confirm.py --truth <traj.csv>)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import numpy as np

TOOLS = ("floor_mosaic2", "calibrate_mount")


def true_pose(pose, traj_csv):
    """sim-only: the robot's pose lookup (t -> world<-base) moved to where the robot truly was (the sim's traj.csv), to
    measure what pose error costs without a new run"""
    TT = np.genfromtxt(traj_csv, delimiter=",", names=True)
    def pose_t(t):
        M = pose(t)
        if M is None: return None
        th = np.interp(t, TT["wall_t"], np.unwrap(TT["true_yaw"])) - np.interp(t, TT["wall_t"], np.unwrap(TT["odom_yaw"]))
        c_, s_ = np.cos(th), np.sin(th); M = M.copy(); M[:3, :3] = np.array([[c_, -s_, 0], [s_, c_, 0], [0, 0, 1.0]]) @ M[:3, :3]
        M[0, 3] = np.interp(t, TT["wall_t"], TT["true_x"]); M[1, 3] = np.interp(t, TT["wall_t"], TT["true_y"]); return M
    return pose_t


def hook(traj_csv):
    """the pose_hook the robot's tools take: their pose lookup moved onto the sim's true path"""
    return lambda pose: true_pose(pose, traj_csv)


def main(argv=None):
    argv = list(_sys.argv[1:] if argv is None else argv)
    if len(argv) < 2 or argv[1].removesuffix(".py") not in TOOLS:
        raise SystemExit(f"usage: true_path.py <traj.csv> {'|'.join(TOOLS)} <that tool's arguments>")
    import importlib
    tool = importlib.import_module(argv[1].removesuffix(".py"))
    tool.main(argv[2:], pose_hook=hook(argv[0]))


if __name__ == "__main__":
    main()
