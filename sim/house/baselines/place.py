"""One placement for every arm ([o0E]): a detection in a head-camera frame becomes a point on the floor through the
camera's own lens (the fisheye's calibration, as the recording's camera_info gives it), its mount and the robot's pose at
the frame's time, looked up the stored lag earlier on the recording's own clock ([gJq], [55G]). The floor is the plane
z = 0 of the odometry's frame; the point is where the robot would walk to (odometry), which the scorer alone carries into
the sim's frame (traj.csv, as bench_detect does). Nothing here reads the truth."""
import numpy as np


def bottom_centre(box):
    """the pixel a box stands on: its bottom edge's middle (x0, y0, x1, y1 in pixels)"""
    x0, y0, x1, y1 = (float(v) for v in box)
    return 0.5 * (x0 + x1), max(y0, y1)


def pixel_ray(u, v, K, D=None, model="equidistant"):
    """the unit ray (camera optical frame: z forward, x right, y down) through pixel (u, v); the fisheye's coefficients
    D straighten it first (cv2.fisheye.undistortPoints), a pinhole (no D) takes K alone"""
    K = np.asarray(K, float).reshape(3, 3)
    if D is not None and len(D) == 4 and model == "equidistant" and np.abs(np.asarray(D, float)).max() > 0:
        import cv2
        xy = cv2.fisheye.undistortPoints(np.array([[[u, v]]], np.float64), K, np.asarray(D, float)).reshape(2)
    else:
        xy = np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1]])
    r = np.array([xy[0], xy[1], 1.0]); return r / np.linalg.norm(r)


def to_floor(u, v, T_wc, K, D=None, model="equidistant", floor_z=0.0, rmax=None):
    """pixel (u, v) on the floor plane z = floor_z: (x, y, range m) in the frame T_wc (world <- camera optical) is in,
    or None if its ray never meets the floor ahead (at or above the horizon) or lands beyond rmax (m, along the floor)"""
    T = np.asarray(T_wc, float); d = T[:3, :3] @ pixel_ray(u, v, K, D, model); C = T[:3, 3]
    if d[2] >= -1e-6: return None
    s = (floor_z - C[2]) / d[2]
    if s <= 0: return None
    p = C + s * d; rng = float(np.hypot(p[0] - C[0], p[1] - C[1]))
    if rmax is not None and rng > rmax: return None
    return float(p[0]), float(p[1]), rng


def project(xy, T_wc, K, D=None, model="equidistant", floor_z=0.0):
    """the inverse: a floor point (x, y) to its pixel (u, v) in the frame, or None behind the camera (for crops)"""
    T = np.asarray(T_wc, float); P = np.linalg.inv(T) @ np.array([xy[0], xy[1], floor_z, 1.0])
    if P[2] <= 0.05: return None
    K = np.asarray(K, float).reshape(3, 3)
    if D is not None and len(D) == 4 and model == "equidistant" and np.abs(np.asarray(D, float)).max() > 0:
        import cv2
        uv = cv2.fisheye.distortPoints(np.array([[[P[0] / P[2], P[1] / P[2]]]], np.float64), K, np.asarray(D, float)).reshape(2)
        return float(uv[0]), float(uv[1])
    return float(K[0, 0] * P[0] / P[2] + K[0, 2]), float(K[1, 1] * P[1] / P[2] + K[1, 2])


def place(det, frame, rmax=None):
    """a detection ({label, box, score?}) seen in a manifest frame: the floor point it stands on, in the odometry's
    frame, with the frame's time; None if it stands nowhere on the floor within rmax"""
    u, v = bottom_centre(det["box"])
    p = to_floor(u, v, frame["T_world_cam"], frame["K"], frame.get("D"), frame.get("model", "equidistant"), rmax=rmax)
    if p is None: return None
    return {"x": p[0], "y": p[1], "range_m": p[2], "ts": frame["ts"], "label": det.get("label", ""),
            "score": float(det.get("score", 1.0)), "box": [float(b) for b in det["box"]], "frame": frame["id"]}


def odom_to_true(traj_csv):
    """scoring only: a function (x, y, ts) -> (x, y) carrying a point from the odometry's frame at that moment to the
    sim's own, through the sim's path file (true and odometry poses, interpolated on the recording's clock; as
    bench_detect)"""
    T = np.genfromtxt(traj_csv, delimiter=",", names=True)
    U = {c: np.unwrap(T[c]) if c.endswith("yaw") else T[c] for c in ("true_x", "true_y", "true_yaw", "odom_x", "odom_y", "odom_yaw")}
    tt = T["wall_t"]
    def f(x, y, ts):
        it = lambda c: float(np.interp(ts, tt, U[c]))
        th = it("true_yaw") - it("odom_yaw"); c, s = np.cos(th), np.sin(th)
        px, py = x - it("odom_x"), y - it("odom_y")
        return float(c * px - s * py + it("true_x")), float(s * px + c * py + it("true_y"))
    def inv(x, y, ts):
        it = lambda c: float(np.interp(ts, tt, U[c]))
        th = it("odom_yaw") - it("true_yaw"); c, s = np.cos(th), np.sin(th)
        px, py = x - it("true_x"), y - it("true_y")
        return float(c * px - s * py + it("odom_x")), float(s * px + c * py + it("odom_y"))
    f.inverse = inv
    return f


def sees(frame, xy, rmax=None, margin=0.0):
    """whether a manifest frame sees floor point xy (its own frame's coordinates): inside the image (by margin px) and
    within rmax (m, along the floor) of the camera; its pixel, or None. Line of sight is not checked"""
    uv = project(xy, frame["T_world_cam"], frame["K"], frame.get("D"), frame.get("model", "equidistant"))
    if uv is None or not (margin <= uv[0] < frame["width"] - margin and margin <= uv[1] < frame["height"] - margin): return None
    C = frame["T_world_cam"]
    if rmax is not None and np.hypot(xy[0] - C[0][3], xy[1] - C[1][3]) > rmax: return None
    return uv
