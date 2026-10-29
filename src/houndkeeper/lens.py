"""The head camera's real lens, a fisheye ([nAH] kJ6; principle [x3f]): DimOS's calibration of a Go2's front camera,
equidistant model, four coefficients, 1280 x 720 (dimos/robot/unitree/go2/front_camera_720.yaml).

Two directions, one calibration:
- the sim renders a wide pinhole view and `fisheye_maps` warps it into what the real lens delivers;
- everything that places head-camera pixels on the floor first straightens them with `Rectifier`, into a pinhole
  view whose focal length is the fisheye's own at its centre (its resolution kept where it's finest; the fisheye's far
  corners, beyond ~39 deg either side, fall outside and are dropped).
The sim publishes the calibration as DimOS's camera_info (scaled to the published frame size), as the robot does."""
import math
from functools import lru_cache
from pathlib import Path

import numpy as np

from houndkeeper import workspace

CALIBRATION = workspace.dimos() / "dimos/robot/unitree/go2/front_camera_720.yaml"


def calibration(width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    """(K, D) at this frame size: focal lengths and principal point scale with the image, the coefficients don't"""
    import yaml
    c = yaml.safe_load(CALIBRATION.read_text())
    K = np.array(c["camera_matrix"]["data"], float).reshape(3, 3)
    K[0] *= width / c["image_width"]; K[1] *= height / c["image_height"]
    return K, np.array(c["distortion_coefficients"]["data"], float)


def render_fovy(width: int, height: int, margin: float = 1.02) -> float:
    """the vertical field of view (deg) a centred pinhole render needs to cover the whole fisheye frame"""
    import cv2
    K, D = calibration(width, height)
    edge = np.array([[[u, v]] for u in (0, width / 2, width - 1) for v in (0, height - 1)], np.float64)
    xy = cv2.fisheye.undistortPoints(edge, K, D).reshape(-1, 2)                     # normalised pinhole coordinates
    need_y = np.abs(xy[:, 1]).max(); need_x = np.abs(xy[:, 0]).max() * height / width
    return float(2 * math.degrees(math.atan(max(need_y, need_x) * margin)))


@lru_cache(maxsize=4)
def fisheye_maps(out_w: int, out_h: int, src_w: int, src_h: int, src_fovy: float):
    """remap tables: for each fisheye output pixel, where to read a centred pinhole render (src size, vertical fov)"""
    import cv2
    K, D = calibration(out_w, out_h)
    u, v = np.meshgrid(np.arange(out_w, dtype=np.float64), np.arange(out_h, dtype=np.float64))
    xy = cv2.fisheye.undistortPoints(np.stack([u, v], -1).reshape(-1, 1, 2), K, D).reshape(out_h, out_w, 2)
    f = (src_h / 2) / math.tan(math.radians(src_fovy) / 2)
    return (xy[..., 0] * f + src_w / 2 - 0.5).astype(np.float32), (xy[..., 1] * f + src_h / 2 - 0.5).astype(np.float32)


def to_fisheye(img: np.ndarray, out_w: int, out_h: int, src_fovy: float) -> np.ndarray:
    """a centred pinhole render (any size, this vertical fov) as the real lens would have delivered it"""
    import cv2
    mx, my = fisheye_maps(out_w, out_h, img.shape[1], img.shape[0], round(src_fovy, 4))
    return cv2.remap(img, mx, my, cv2.INTER_AREA if img.shape[1] > 2 * out_w else cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


class Rectifier:
    """straightens a fisheye frame into a pinhole view with the fisheye's own focal length and principal point.
    `K` is that pinhole's (fx, fy, cx, cy); frames without distortion pass through unchanged."""

    def __init__(self, K: np.ndarray, D, width: int, height: int, model: str = "equidistant"):
        self.size = (width, height); K = np.asarray(K, float).reshape(3, 3); D = np.asarray(D, float).ravel()
        self.active = model == "equidistant" and len(D) == 4 and np.abs(D).max() > 0
        self.K = (K[0, 0], K[1, 1], K[0, 2], K[1, 2])
        self.D = D if self.active else None          # the fisheye's coefficients: projecting onto the raw frame (mind.project)
        if self.active:
            import cv2
            self.maps = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (width, height), cv2.CV_32FC1)

    @classmethod
    def from_camera_info(cls, ci) -> "Rectifier":
        return cls(np.array(ci.K).reshape(3, 3) if hasattr(ci, "K") else ci.get_K_matrix(), getattr(ci, "D", []) or [],
                   ci.width, ci.height, getattr(ci, "distortion_model", "") or "")

    def __call__(self, img: np.ndarray) -> np.ndarray:
        if not self.active: return img
        import cv2
        return cv2.remap(np.ascontiguousarray(img), *self.maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def fisheye_on() -> bool:
    """the sim's head camera delivers the real lens's fisheye frames (on unless HK_FISHEYE=0); the robot's always does"""
    import os
    return os.environ.get("HK_FISHEYE", "1") == "1"


def head(width: int, height: int) -> Rectifier:
    """the head camera's lens, the one place every consumer takes it from ([4hm]): the fisheye's calibration, or with
    HK_FISHEYE=0 the pinhole params/robot.yaml gives. Its `K` is the straightened view's; call it on each frame. (The
    robot's kinematic copy carries MuJoCo's stock 45 deg head camera: taken from there, live floor points sat 13% out,
    ~26 cm at 2 m, 27 Sep)"""
    if fisheye_on():
        K, D = calibration(width, height); return Rectifier(K, D, width, height)
    from houndkeeper import params
    f = (height / 2) / math.tan(math.radians(params.get("go2_air.head_camera_fovy_deg")) / 2)   # a centred render: its
    return Rectifier(np.array([[f, 0, width / 2 - 0.5], [0, f, height / 2 - 0.5], [0, 0, 1.0]]), [], width, height, "")   # centre
                                                                     # in OpenCV's convention, integers at pixel centres ([AKY])
