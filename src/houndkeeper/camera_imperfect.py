"""What a real camera does to a frame that a renderer doesn't ([kJ6] in waymark [nAH]; principle [x3f]): the frame is
exposed over a time while the camera moves (motion blur), read out row by row while it moves (rolling shutter), amplified
at night (noise), and compressed for the video stream. Applied to every head and wrist frame the sim publishes, after
the renderer's own auto-exposure, sized from the exposure that auto-exposure chose.

The camera's motion is its pose now against its pose at its previous frame (the sim's own state). Each pixel's motion
over the exposure comes from where it looks: the floor plane where its ray meets the floor, a far wall (FAR_M) where it
doesn't. Rotation dominates (a head bob of a few deg/s moves every pixel alike); translation matters close in.
All sizes are estimates until the real camera is measured (real/calibration/head_camera.md)."""
import math

import numpy as np

from houndkeeper import params as P

FAR_M = 3.0        # depth taken for pixels whose ray doesn't meet the floor (est.: the room's walls)
SAMPLES = 7        # sub-exposure samples averaged for the blur
GRID_PX = 16       # the motion is worked out every this many pixels and interpolated between
UY, UX = np.mgrid[0:1440, 0:2560].astype(np.float32)   # pixel coordinates, cut to each frame's size


def settings(profile: str = "go2_air.camera_imperfections") -> dict:
    return P.get(profile)


def lens_roundtrip(img, f: float, k1: float, k2: float, k1_err: float):
    """what a wide lens's distortion leaves after a calibration removes it (waymark 0011 [A8K]): the rectified image is
    stretched where the lens squeezed it, so it is softer there (the local stretch 1 / (1 + 3 k1 r2 + 5 k2 r4), r the
    radius over the focal length), and the calibration's k1 is off by k1_err of itself, so points sit slightly off"""
    import cv2
    H, W = img.shape[:2]; y, x = UY[:H, :W] - H / 2 + 0.5, UX[:H, :W] - W / 2 + 0.5; r2 = (x * x + y * y) / (f * f)
    scale = np.clip(1 + 3 * k1 * r2 + 5 * k2 * r2 * r2, 0.25, 1.0)
    sig = np.clip(0.5 * (1 / scale - 1), 0, 3)                                 # px of blur the stretch amounts to
    levels = [img.astype(np.float32)] + [cv2.GaussianBlur(img.astype(np.float32), (0, 0), s_) for s_ in (0.75, 1.5, 3.0)]
    w = np.interp(sig, [0, 0.75, 1.5, 3.0], [0, 1, 2, 3]); i0 = np.floor(w).astype(int); a = w - i0
    out = np.zeros_like(levels[0]); ch = (lambda x: x[..., None]) if img.ndim == 3 else (lambda x: x)   # colour or grey
    for k in range(4):
        out += levels[k] * ch((i0 == k) * (1 - a) + (i0 + 1 == k) * a)
    d = 1 + k1 * k1_err * r2                                                     # the calibration's residual, radially
    return cv2.remap(out, (x * d + W / 2 - 0.5).astype(np.float32), (y * d + H / 2 - 0.5).astype(np.float32), cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


class Degrade:
    """one per camera: remembers the camera's last pose to know how it is moving"""

    def __init__(self, fovy_deg: float, lens=None, profile: str = "go2_air.camera_imperfections", lens_err_sign: float = 1.0):
        self.fovy = fovy_deg; self.lens = lens                                 # lens: (K, D) of an equidistant fisheye frame
        self.last = None; self.rng = np.random.default_rng(0); self.s = settings(profile); self.err_sign = lens_err_sign
        self.dip_until, self.t_last = -1.0, 0.0                                # a bitrate dip in progress, until then
        self.noise: dict = {}

    def exposure(self, ev: float, ev_day: float) -> tuple[float, float]:
        """(exposure time s, gain) for the auto-exposure's amplification 2**ev: the time lengthens first, up to the
        frame's time; past that the sensor's gain rises (and with it the noise)"""
        s = self.s; amp = 2.0 ** (ev - ev_day)
        t = float(np.clip(s["exposure_day_s"] * amp, s["exposure_day_s"], s["exposure_max_s"]))
        return t, max(1.0, s["exposure_day_s"] * amp / t)

    def __call__(self, img: np.ndarray, t: float, C: np.ndarray, R: np.ndarray, ev: float, ev_day: float) -> np.ndarray:
        """img: the frame as the lens delivers it (H x W x 3 uint8: a pinhole view at this camera's fovy, or with `lens`
        the fisheye); t: sim time; C, R: the camera's position and rotation (MuJoCo's camera frame: x right, y up,
        looking down -z) in the world. The blur is where the scene moves in this image over the exposure"""
        import cv2
        s = self.s; H, W = img.shape[:2]; f = (H / 2) / math.tan(math.radians(self.fovy) / 2)
        T_exp, gain = self.exposure(ev, ev_day)
        out = img.astype(np.float32)
        if s.get("lens_k1"):                                                  # a wide lens, calibrated away (the stereo pair's)
            out = lens_roundtrip(out, f, s["lens_k1"], s.get("lens_k2", 0.0), s.get("lens_k1_err", 0.0) * self.err_sign)
        if self.last is not None and 1e-4 < t - self.last[0] < 0.5:
            dt = t - self.last[0]; v = (C - self.last[1]) / dt
            dR = R @ self.last[2].T; w = _rotvec(dR) / dt                    # world angular velocity (rad/s)
            g = GRID_PX; u, vv = np.meshgrid(np.arange(0, W + g, g, dtype=np.float32) + 0.5, np.arange(0, H + g, g, dtype=np.float32) + 0.5)   # coarse: motion is smooth
            ray = self._rays(u - 0.5, vv - 0.5, W, H, f) @ R.T              # world directions
            depth = np.where(ray[..., 2] < -1e-3, -C[2] / np.minimum(ray[..., 2], -1e-3), FAR_M)
            X = C + ray * np.minimum(depth, FAR_M * 3)[..., None]              # the point each pixel sees
            rs = s["rolling_shutter_s"] * ((vv / H) - 0.5)                    # each row's own moment
            def where(tk):                                                     # where each grid point's scene point lies at time tk
                Rk = _rotate(R, w, tk); p = np.einsum("hwij,hwj->hwi", np.transpose(Rk, (0, 1, 3, 2)), X - (C + v * tk[..., None]))
                return self._project(p, W, H, f)
            (a0, b0), (a1, b1) = where(rs - T_exp / 2), where(rs + T_exp / 2)  # the exposure's two ends: how far it smears
            smear = float(np.hypot(a1 - a0, b1 - b0).max())
            n_s = int(np.clip(np.ceil(smear), 1, SAMPLES))                     # a sample a pixel of smear (none still)
            acc = np.zeros_like(out)
            for k in range(n_s):
                tk = rs + T_exp * ((k + 0.5) / n_s - 0.5)                      # this sample's time offset, per row
                mu, mv = where(tk)                                             # the camera moved and turned by then
                # the frame shows at pixel u what the (rendered, still) camera saw at the point that sample puts there:
                # pull from where that point lies in the rendered frame, displaced the opposite way
                fu = cv2.resize(mu - (u - 0.5), (W, H), interpolation=cv2.INTER_LINEAR)   # each pixel's displacement, full size
                fv = cv2.resize(mv - (vv - 0.5), (W, H), interpolation=cv2.INTER_LINEAR)
                acc += cv2.remap(out, UX[:H, :W] - fu, UY[:H, :W] - fv, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
            out = acc / n_s
        self.last = (t, C.copy(), R.copy())
        sigma = np.sqrt(np.float32(s["shot_noise"] * gain * 255.0) * out + np.float32((s["read_noise"] * gain * 255.0) ** 2))   # sensor
                                                                               # noise: shot and read, grown by gain (DN)
        blur = float(s.get("isp_noise_blur_px", 0.0)); keep = np.float32(s.get("isp_noise_keep", 1.0))
        key = (out.shape, blur); bank = self.noise.get(key)                   # unit noise fields, drawn once per frame size:
        if bank is None:
            bank = self.rng.standard_normal((4,) + out.shape, dtype=np.float32)
            if blur > 0:                                                      # what the camera's own noise reduction leaves:
                bank = np.stack([cv2.GaussianBlur(b, (0, 0), blur) for b in bank])   # blotches, not single pixels ([kJ6];
                bank /= bank.std()                                            # a real stream is denoised before it's encoded)
            self.noise[key] = bank
        z = bank[self.rng.integers(4)] * np.float32(1 if self.rng.random() < 0.5 else -1)   # one a frame, either sign
        out = np.clip(out + z * sigma * keep, 0, 255).astype(np.uint8)       # keep: the share of the raw noise that survives
        q = s["jpeg_quality"]
        if s.get("dip_per_min"):                                               # the link congests now and then: the encoder
            if t >= self.dip_until and self.rng.random() < 1 - math.exp(-s["dip_per_min"] / 60.0 * max(0.0, t - self.t_last)):
                self.dip_until = t + self.rng.exponential(s["dip_s"])          # drops its bitrate for a few seconds
            if t < self.dip_until: q = s["jpeg_low"]
        self.t_last = t
        if q < 100:                                                            # the stream's compression (a per-frame
            ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, int(q)])   # stand-in for H.264)
            if ok: out = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
        return np.ascontiguousarray(out)


    def _rays(self, u, v, W, H, f):
        """camera-frame directions (x right, y up, -z forward) of pixel positions u, v (pixel centres at integers)"""
        if self.lens is None:
            return np.stack([(u + 0.5 - W / 2) / f, -(v + 0.5 - H / 2) / f, -np.ones_like(u)], -1).astype(np.float32)
        import cv2
        key = (u.shape, W, H)
        if getattr(self, "_ray_key", None) != key:                             # the grid is the same every frame: once
            K, D = self.lens
            xy = cv2.fisheye.undistortPoints(np.stack([u, v], -1).reshape(-1, 1, 2).astype(np.float64), K, D).reshape(u.shape + (2,))
            self._ray_cache, self._ray_key = np.stack([xy[..., 0], -xy[..., 1], -np.ones_like(u)], -1).astype(np.float32), key
        return self._ray_cache

    def _project(self, p, W, H, f):
        """pixel positions (pixel centres at integers) of camera-frame points p"""
        z = np.minimum(p[..., 2], -1e-3); a, b = p[..., 0] / -z, -p[..., 1] / -z   # optical: x right, y down, z forward
        if self.lens is None:
            return (W / 2 + f * a - 0.5).astype(np.float32), (H / 2 + f * b - 0.5).astype(np.float32)
        K, D = self.lens; r = np.hypot(a, b); th = np.arctan(r); t2 = th * th
        thd = th * (1 + D[0] * t2 + D[1] * t2 ** 2 + D[2] * t2 ** 3 + D[3] * t2 ** 4); sc = np.where(r > 1e-9, thd / np.maximum(r, 1e-9), 1.0)
        return (K[0, 0] * sc * a + K[0, 2]).astype(np.float32), (K[1, 1] * sc * b + K[1, 2]).astype(np.float32)


def _rotvec(Rm: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(Rm).as_rotvec()


def _rotate(R: np.ndarray, w: np.ndarray, t: np.ndarray) -> np.ndarray:
    """R turned by the world angular velocity w for each time in t (H x W): a small-angle rotation per pixel"""
    th = t[..., None] * w                                                     # H x W x 3 rotation vectors (small)
    K = np.zeros(th.shape[:-1] + (3, 3), np.float32)
    K[..., 0, 1], K[..., 0, 2], K[..., 1, 2] = -th[..., 2], th[..., 1], -th[..., 0]
    K[..., 1, 0], K[..., 2, 0], K[..., 2, 1] = th[..., 2], -th[..., 1], th[..., 0]
    return (np.eye(3, dtype=np.float32) + K) @ R.astype(np.float32)
