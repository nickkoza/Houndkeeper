"""The robot's cameras rendered by Filament (mujofil) inside DimOS's sim process: physically based materials, ambient
occlusion, soft shadows and a camera that exposes like a real one ([CaM]; waymark [wjx]'s house lighting translated).

DimOS's sim calls `make(model, data)` when DIMOS_SIM_CAMERA_RENDERER=houndkeeper.filament_cameras:make and then
`render(data, camera_id)` for the head and wrist camera frames it publishes; its lidar depth cameras and the viewer stay
on MuJoCo's classic renderer. Needs mujofil (MuJoCo 3.9, the version DimOS's venv runs for it).

Lights: MuJoCo's own lights are kept out of Filament's scene and re-added in the renderer's units by what they are
(house lights by name, `make_nights.py`): the classic renderer's diffuse values were never physical. Ambient: none at
night, a sky light by day (a scene with window lights or a sun is a day). Exposure: every frame is metered and the exposure
moves toward mid-grey for the next (mujofil's set_exposure, tools/mujofil_pbr_maps.patch), as a camera's auto-exposure
does."""
from __future__ import annotations

import numpy as np

CALIBRATION = 14           # bump when UNITS, SUN, the day light, exposure rules or TOGGLES change: baselines are re-rendered
                            # 13 (27 Sep): frames carry the real camera's blur, rolling shutter, noise and compression (camera_imperfect)
UNITS = {"ceiling": 144000.0, "lamp": 160000.0, "window": 96000.0, "sun_bounce": 24000.0, "arm_wrist_light": 8340.0 / 0.04}   # per unit of diffuse, in the
DEFAULT_UNIT = 96000.0      # renderer's units: mujofil adds a small fixed fill light (~12 levels at +8 EV with no lights, 25 Sep);
                            # lights set well above it keep night exposures ~3 stops lower, so the fill doesn't haze the frame
SUN = 640000.0                                          # per unit of a directional light's diffuse (same scale as UNITS)
DAY_AMBIENT, DAY_SKY = 20000.0, 24000.0                 # the renderer's ambient and image-based sky light by day (same scale)
LIGHT_REACH_M = 30.0        # how far a lamp's light reaches (Filament's falloff: beyond it, no light at all). Past the
                            # house's far corner (~19 m), so light falls off by distance alone; 10 m cut it off (26 Sep)
NIGHT_AMBIENT = 100.0       # by night: the light bounced off walls and ceiling that Filament doesn't trace (est., set by eye on
                            # the house, 26 Sep: 30 hazes, 300 and up flattens the night toward day). Without it everything a
                            # lamp doesn't reach directly is black at any exposure: 53% of a night's frames, 26 Sep
EV_RANGE = (-3.0, 8.0)      # how far the camera's auto-exposure can go, in stops from EV_START's day: exposure 4 ms -> 33 ms
                            # (3 stops, params camera_imperfections) then gain to 16x (4 stops, est.: a phone-class sensor's
                            # analog gain tops out near 24 dB); darker than that the frame is dark, not noisier. It was 12:
                            # gain to 250x, the stowed wrist camera's frames all grain (27 Sep)
AE_STEP = 0.25              # most it moves in one frame, in stops, and AE_RATE the share of the way it moves (est.: a
AE_RATE = 0.15              # phone settles in ~0.3 s at 20 frames/s; a stop a frame looked like lights popping on, 26 Sep)
TARGET = 0.45                                           # auto-exposure aims the frame's mean at this (0-1): mid-grey, an 18%
                                                        # card in sRGB, as cameras meter (was 0.30, a stop dark, 26 Sep)
HIGHLIGHT_PCT, HIGHLIGHT_MAX = 2.0, 0.92                 # the highlights metering keeps: the brightest 2% at 92% at most (est.)
EV_START = {"day": 1.0, "night": 5.0}                   # where each camera's exposure starts (measured on the house, 25 Sep)
TOGGLES: dict = {"msaa_samples": 8, "bloom": True, "shadow_type": "vsm", "ssao_ssct": False,   # all on, as photographic as Filament
                  "ssao_resolution": 1.0, "ssao_bias": 0.005}              # goes. SSAO at full resolution and
                            # a larger bias: at half resolution it banded across surfaces seen the length of the house,
                            # flickering as the camera crept (it looked like z-fighting; 0.4-7% of pixels jumped at 1 mm steps)
IMPERFECT = __import__("os").environ.get("HK_CAMERA_IMPERFECT", "1") == "1"   # [x3f]: frames as a real camera delivers them
FISHEYE = __import__("os").environ.get("HK_FISHEYE", "1") == "1"   # the head camera through its real fisheye (lens.py)
BULB_M = {"ceiling": 0.12, "lamp": 0.05, "window": 0.15, "arm_wrist_light": 0.002}
# arm_wrist_light: Cree XP-G3 white (R4, 90 CRI, 4500 K), 130 lm min at 350 mA, ~Lambertian: 41 cd, 460 lux at 30 cm; Filament's
# SPOT gives intensity / pi candela, and the lit house's ~70 lux floor comes to ~4500 of its units: 64 a lux (29 Sep, [J7G]);
# per unit of the light's diffuse, which is 0.04 (go1_arm.WRIST_LIGHT_CLASSIC, set for the viewer): 8340 in all   # each light's size (m): the penumbra of its soft shadows (est.)


FIXED_MUJOFIL = __import__("houndkeeper.workspace").workspace.sibling("build") / "mujofil-f177-site"   # draws every lamp


def mujofil_dir() -> str:
    """the mujofil build to use: HK_MUJOFIL_DIR, else the one that draws every lamp (waymark 0007 [3my]: it was opt-in,
    and whatever ran without the variable fell back to the installed build, which draws one; 30 Sep)"""
    import os
    return os.environ.get("HK_MUJOFIL_DIR") or (str(FIXED_MUJOFIL) if FIXED_MUJOFIL.exists() else "")


def use_chosen_mujofil() -> None:
    """the chosen mujofil build (mujofil_dir) put first on the import path, so it is the one imported (waymark 0007 [3my])"""
    import sys
    d = mujofil_dir()
    if d and (not sys.path or sys.path[0] != d):
        if d in sys.path: sys.path.remove(d)
        sys.path.insert(0, d)


class FilamentCameras:
    """Filament renderers for one model at one image size: each camera keeps one renderer per regime (day, night). When
    the lighting changes (a new episode) the lights are swapped on the existing renderer, never rebuilt (rebuilding every
    episode churned GPU memory until allocations failed, 25 Sep); its exposure follows the scene frame by frame."""

    def __init__(self, model, data, width=640, height=360, render=None):
        import mujoco
        self.mujoco, self.m, self.out = mujoco, model, (width, height)
        self.size = render or (width, height)             # rendered larger when frames are made from it (the fisheye's centre
        self.head = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")   # is finer than a pinhole covering it)
        self.fisheye = False                               # DimOS's cameras (make) deliver the head camera's frames through its lens
        self.names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_LIGHT, i) or "" for i in range(model.nlight)]
        self.rs: dict = {}                                 # regime (day?) -> {"r": renderer, "lit": lighting seen, "ev": exposure set}
        self.cams: dict = {}                               # (camera id, regime) -> {"ev": that camera's exposure}
        self.lighting = 0                                  # bumped by relit(): renderers re-add their lights before the next frame

    def _day(self):
        m, mujoco = self.m, self.mujoco
        on = [i for i in range(m.nlight) if m.light_active[i]]   # a scene with window light or a sun on is a day
        return any(self.names[i].startswith(("window", "sun")) for i in on) or any(
            m.light_type[i] == mujoco.mjtLightType.mjLIGHT_DIRECTIONAL for i in on)

    def relit(self):
        self.lighting += 1

    def texture_changed(self, texid: int):
        """the model's texture `texid` was rewritten in tex_data (a new floor): every renderer re-uploads it"""
        for c in self.rs.values():
            up = getattr(c["r"], "update_texture", None)   # mujofil built with tools/mujofil_pbr_maps.patch; upstream's can't
            if up is not None: up(self.m, texid)

    def _meter(self, img):
        """stops to move so the frame's mean sits at TARGET, but no brighter than keeps its highlights: the brightest
        HIGHLIGHT_PCT of the frame at HIGHLIGHT_MAX at most, as a camera's metering protects them (a dark floor metered
        to mid-grey blew a beige carpet and a pale table to white, 30 Sep)"""
        a = img[::4, ::4].astype(np.float32) / 255.0; mean = float(a.mean())
        hi = float(np.percentile(a.max(-1), 100.0 - HIGHLIGHT_PCT))
        return float(min(np.log2(TARGET / max(mean, 1e-3)), np.log2(HIGHLIGHT_MAX / max(hi, 1e-3))))

    def _build(self, ev, day):
        check_lamps()
        import os
        os.environ.setdefault("MUJOFIL_BACKEND", "vulkan"); use_chosen_mujofil()   # headless Vulkan: DimOS's viewer holds a GLFW OpenGL context on
        import mujofil                                       # this thread, and mujofil's EGL can't make its own current (25 Sep)
        r = mujofil.WarpRenderer(preset="high", auto_ibl=False, width=self.size[0], height=self.size[1], exposure=ev, **TOGGLES)
        active = self.m.light_active.copy(); self.m.light_active[:] = 0    # keep MuJoCo's lights out of Filament's scene
        r.load_model(self.m); self.m.light_active[:] = active
        if day: r.load_default_ibl(with_skybox=False, intensity=DAY_SKY)
        self._lights(r, day)
        return r

    def _bounce(self):
        """the colour light takes bouncing round the room: the floor's and walls' mean colour (linear), at unit
        brightness. Filament traces no bounces, so the ambient is tinted with it: a beige room's shade is warm, not the
        sky's blue (26 Sep)"""
        m, mujoco = self.m, self.mujoco; cols = []
        for name in ("housefloor", "wall"):
            mid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_MATERIAL, name)
            if mid < 0: continue
            tid = int(m.mat_texid[mid][mujoco.mjtTextureRole.mjTEXROLE_RGB]); c = np.ones(3)
            if tid >= 0:
                a, n = int(m.tex_adr[tid]), int(m.tex_width[tid] * m.tex_height[tid] * m.tex_nchannel[tid])
                px = m.tex_data[a:a + n].reshape(-1, int(m.tex_nchannel[tid]))[::97, :3] / 255.0
                c = (px ** 2.2).mean(0)
            cols.append(c * m.mat_rgba[mid][:3] ** 2.2)
        if not cols: return (1.0, 1.0, 1.0)
        c = np.mean(cols, 0); c = c / max(float(c @ [0.2126, 0.7152, 0.0722]), 1e-6)
        return tuple(float(v) for v in c)

    def _lights(self, r, day, data=None):
        m, mujoco = self.m, self.mujoco
        r.clear_dynamic_lights(); r.set_ambient_intensity(DAY_AMBIENT if day else NIGHT_AMBIENT)
        r.set_ambient_tint(*self._bounce())
        for i, name in enumerate(self.names):
            if not m.light_active[i] or "ir_led" in name: continue         # only the lights that are on, and seen (not infrared)
            c = m.light_diffuse[i]; k = float(np.mean(c)); rgb = c / max(float(c.max()), 1e-6)
            if m.light_type[i] == mujoco.mjtLightType.mjLIGHT_DIRECTIONAL:
                r.add_directional_light(*m.light_dir[i], *rgb, k * SUN, bool(m.light_castshadow[i])); continue
            unit = next((u for pre, u in UNITS.items() if name.startswith(pre)), DEFAULT_UNIT)
            cut = float(m.light_cutoff[i])
            bulb = next((b for pre, b in BULB_M.items() if name.startswith(pre)), 0.0)
            if abs(float(m.light_bulbradius[i]) - 0.02) > 1e-6: bulb = float(m.light_bulbradius[i])   # a light that states its
                                                             # size (MJCF bulbradius; 0.02 is MuJoCo's default) keeps it
            pos, dir_ = (data.light_xpos[i], data.light_xdir[i]) if data is not None else (m.light_pos[i], m.light_dir[i])   # a light
            r.add_spot_light(*pos, *dir_, *rgb, k * unit, LIGHT_REACH_M, max(cut - 30.0, 5.0), cut,     # on the arm goes where it is
                             cast_shadows=bool(m.light_castshadow[i]), bulb_radius=bulb)

    def _moving(self, data):
        """the lights on the arm that are on, and where they are (rounded to a mm), or None"""
        m = self.m; on = [i for i in range(m.nlight) if m.light_active[i] and m.light_bodyid[i] != 0 and "ir_led" not in self.names[i]]
        return tuple((i, *np.round(data.light_xpos[i], 3), *np.round(data.light_xdir[i], 3)) for i in on) or None

    def _shot(self, r, data, camera_id):
        r.sync_transforms(self.m, data); r.sync_camera(self.m, data, camera_id)
        return np.asarray(r.render().cpu())[::-1, :, :3]   # mujofil's rows run bottom-up (OpenGL's order); the classic renderer's top-down

    def render(self, data, camera_id: int) -> np.ndarray:
        """one camera's frame (H x W x 3 uint8), auto-exposed as a camera does: each frame is metered and the exposure moves
        toward mid-grey by up to AE_STEP stops for the next, within EV_RANGE; the first frame is metered until it settles.
        All cameras share one renderer per regime (one copy of the scene's textures on the GPU: a copy per camera ran a
        night out of memory with the photographed house floor, 26 Sep); each keeps its own exposure"""
        day = self._day(); r = self.rs.get(day)
        if r is None:
            r = self.rs[day] = {"r": self._build(EV_START["day" if day else "night"], day), "lit": self.lighting, "ev": None}
        elif r["lit"] != self.lighting:                     # new lighting, same regime: swap the lights
            self._lights(r["r"], day, data); r["lit"] = self.lighting; r["moving"] = self._moving(data)
        moving = self._moving(data)                          # a light on the arm, on: re-placed whenever it's switched or has moved
        if moving != r.get("moving"):
            switched = bool(moving) != bool(r.get("moving"))
            self._lights(r["r"], day, data if moving else None); r["moving"] = moving
            if switched:                                     # switched on or off: every camera settles its exposure again, as a
                for key in [k_ for k_ in self.cams if k_[1] == day]: del self.cams[key]   # real one does while the light fades
        key = (camera_id, day); c = self.cams.get(key)
        def expose(ev):
            if r["ev"] != ev: r["r"].set_exposure(ev); r["ev"] = ev
        if c is None:
            c = self.cams[key] = {"ev": EV_START["day" if day else "night"]}
            for _ in range(4):                               # settle the first frame
                expose(c["ev"]); step = float(np.clip(self._meter(self._shot(r["r"], data, camera_id)), -6.0, 6.0))
                if abs(step) < 0.25: break
                c["ev"] = float(np.clip(c["ev"] + step, *EV_RANGE))
        expose(c["ev"]); img = self._shot(r["r"], data, camera_id)
        step = float(np.clip(AE_RATE * self._meter(img), -AE_STEP, AE_STEP))
        shown = c["ev"]
        if abs(step) > 0.1: c["ev"] = float(np.clip(c["ev"] + step, *EV_RANGE))   # the next frame's (a camera lags a frame too)
        optics = self._optics(camera_id)
        if optics is not None: img = optics(img)             # the lens (the head camera's fisheye) or the frame size, first:
        if IMPERFECT:                                        # then what the real camera adds at the frame's own size, blur
            from houndkeeper.camera_imperfect import Degrade   # from its motion over this exposure, rolling shutter, noise,
            if c.get("degrade") is None:                       # compression ([kJ6]); blur computed in the fisheye image itself
                from houndkeeper import lens
                fish = camera_id == self.head and self.fisheye
                name = self.mujoco.mj_id2name(self.m, self.mujoco.mjtObj.mjOBJ_CAMERA, camera_id) or ""
                elp = name.startswith("arm_wrist") and self.mujoco.mj_name2id(self.m, self.mujoco.mjtObj.mjOBJ_CAMERA, "arm_wrist_r") >= 0
                c["degrade"] = Degrade(float(self.m.cam_fovy[camera_id]), lens.calibration(*self.out) if fish else None,   # the stereo
                                       "maker_arm.elp_camera" if elp else "go2_air.camera_imperfections",   # pair's ELP boards ([A8K])
                                       -1.0 if name.endswith("_r") else 1.0)
            img = c["degrade"](img, float(data.time), data.cam_xpos[camera_id].copy(), data.cam_xmat[camera_id].reshape(3, 3).copy(), shown, EV_START["day"])
        return np.ascontiguousarray(img)

    def _optics(self, camera_id: int):
        """the rendered pinhole view to the published frame: the head camera through its real fisheye ([kJ6], lens.py),
        any other camera resized to the frame size; None when the render already is the frame"""
        from houndkeeper import lens
        W, H = self.out; fovy = float(self.m.cam_fovy[camera_id])
        if camera_id == self.head and self.fisheye: return lambda im: lens.to_fisheye(im, W, H, fovy)
        if self.size != self.out:
            import cv2; return lambda im: cv2.resize(im, (W, H), interpolation=cv2.INTER_AREA)
        return None


def make(model, data):
    """DimOS's sim cameras at its frame size (DIMOS_SIM_VIDEO_SIZE: the real head camera's 1280 x 720, params
    head_camera_px); rendered 1.5 times larger when the head camera's fisheye is made from them (960 x 540 for 640 x 360)"""
    from dimos.simulation.mujoco.constants import VIDEO_HEIGHT, VIDEO_WIDTH
    fc = FilamentCameras(model, data, VIDEO_WIDTH, VIDEO_HEIGHT, render=(VIDEO_WIDTH * 3 // 2, VIDEO_HEIGHT * 3 // 2) if FISHEYE else None)
    fc.fisheye = FISHEYE
    return fc


def enabled() -> bool:
    """the sims' cameras render through Filament unless HK_RENDERER=classic (the same switch as sim/dimos/env.py)"""
    import os
    return os.environ.get("HK_RENDERER", "filament") == "filament"


_BY_SIZE: dict = {}


def reset(model):
    """the model's lighting changed (a new episode): its renderers swap in the new lights before their next frame"""
    for key in [k for k in _BY_SIZE if k[0] == id(model) and _BY_SIZE[k].m is model]:
        _BY_SIZE[key].relit()


def texture_changed(model, texid: int):
    """the standalone sims rewrote a texture (floors.py's per-episode floor): Filament shows the new pixels, as
    mjr_uploadTexture does for the classic renderers"""
    for key in [k for k in _BY_SIZE if k[0] == id(model) and _BY_SIZE[k].m is model]:
        _BY_SIZE[key].texture_changed(texid)


def render(model, data, camera, width, height) -> np.ndarray:
    """a camera's frame at a given size, for the standalone sims (one renderer per model and size, reused)"""
    import mujoco
    key = (id(model), width, height)
    if key not in _BY_SIZE or _BY_SIZE[key].m is not model:           # the same model, not one at a reused address (29 Sep:
        _BY_SIZE[key] = FilamentCameras(model, data, width, height)   # an id()-keyed cache scrambled every toy's size)
    cid = camera if isinstance(camera, int) else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
    return _BY_SIZE[key].render(data, cid)


_LAMPS_OK = False


def _judge_lamps(a, b, both, dark):
    """lamp a lights the left half, lamp b the right ((left, right) means): together each half must be as lit as by
    its own lamp. A build that draws only one lamp (Filament 1.56.3, waymark 0007 [3my]) leaves one half dark"""
    lift = lambda alone, half: alone - dark < 1.0 or both[half] - dark >= 0.8 * (alone - dark)
    if not (lift(a[0], 0) and lift(b[1], 1)):
        raise RuntimeError(f"this mujofil build draws only one lamp at a time (waymark 0007 [3my]): lamp a {a}, lamp b {b}, "
                           f"both {both}, none {dark}. Use the build that draws them all: {FIXED_MUJOFIL}")


def check_lamps():
    """once a process, before the first render: two lamps on a bare floor, each lighting its half, must both show
    (_judge_lamps). Refuses a build that would light every house with one lamp, however it was imported"""
    global _LAMPS_OK
    if _LAMPS_OK: return
    import os, mujoco
    os.environ.setdefault("MUJOFIL_BACKEND", "vulkan"); use_chosen_mujofil(); import mujofil
    xml = ('<mujoco><worldbody><geom type="plane" size="6 6 .1" rgba=".6 .6 .6 1"/>'
           '<camera name="top" pos="0 0 6" xyaxes="1 0 0 0 1 0" fovy="80"/></worldbody></mujoco>')
    cb = mujoco.get_mjcb_control(); mujoco.set_mjcb_control(None)    # inside DimOS's sim a walking policy is MuJoCo's
    try:                                                              # global control callback: not for this bare scene
        m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    finally:
        mujoco.set_mjcb_control(cb)
    r = mujofil.WarpRenderer(preset="high", auto_ibl=False, width=160, height=80, exposure=4.0); r.load_model(m); r.set_ambient_intensity(0)
    def shot(xs):
        r.clear_dynamic_lights()
        for x in xs: r.add_spot_light(x, 0, 2, 0, 0, -1, 1, 1, 1, 160000, 30, 30, 60, cast_shadows=False)
        for _ in range(2): r.sync_transforms(m, d); r.sync_camera(m, d, 0); a = np.asarray(r.render().cpu())[:, :, :3].astype(float)
        return float(a[:, :80].mean()), float(a[:, 80:].mean())
    dark = shot([])[0]; _judge_lamps(shot([-2]), shot([2]), shot([-2, 2]), dark)
    del r; _LAMPS_OK = True
