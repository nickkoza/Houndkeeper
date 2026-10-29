"""The infrared wrist camera trial (waymark [6hI]): an Arducam B0332 (OV9281, mono, global shutter, no IR filter) where
the wrist camera is, and four ams OSRAM SFH 4181BS 940 nm LEDs round its lens, each fired for a frame of its own. On
with HK_WRIST_IR=1. A look is five frames: the room's own light, then each LED; each LED's light alone is its frame less
the first. Radiometry from the datasheets ([m52]); estimates marked."""
import os
import numpy as np, mujoco

ON = os.environ.get("HK_WRIST_IR", "0") == "1"
MODE = os.environ.get("HK_IR_MODE", "steady")   # "steady": the four LEDs on together, a flashlight (the
                                               # simpler choice); "flash": one LED a frame

# the camera (B0332 datasheet v1.0): 70 deg across 1280 px; 2 x 2 binned to 640 x 400
W, H = 640, 400
HFOV_DEG = 70.0
FOVY_DEG = float(np.degrees(2 * np.arctan(np.tan(np.radians(HFOV_DEG / 2)) * H / W)))   # 47.3 deg
PIXEL_M = 2 * 3e-6               # a binned pixel: 2 x 2 of 3 um
F_NUMBER = 2.4                   # est.: not published; M12 lenses of 2.8 mm run f/2.0-2.4, the unkind end
QE_940, QE_VIS = 0.12, 0.55      # est.: OmniVision publishes no curve ("best-in-class NIR"); see [m52]
READ_E = 4.0                     # est.: read noise, electrons (per binned pixel, summed charge)
FULL_E = 8000.0                  # est.: electrons at full scale (gain set so one native pixel's well fills the 10 bits)
BITS = 10
EXPOSURE_S = 0.001               # every frame of a look at the same fixed exposure and gain (flash frames must compare)
JPEG_Q = 75                      # MJPG, as the camera's fast modes deliver

# the LEDs (SFH 4181BS): 765 mW/sr typ. at 1 A, half angle 35 deg; output near proportional to current
LED_W_SR_PER_A = 0.765
LED_A = 1.0                      # drive current: its continuous rating (1 ms a frame, well inside the pulse limits)
LED_HALF_DEG = 35.0
LED_EXPONENT = float(np.log(0.5) / np.log(np.cos(np.radians(LED_HALF_DEG))))   # cos^n falls to half at 35 deg: n = 3.5
LED_CUTOFF_DEG = 60.0            # its datasheet pattern keeps ~15% out to 60-80 deg; beyond the camera's view anyway
LED_OFFSET_M = float(os.environ.get("HK_IR_LED_OFFSET", "0.035"))   # from the lens axis ([GWg], est.)
LAYOUT = os.environ.get("HK_IR_LAYOUT", "corners")   # "cross": above/below/right/left in the image; "corners": the diagonals
_OFFS = {"cross": ((0, 1), (0, -1), (1, 0), (-1, 0)),                  # (image right, image up) of each LED, unit length
         "corners": ((0.7071, 0.7071), (-0.7071, 0.7071), (0.7071, -0.7071), (-0.7071, -0.7071))}
PHOTON_J = 6.626e-34 * 2.998e8 / 940e-9

# the room's visible light, which the unfiltered camera sees too: an average floor (albedo 0.3) at ROOM_LUX in 1 ms
ROOM_LUX = float(os.environ.get("HK_ROOM_LUX", "70"))   # the lit nights' level (the unkind case: more light to subtract)
AMBIENT_E_AT_70LUX = 165.0       # est.: 70 lux of white LED light ~0.23 W/m2; albedo 0.3, f/2.4, 36 um2, 1 ms, QE 0.55

S_CLASSIC = 0.02                 # the classic renderer's light "diffuse" per LED: linear pixel = albedo x S x spot x cos / d2
NAMES = [f"ir_led_{k}" for k in range(4)]


def add_leds(body, cam_pos, x_, y_, f_):
    """the four LEDs on the wrist body round the lens at cam_pos (x_ right, y_ up in the image, f_ the view), pointing
    where the camera looks; off (only an infrared frame lights one)"""
    for k, (a, b) in enumerate(_OFFS[LAYOUT]):
        off = a * np.asarray(x_) + b * np.asarray(y_)
        L = body.add_light(); L.name = NAMES[k]; L.active = False; L.castshadow = True
        L.pos = list(np.asarray(cam_pos) + LED_OFFSET_M * np.asarray(off)); L.dir = list(f_)
        L.cutoff = LED_CUTOFF_DEG; L.exponent = LED_EXPONENT
        L.diffuse = [S_CLASSIC] * 3; L.ambient = [0, 0, 0]; L.specular = [0, 0, 0]
        L.attenuation = [0.0, 0.0, 1.0]                               # inverse square, as a point source is


def electrons_per_level():
    """electrons one unit of the classic renderer's linear pixel stands for: pixel = albedo x S x spot x cos / d2, and a
    Lambertian surface under irradiance E puts albedo x E x A / (4 N2) watts on a pixel"""
    return (LED_W_SR_PER_A * LED_A / S_CLASSIC) * PIXEL_M ** 2 / (4 * F_NUMBER ** 2) * EXPOSURE_S / PHOTON_J * QE_940


SHADOW_PX = 2048                 # the classic renderer's shadow map: a 1 mm shadow edge at 30 cm is under a pixel either way


def led_linear(m, d, cam, ks, r):
    """the light of LEDs `ks` alone, as the classic renderer `r` (the wrist capture's own, at this camera's size) draws it: a
    linear 0-1 pixel. Only `r`'s own scene is relit (its light list replaced by the one LED, no headlight); the model's
    lights are never switched, so no viewer drawing the same model can catch a flicker (nothing on screen may flash:
    [0Py], safe for photosensitive viewers)"""
    r.update_scene(d, cam)
    for g in range(r.scene.ngeom): r.scene.geoms[g].emission = 0.0
    for j, k in enumerate(ks):                                         # LEDs `ks` (indices), and no other light
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, "arm_" + NAMES[k])
        if i < 0: i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, NAMES[k])
        L = r.scene.lights[j]
        L.type = int(m.light_type[i]); L.headlight = 0; L.castshadow = 1; L.id = i
        L.pos[:] = d.light_xpos[i]; L.dir[:] = d.light_xdir[i]
        L.attenuation[:] = m.light_attenuation[i]; L.cutoff = float(m.light_cutoff[i]); L.exponent = float(m.light_exponent[i])
        L.ambient[:] = 0; L.diffuse[:] = m.light_diffuse[i]; L.specular[:] = 0
    r.scene.nlight = len(ks)
    r.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = 0; r.scene.flags[mujoco.mjtRndFlag.mjRND_FOG] = 0
    r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 1
    return r.render().astype(np.float32).mean(axis=2) / 255.0          # mono: the textures' brightness as their infrared albedo


def room_electrons(visible_rgb):
    """the room's own light on the sensor: the visible frame (auto-exposed to mid-grey) in mono, linearised, at ROOM_LUX"""
    lin = (visible_rgb.astype(np.float32) / 255.0) ** 2.2
    lum = lin @ np.array([0.2126, 0.7152, 0.0722], np.float32)
    return lum / 0.18 * 0.3 * AMBIENT_E_AT_70LUX / 0.3 * (ROOM_LUX / 70.0)


def sensor(e, rng):
    """electrons to what the camera delivers: shot noise, read noise, 10 bits, then 8-bit MJPG"""
    import cv2
    e = rng.poisson(np.clip(e, 0, None)).astype(np.float32) + rng.normal(0, READ_E, e.shape).astype(np.float32)
    dn = np.clip(np.round(e / FULL_E * (2 ** BITS - 1)), 0, 2 ** BITS - 1)
    img8 = (dn / 4).astype(np.uint8)
    ok, buf = cv2.imencode(".jpg", img8, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
    return cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)


def capture(m, d, cam, visible_rgb, renderer, rng=None):
    """one look's frames (8-bit mono, H x W): steady, one frame with all four LEDs on; flash, five: [room light, LED
    0..3], each at the same exposure. `renderer` a classic MuJoCo renderer at H x W"""
    rng = rng or np.random.default_rng()
    import cv2
    amb = room_electrons(cv2.resize(visible_rgb, (W, H), interpolation=cv2.INTER_AREA)); k_e = electrons_per_level()
    if MODE == "steady": return [sensor(amb + led_linear(m, d, cam, range(4), renderer) * k_e, rng)]   # the flashlight: one frame
    return [sensor(amb, rng)] + [sensor(amb + led_linear(m, d, cam, [k], renderer) * k_e, rng) for k in range(4)]


def led_image_dirs():
    """each LED's offset as seen in the image (unit vectors, x right, y down): a shadow falls on the far side"""
    return [np.array([a, -b], float) for a, b in _OFFS[LAYOUT]]            # image y runs down


def depth_edges(frames, ratio_drop=0.35, min_light=12):
    """multi-flash depth edges (Raskar et al. 2004): each LED's light alone (its frame less the room's), divided by the
    brightest of the four; stepping away from an LED, a sharp fall of that ratio is where something standing ends and
    its shadow begins. Returns (edges, shadow, max image)"""
    amb = frames[0].astype(np.float32); led = [np.clip(f.astype(np.float32) - amb, 0, None) for f in frames[1:]]
    mx = np.maximum.reduce(led); lit = mx > min_light
    edges = np.zeros(mx.shape, bool); shadow = np.zeros(mx.shape, bool)
    for L, dv in zip(led, led_image_dirs()):
        R = np.where(lit, L / np.maximum(mx, 1.0), 1.0)
        shadow |= lit & (R < 0.5)
        dx, dy = int(round(2 * dv[0])), int(round(2 * dv[1]))
        ahead = np.roll(R, (dy, dx), axis=(0, 1))                      # ahead[p] = R[p - 2 dv]: 2 px further from the LED
        edges |= lit & (R - ahead > ratio_drop)
    return edges, shadow, mx


MAX_SHARE = 0.45                 # an outline taking more of the frame than this has leaked into the floor: not used


def outline(frames, seed_mask, robot):
    """the whole item's pixels from the flash shadows ([6hI]): what stands on the floor is ringed by its shadows and
    depth edges, a printed floor pattern by nothing; so the region the detector's mask sits in, bounded by them, is the
    item, whole (a part the detector boxed grows to its toy). None if it can't be closed (it spreads over the floor)"""
    from scipy import ndimage as ndi
    edges, shadow, mx = depth_edges(frames)
    wall = ndi.binary_dilation(edges | shadow, iterations=1) | robot
    free = ~wall; lab, _ = ndi.label(free)
    seeds = lab[seed_mask & free]
    if not len(seeds): return None
    ids, counts = np.unique(seeds[seeds > 0], return_counts=True)
    if not len(ids): return None
    reg = lab == ids[np.argmax(counts)]
    if reg.mean() > MAX_SHARE: return None
    border = np.r_[reg[0], reg[-1], reg[:, 0], reg[:, -1]].mean()
    if border > 0.25: return None                                    # runs out along a quarter of the frame's edge: the floor
    return ndi.binary_fill_holes(ndi.binary_closing(reg, iterations=2)) & ~robot


def led_world(cv):
    """the four LEDs' positions in the world, from the camera's pose (c, R: columns x right, y up, z back)"""
    R = np.asarray(cv["R"]); c = np.asarray(cv["c"])
    return [c + LED_OFFSET_M * (a * R[:, 0] + b * R[:, 1]) for a, b in _OFFS[LAYOUT]]


def floor_agreement(frames, cv, floor_z=0.0):
    """how well each pixel fits the flat floor ([6hI]): a floor point's brightness under each LED is its albedo times what
    the geometry predicts (the LED's beam at that angle, the slant, the distance squared); divided by that prediction,
    the four LEDs agree on a floor point, and disagree on anything raised or tilted, whatever its print. Returns (spread,
    lit count): the spread of albedo estimates over the LEDs that light the pixel (std / mean), and how many do"""
    Hh, Ww = frames[0].shape; f = (Hh / 2) / np.tan(np.radians(cv["fovy"]) / 2)
    vv, uu = np.mgrid[0:Hh, 0:Ww]
    rays = np.stack([(uu - Ww / 2) / f, -(vv - Hh / 2) / f, -np.ones_like(uu, float)], -1) @ np.asarray(cv["R"]).T
    c = np.asarray(cv["c"]); t = (floor_z - c[2]) / np.where(np.abs(rays[..., 2]) < 1e-6, -1e-6, rays[..., 2])
    P = c + rays * t[..., None]                                          # each pixel's floor point
    axis = -np.asarray(cv["R"])[:, 2]                                    # the LEDs point where the camera looks
    amb = frames[0].astype(np.float32); est = []
    for pos, fr in zip(led_world(cv), frames[1:]):
        L = P - pos; dist = np.linalg.norm(L, axis=-1); Lu = L / dist[..., None]
        spot = np.clip(Lu @ axis, 0, 1) ** LED_EXPONENT; slant = np.clip(-Lu[..., 2], 0, 1)
        g = spot * slant / dist ** 2
        est.append(np.clip(fr.astype(np.float32) - amb, 0, None) / np.maximum(g, 1e-6))
    E = np.stack(est); lit = E > 0.25 * np.max(E, axis=0, keepdims=True)   # an LED that lights it (not shadowed)
    n = lit.sum(0); mean = (E * lit).sum(0) / np.maximum(n, 1)
    var = (((E - mean) ** 2) * lit).sum(0) / np.maximum(n, 1)
    return np.sqrt(var) / np.maximum(mean, 1e-6), n


OFF_FLOOR_SPREAD = 0.15          # a pixel whose LEDs disagree by more than this (std/mean) is off the floor (est.; the floor
                                 # sits at 0.03-0.08 in the first looks, 29 Sep, stripes and all)


def item_mask(frames, cv, seed_mask, robot):
    """the whole item's pixels: everything the flat floor doesn't explain ([6hI], floor_agreement), gaps closed and
    holes filled (a plush toy's flat back fits the floor's shading but is ringed by what doesn't), the piece the
    detector's mask falls in; None if nothing off the floor meets it"""
    from scipy import ndimage as ndi
    spread, n = floor_agreement(frames, cv)
    off = (spread > OFF_FLOOR_SPREAD) & (n >= 3) & ~robot
    off = ndi.binary_opening(off, iterations=1)                           # lone noisy pixels
    off = ndi.binary_fill_holes(ndi.binary_closing(off, iterations=3)) & ~robot
    lab, _ = ndi.label(off); ids = lab[seed_mask & off]
    if not len(ids): return None
    k = np.bincount(ids).argmax(); reg = lab == k
    return reg if reg.mean() <= MAX_SHARE else None
