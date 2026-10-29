"""Look-closer geometry from the wrist-camera image alone: find what stands out from the carpet (same relative colour /
brightness features as the floor detector), project it onto the floor, and return its centre and long axis."""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import os, sys
sys.path.insert(0, str(_HK / "src")); from houndkeeper import workspace   # noqa: E402  (decision [cy4])
_DIMOS = str(workspace.dimos())   # the DimOS checkout beside this repo
import numpy as np, mujoco
import jaws as jw
sys.path.insert(0, str(_HK / "src" / "houndkeeper"))
from scipy import ndimage as ndi
sys.path.insert(0, str(_HK / "sim" / "house"))
_SEG = {}
def capture(m, d, cam="arm_wrist", H=480, W=640, filament=None):
    """One wrist-camera look: the image, which pixels are the robot itself, and the camera pose (what the real robot has
    from its camera, its URDF and its joint angles)."""
    import go1_arm
    if go1_arm.WRIST_STEREO:                                  # the stereo pair ([A8K]): 1280 x 720 whole (full
        H, W = (720, 1280) if go1_arm.STEREO_FULL else (360, 640)   # resolution), or binned 2 x 2
    import ir_camera
    if ir_camera.ON:                                          # the infrared trial ([6hI]): the camera's own frame size, and
        H, W = ir_camera.H, ir_camera.W                       # its LED frames drawn by this capture's own classic renderer
        m.vis.quality.shadowsize = ir_camera.SHADOW_PX        # (a third GL context ran the GPU out of memory, 29 Sep)
    key = (id(m), H, W)
    if _SEG.get("key") != key or _SEG.get("model") is not m:   # renderers per model, reused (creating one per look was slow);
        _SEG["model"] = m                                       # the same model, not one at a reused address (29 Sep)
        _SEG["rgb"] = mujoco.Renderer(m, H, W)
        _SEG["r"] = mujoco.Renderer(m, H, W); _SEG["r"].enable_segmentation_rendering(); _SEG["key"] = key
        root = m.body_rootid[m.body("trunk").id]                # every geom of the robot itself: arm, legs, body, bin
        _SEG["robot_g"] = np.array([i for i in range(m.ngeom) if m.body_rootid[m.geom_bodyid[i]] == root])
    cid = m.camera(cam).id
    if _SEG.get("tex_names", (None,))[0] is not m: _SEG["tex_names"] = (m, [m.texture(i).name for i in range(m.ntex)])
    if "local_floor" in _SEG["tex_names"][1]:
        import floors; floors.view_from(m, d.cam_xpos[cid], renderers=(_SEG["rgb"],), viewer=_SEG.get("viewer"))   # carpet pile looks different from each side
    import filament_cameras as fc                                            # Filament unless HK_RENDERER=classic
    if (fc.enabled() if filament is None else filament): img = fc.render(m, d, cam, W, H)
    else: _SEG["rgb"].update_scene(d, cam); img = _SEG["rgb"].render().copy()
    _SEG["r"].update_scene(d, cam); seg = _SEG["r"].render()[..., 0]
    cid = m.camera(cam).id
    v = {"img": img, "robot": np.isin(seg, _SEG["robot_g"]), "c": d.cam_xpos[cid].copy(), "R": d.cam_xmat[cid].reshape(3, 3).copy(), "fovy": float(m.cam_fovy[cid]),
         "seg": seg}                                                    # the sim's truth, per pixel (benches only: the real camera has none)
    if ir_camera.ON: v = ir_look(m, d, cam, v)
    if go1_arm.WRIST_STEREO:                                  # the right camera's frame, and where the robot believes it is:
        cr = m.camera(cam + "_r").id                          # the left camera's pose, a baseline along its x (as calibrated)
        v["img_r"] = fc.render(m, d, cam + "_r", W, H) if (fc.enabled() if filament is None else filament) else _classic(m, d, cam + "_r")
        v["baseline"] = go1_arm.STEREO_BASELINE_M; v["c_r_true"] = d.cam_xpos[cr].copy()
        import stereo_eval                                    # the height map the pair measures: the close look uses it ([GMs]),
        v["stereo_h"] = stereo_eval.heights(v); v["stereo_P"] = stereo_eval.heights.last_points   # so a failure is loud (29 Sep:
                                                              # one swallowed here left a whole live bench on the detector alone)
    return v


def _classic(m, d, cam):
    _SEG["rgb"].update_scene(d, cam); return _SEG["rgb"].render().copy()


def ir_look(m, d, cam, v):
    """the infrared trial's look (waymark [6hI]): its five frames, the detector's image (the four LEDs' brightest, grey
    in three channels: as lit, shadows filled by the other LEDs), and the robot's pixels at the camera's own size"""
    import cv2, ir_camera as ir
    frames = ir.capture(m, d, cam, v["img"], _SEG["rgb"])
    if len(frames) == 1: grey = frames[0]                                   # steady: the flashlight's frame as it is
    else: _, _, mx = ir.depth_edges(frames); grey = np.clip(mx + frames[0], 0, 255).astype(np.uint8)
    return dict(v, img=np.repeat(grey[..., None], 3, axis=2), ir_frames=frames)


def refine(m, d, cam="arm_wrist", H=480, W=640, z_plane=0.015, renderer=None, aim=None, how="detect6"):
    """aim: floor point where the item is expected (default: the view centre)"""
    v = capture(m, d, cam, H, W); v["aim"] = aim; return globals()[how](v, z_plane), v["img"]


def fuse(views, gate=0.03):
    """Triangulate one item from several looks: the 3D point nearest every look's ray through the blob centre (least
    squares). A look whose ray misses the fused point by more than gate saw something else and is dropped. Returns
    (xyz, kept views); xyz is None with fewer than two agreeing views."""
    vs_ = list(views)
    while len(vs_) >= 2:
        A = np.zeros((3, 3)); b = np.zeros(3)
        for v in vs_:
            P = np.eye(3) - np.outer(v["ray"], v["ray"]); A += P; b += P @ v["cam"]
        x = np.linalg.solve(A + 1e-9 * np.eye(3), b)
        miss = [np.linalg.norm((np.eye(3) - np.outer(v["ray"], v["ray"])) @ (x - v["cam"])) for v in vs_]
        if max(miss) <= gate: return x, vs_
        vs_.pop(int(np.argmax(miss)))
    return None, vs_


_SAM = {}


STRIP = 0.02                     # the jaws close on the item within +-2 cm of the grasp point (8 mm fingers + where it may be off)
def grip_options(P2, strip=STRIP, step=5, slides=(0.04,)):
    """Ways to grip an item from its footprint on the floor (N x 2 points, m). For each jaw closing direction: the grasp
    point on the item nearest its centre, the width the jaws meet in a finger-wide strip there (5th-95th percentile), and
    that strip's middle. A curved or L-shaped item (a rope, a tail) is gripped across one part, not over its empty centre.
    Returns [(width, jaw unit vector, grasp point offset from the centroid)], narrowest first."""
    ctr = P2.mean(0); g0 = P2[np.argmin(np.linalg.norm(P2 - ctr, axis=1))]    # the item point nearest the centroid
    mid, slid = [], []
    for deg in range(0, 180, step):
        a_ = np.radians(deg); jaw = np.array([np.cos(a_), np.sin(a_)]); along = np.array([-jaw[1], jaw[0]]); t = (P2 - g0) @ along
        for slide in (0.0,) + tuple(v for s_ in slides for v in (-s_, s_)):    # a long item may be gripped 4 cm either side of its
            if slide and not (t.min() <= slide - strip if slide < 0 else t.max() >= slide + strip):   # middle, but only where it
                continue                                                       # runs on that far (not the rim of a ball)
            g = g0 + along * slide; sel = P2[np.abs((P2 - g) @ along) < strip]
            if len(sel) < 10: sel = P2
            u = (sel - g) @ jaw; lo, hi = np.percentile(u, 5), np.percentile(u, 95)
            if not (lo <= 0 <= hi):                                            # the strip's part holding g: one strand, not the next
                lo, hi = min(lo, 0.0), max(hi, 0.0)
            (slid if slide else mid).append((float(hi - lo), jaw, g - ctr + jaw * (lo + hi) / 2))
    return sorted(mid, key=lambda o: o[0]) + sorted(slid, key=lambda o: o[0])   # the middle first; sliding only if it is out of reach


PART_GAP_M = 0.025   # a gap the jaws' fingers fit through (est.: a finger ~2 cm thick): parts either side are gripped apart
PART_STEADY_M, PART_TAPER_M = 0.015, 0.012   # a grippable part keeps its width (within 1.2 cm) 1.5 cm either side of the
                                             # grasp (est.): a leg does; a box's corner or a tapering hand doesn't


def part_grip_options(P2, strip=STRIP, step=10, points=40, gap=PART_GAP_M):
    """ways to grip one part of an item (a leg, an arm): grasp points spread over its footprint, and for each jaw
    direction only the run of the item under the grasp point, split where a finger-wide gap opens (a figure lying with
    its legs apart: every strip through its middle spans both legs, one leg is 5 cm, 28 Sep). Same form as grip_options,
    offsets from the centroid; narrowest first"""
    ctr = P2.mean(0); rng = np.random.default_rng(0)
    G = P2[rng.choice(len(P2), min(points, len(P2)), replace=False)]; out = []
    def run_at(g, jaw, along):
        sel = P2[np.abs((P2 - g) @ along) < strip]
        if len(sel) < 10: return None
        u = np.sort((sel - g) @ jaw); k0 = int(np.searchsorted(u, 0.0)); cut = np.nonzero(np.diff(u) > gap)[0]
        lo_i = int(cut[cut < k0].max()) + 1 if (cut < k0).any() else 0
        hi_i = int(cut[cut >= k0].min()) if (cut >= k0).any() else len(u) - 1
        run = u[lo_i:hi_i + 1]
        return None if len(run) < 10 else (np.percentile(run, 5), np.percentile(run, 95))
    for g in G:
        for deg in range(0, 180, step):
            a_ = np.radians(deg); jaw = np.array([np.cos(a_), np.sin(a_)]); along = np.array([-jaw[1], jaw[0]])
            r0 = run_at(g, jaw, along)
            if r0 is None: continue
            side = [run_at(g + along * d, jaw, along) for d in (-PART_STEADY_M, PART_STEADY_M)]
            if any(r is None for r in side): continue                       # the part must run on either side of the grasp,
            ws = [r0[1] - r0[0]] + [r[1] - r[0] for r in side]              # its sides near parallel: a leg, not a corner
            if max(ws) - min(ws) > PART_TAPER_M: continue                   # or a tapering tip
            lo, hi = r0
            out.append((float(hi - lo), jaw, g - ctr + jaw * (lo + hi) / 2))
    return sorted(out, key=lambda o: o[0] + 0.5 * float(np.linalg.norm(o[2])))   # narrow, and nearer the middle


QUERIES = ["a toy", "a stuffed animal", "a sock", "a shoe", "a ball", "a pencil", "a piece of paper", "a tissue",
           "a piece of cloth", "a rope", "a toy car", "a toy animal", "a block", "a box", "a game case", "a plastic object",
           "a small object", "a computer mouse", "a roll of tape", "a cup", "a mug", "a bowl", "a towel", "a glove",
           "scissors", "crayons", "a pencil case"]   # what's left on a home's floor besides toys (28 Sep: a mouse and a roll
                                                 # of tape went unseen, the list asking only for toys and cloth)
_OWL = {}
OWL_THR = float(os.environ.get("HK_OWL_THR", "0.05"))   # OWLv2's cut-off: 0.08 was set on classic frames; on Filament's dim
                                                      # night house a visible toy scored 0.07 and was missed (26 Sep)


MAX_ITEM_M = 0.60   # a detection bigger than this on the floor is not a toy (the largest too-big ones run ~40 cm; look-closer [pYZ])
WHOLE_BOX = os.environ.get("HK_WHOLE_BOX", "0") == "1"
WHOLE_COVER, WHOLE_GROW = 0.8, 1.5   # a box holding 80% of the chosen one and 1.5x its area is the thing it is part of (est.)


def whole_box(best, dets, cover=WHOLE_COVER, grow=WHOLE_GROW):
    """the detection a chosen box is part of: the largest box that holds most of it and is clearly bigger (29 Sep: on
    big toys the box nearest the flag was a puzzle's piece, a bus's wheel, a turtle's head, so a 31 x 24 cm board
    measured 7 x 5 cm and the robot tried it); else the chosen box. Boxes are (x1, y1, x2, y2) in `.bbox`"""
    def area(b): return max(b[2] - b[0], 0) * max(b[3] - b[1], 0)
    b0 = best.bbox; a0 = area(b0)
    def holds(dd):
        b = dd.bbox; inter = area((max(b[0], b0[0]), max(b[1], b0[1]), min(b[2], b0[2]), min(b[3], b0[3])))
        return dd is not best and a0 > 0 and inter >= cover * a0 and area(b) >= grow * a0
    outer = [dd for dd in dets if holds(dd)]
    return max(outer, key=lambda dd: area(dd.bbox)) if outer else best


def detect6(v, z_plane=0.015, queries=QUERIES, thr=None, max_dist=0.15):
    """Grounded segmentation: an open-vocabulary detector (OWLv2, local, in DimOS) proposes whole objects from a list
    of things people leave on floors, with the robot's own pixels painted out; the best box near where the item is
    expected is cut out by EdgeTAM (box prompt). The mask gives centre and grip axis."""
    import cv2
    if "d" not in _OWL:
        sys.path.insert(0, _DIMOS); from dimos.perception.detection.detectors.owlv2 import Owlv2Detector
        _OWL["d"] = Owlv2Detector()
    if "s" not in _SAM:
        from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter; _SAM["s"] = EdgeTAMImageSegmenter()
    from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
    img = v["img"]; H, W = img.shape[:2]; c = v["c"]; R = v["R"]; f = (H / 2) / np.tan(np.radians(v["fovy"]) / 2)
    rob = ndi.binary_dilation(v["robot"], iterations=4)
    small = cv2.resize(img, (W // 2, H // 2)); rs_ = cv2.resize(rob.astype(np.uint8), (W // 2, H // 2), interpolation=cv2.INTER_NEAREST)
    filled = np.where(rob[..., None], cv2.resize(cv2.inpaint(small, rs_, 5, cv2.INPAINT_TELEA), (W, H)), img)
    def to_floor(uu, vv):
        uu = np.atleast_1d(np.asarray(uu, float)); vv = np.atleast_1d(np.asarray(vv, float))
        rays = np.stack([(uu - W / 2) / f, -(vv - H / 2) / f, -np.ones_like(uu)], 1) @ R.T
        return c + rays * ((z_plane - c[2]) / rays[:, 2])[:, None]
    aim = to_floor(W / 2, H / 2)[0, :2] if v.get("aim") is None else np.asarray(v["aim"])[:2]
    im = Image.from_numpy(np.ascontiguousarray(filled), format=ImageFormat.RGB)
    dets = [dd for dd in _OWL["d"].query_detections(im, queries, threshold=OWL_THR if thr is None else thr).detections
            if (dd.bbox[2] - dd.bbox[0]) * (dd.bbox[3] - dd.bbox[1]) < 0.6 * W * H]
    def floor_span(dd):                                                     # the box's size laid on the floor (m)
        x1, y1, x2, y2 = dd.bbox; P = to_floor([x1, x2, x1, x2], [y1, y1, y2, y2])[:, :2]
        return float(max(np.ptp(P[:, 0]), np.ptp(P[:, 1])))
    dets = [dd for dd in dets if floor_span(dd) <= MAX_ITEM_M]              # nothing bigger than any toy: a floor patch or
                                                                            # furniture called "a block" (29 Sep: a third of a
                                                                            # wide look, 27 cm off, drew every close look away)
    dist = {id(dd): float(np.linalg.norm(to_floor((dd.bbox[0] + dd.bbox[2]) / 2, (dd.bbox[1] + dd.bbox[3]) / 2)[0, :2] - aim)) for dd in dets}
    near = [dd for dd in dets if dist[id(dd)] <= max_dist]
    if not near: return None
    best = max(near, key=lambda dd: dd.confidence - 0.5 * dist[id(dd)])       # confident, and near where it was flagged
    if WHOLE_BOX:                                                            # the whole toy, not its part (look-closer [pYZ])
        best = whole_box(best, dets)
    seg = _SAM["s"].segment(ImageDetections2D(im, [best])).detections
    mk = np.asarray(seg[0].mask).astype(bool) if seg else None
    if mk is None or mk.shape != (H, W) or (mk & ~rob).sum() < 30:
        mk = np.zeros((H, W), bool); x1, y1, x2, y2 = map(int, best.bbox); mk[y1:y2, x1:x2] = True
    mk &= ~rob
    if v.get("ir_frames") is not None and len(v["ir_frames"]) == 5:          # the infrared trial, flashing: the item's outline from the
        import ir_camera                                                     # flash shadows, whole ([6hI]), when it closes
        whole = ir_camera.item_mask(v["ir_frames"], v, mk, rob)
        if whole is not None and whole.sum() >= 30: mk = whole
    grew = None                                                             # None: no pair fitted
    if v.get("stereo_h") is not None:                                       # the pair: the item whole where the detector boxed
        import wrist_stereo                                                 # a part of it (look-closer [GMs])
        mk, grew = wrist_stereo.whole(mk, wrist_stereo.standing(v["stereo_h"], rob))
    vv, uu = np.nonzero(mk); pts = to_floor(uu, vv); ctr = pts[:, :2].mean(0); P2 = pts[:, :2] - ctr
    opts = grip_options(P2)                                                   # width where the fingers close, per direction (it fits if any way does)
    width, jaw_axis = min(opts, key=lambda o: o[0])[:2]; along = np.array([-jaw_axis[1], jaw_axis[0]]); length = np.percentile(P2 @ along, 95) - np.percentile(P2 @ along, 5)
    breadth = np.percentile(P2 @ jaw_axis, 95) - np.percentile(P2 @ jaw_axis, 5)   # its whole extent across: what the bin must hold
    ray = np.array([(uu.mean() - W / 2) / f, -(vv.mean() - H / 2) / f, -1.0]) @ R.T
    return {"xy": ctr, "cam": c.copy(), "ray": ray / np.linalg.norm(ray), "long_axis": along, "short_axis": jaw_axis, "mask": mk,
            "label": best.name, "score": float(best.confidence), "grip_options": opts, "grip_width_cm": float(width * 100), "fits_jaws": bool(width < jw.max_m()),
            "length_cm": float(length * 100), "breadth_cm": float(breadth * 100), "width_cm": float(width * 100), "pixels": int(len(vv)),
            "edge_xy": edge_contact(v, mk, rob, z_plane), "whole_from_stereo": grew}


FIND_QUERIES = ["a toy", "a stuffed animal", "a sock", "a shoe", "a ball", "a toy car", "a toy animal", "a plastic toy", "a piece of cloth",
                "a small object", "a computer mouse", "a roll of tape", "a cup", "a mug", "a bowl", "a towel", "a glove",
                "scissors", "a box", "a game case", "crayons", "a pencil case"]   # as QUERIES: at 25 empty flags these added a
                                                 # detection near the flag at 1 (28 Sep), and found the mouse and the tape
def owl_boxes(img, queries=FIND_QUERIES, thr=0.15):
    """open-vocabulary boxes in an image (e.g. the head camera): [((x1, y1, x2, y2), score, label)]"""
    if "d" not in _OWL:
        sys.path.insert(0, _DIMOS); from dimos.perception.detection.detectors.owlv2 import Owlv2Detector
        _OWL["d"] = Owlv2Detector()
    from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
    dets = _OWL["d"].query_detections(Image.from_numpy(np.ascontiguousarray(img), format=ImageFormat.RGB), queries, threshold=thr).detections
    return [(tuple(float(x) for x in dd.bbox), float(dd.confidence), dd.name) for dd in dets]


ZOOM_THR = 0.10     # in the crop around a flag, where the sweep already found a change: the 4 cm wind-up toy scored 0.118
                    # there, the slate tile's own boxes 0.03-0.07 (27 Sep, night 70/1 round 3)


def owl_boxes_zoomed(img, centre, queries=FIND_QUERIES, thr=0.15, zooms=(3, 4), zoom_thr=ZOOM_THR):
    """open-vocabulary boxes in the whole image and in crops around `centre` (x, y px) enlarged `zooms` times, mapped
    back: small finds (a 4 cm wind-up toy, ~15 px at the standing look's 0.8 m) are too small for OWLv2 at the whole
    frame's scale and found at a crop's (sliced inference, as SAHI; 27 Sep: the toy sat at the flag, unseen). Two
    scales: 3x reaches a flag 13-30 cm off, 4x the smallest (28 Sep: the wind-up toy scored 0.09-0.12 at 3x, 0.21-0.23
    at 4x, 0.03-0.10 at 5x and 0.04-0.18 at 6x in two looks)"""
    import cv2
    H, W = img.shape[:2]; out = owl_boxes(img, queries, thr)
    for zoom in zooms:
        cw, ch = W // zoom, H // zoom
        x0 = int(np.clip(centre[0] - cw / 2, 0, W - cw)); y0 = int(np.clip(centre[1] - ch / 2, 0, H - ch))
        crop = cv2.resize(np.ascontiguousarray(img[y0:y0 + ch, x0:x0 + cw]), (cw * zoom, ch * zoom), interpolation=cv2.INTER_CUBIC)
        for (x1, y1, x2, y2), sc, lab in owl_boxes(crop, queries, zoom_thr):
            if x2 - x1 > 0.5 * crop.shape[1] or y2 - y1 > 0.5 * crop.shape[0]: continue   # most of the crop: floor, not a find
            out.append(((x0 + x1 / zoom, y0 + y1 / zoom, x0 + x2 / zoom, y0 + y2 / zoom), sc, lab))
    return out


EDGE_PX = 3                      # a mask this close to the frame's border runs off it
EDGE_MIN = 5                     # border pixels it takes: a stray pixel or two is the mask's noise


def edge_contact(cv, mask, robot=None, z=0.015):
    """where a look's item mask runs off the image (look-closer [TBu]): the floor point (x, y) of its pixels along the
    frame's border, or None if the whole item is inside the frame. The robot's own pixels don't count: an item passing
    under the jaws is not cut by the frame"""
    m = np.asarray(mask, bool) & (~np.asarray(robot, bool) if robot is not None else True)
    H, W = m.shape; b = np.zeros_like(m); b[:EDGE_PX] = b[-EDGE_PX:] = True; b[:, :EDGE_PX] = b[:, -EDGE_PX:] = True
    if (m & b).sum() < EDGE_MIN: return None
    return footprint_at(cv, m & b, z).mean(0)


def find_point(cv, box, mask=None, middle=True, z=0.015):
    """where on the floor a detection is (sim-slice [82p]). middle: the centroid of its mask laid on the floor, with
    its length and long axis (a toy lying pointing away is seen whole from standing; its middle is where to park and
    aim). Otherwise, or with no mask pixels, the old point: the box's foot, where an upright thing meets the floor, which
    for a lying toy is its end nearest the camera (the elephant gripped by its tail, 28 Sep, night 70/6)"""
    x1, y1, x2, y2 = box                                                    # the look's own size: its mask's, else its image's
    H, W = (mask.shape if mask is not None else cv["img"].shape[:2] if "img" in cv else (360, 640))[:2]   # ([7cU])
    if middle and mask is not None and np.asarray(mask).sum() >= 30:
        P = footprint_at(cv, np.asarray(mask, bool), z); ctr = P.mean(0); Q = P - ctr
        w, V = np.linalg.eigh(Q.T @ Q); ax = V[:, 1]; t = Q @ ax
        return {"xy": ctr, "length_cm": float((np.percentile(t, 97.5) - np.percentile(t, 2.5)) * 100), "long_axis": ax}
    u, vv = (x1 + x2) / 2, y2 - 0.1 * (y2 - y1)
    fpx = (H / 2) / np.tan(np.radians(cv["fovy"]) / 2)
    ray = np.asarray(cv["R"]) @ np.array([(u - W / 2) / fpx, -(vv - H / 2) / fpx, -1.0])
    if ray[2] >= -1e-3: return None
    c = np.asarray(cv["c"]); return {"xy": c[:2] + ray[:2] * (-c[2] / ray[2]), "length_cm": None, "long_axis": None}


def segment_box(img, box, robot=None):
    """EdgeTAM's cut-out of one detection box (as detect6), or the box itself if it gives none; the robot's pixels out"""
    sys.path.insert(0, _DIMOS); from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
    from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
    H, W = img.shape[:2]; x1, y1, x2, y2 = map(int, box); mk = None
    try:                                                                      # no segmenter (no GPU) or no cut-out: the box
        if "s" not in _SAM:
            from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter; _SAM["s"] = EdgeTAMImageSegmenter()
        im = Image.from_numpy(np.ascontiguousarray(img), format=ImageFormat.RGB)
        det = Detection2DBBox(bbox=(x1, y1, x2, y2), track_id=0, class_id=0, confidence=1.0, name="item", ts=0.0, image=im)
        seg = _SAM["s"].segment(ImageDetections2D(im, [det])).detections
        mk = np.asarray(seg[0].mask).astype(bool) if seg else None
    except Exception:
        mk = None
    if mk is None or mk.shape != (H, W) or mk.sum() < 30:
        mk = np.zeros((H, W), bool); mk[max(y1, 0):y2, max(x1, 0):x2] = True
    return mk & ~np.asarray(robot, bool) if robot is not None else mk


def footprint_at(cv, mask, z):
    """a look's mask pixels projected onto the horizontal plane at height z (m): (N, 2) floor x, y"""
    H, W = mask.shape; f = (H / 2) / np.tan(np.radians(cv["fovy"]) / 2); vv, uu = np.nonzero(mask)
    rays = np.stack([(uu - W / 2) / f, -(vv - H / 2) / f, -np.ones(len(uu))], 1) @ np.asarray(cv["R"]).T
    return (np.asarray(cv["c"]) + rays * ((z - cv["c"][2]) / rays[:, 2])[:, None])[:, :2]


def ref_from_mask(v, mask, z_plane=0.015):
    """a look-closer result from a known item mask (the privileged demonstrator's view: the sim's segmentation)"""
    img = v["img"]; H, W = img.shape[:2]; c = v["c"]; R = v["R"]; f = (H / 2) / np.tan(np.radians(v["fovy"]) / 2)
    vv, uu = np.nonzero(mask & ~v["robot"])
    if len(vv) < 20: return None
    rays = np.stack([(uu - W / 2) / f, -(vv - H / 2) / f, -np.ones_like(uu, float)], 1) @ R.T
    pts = c + rays * ((z_plane - c[2]) / rays[:, 2])[:, None]; ctr = pts[:, :2].mean(0); P2 = pts[:, :2] - ctr
    opts = grip_options(P2)                                  # the detector's own grip options (width, jaw, offset), so the
    w0, j0 = min(opts, key=lambda o: o[0])[:2]               # teacher's demonstrations grip the way the robot's pick does
    return {"xy": ctr, "short_axis": j0, "grip_options": opts, "grip_width_cm": w0 * 100, "fits_jaws": bool(w0 < jw.max_m())}
