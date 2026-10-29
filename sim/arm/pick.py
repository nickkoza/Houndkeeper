"""One pick routine for every sim: look closer, grip, lift, carry into the bin, stow. It needs only what the real robot
has: arm motions, the wrist camera (with its pose from the arm's joint angles) and the gripper's own position. It never
reads the simulator's truth, so the same code runs in the fast standalone sim and on DimOS.

The robot object S provides: look_down(xy, out, h_cam), ik(pos, jaw_dir), arm_to(q, secs), arm_to_safe(q, secs),
grip_to(g, secs), step(n), state(name), capture() -> wrist view dict, gripper() -> jaw joint position, and
base_xy() -> robot position, plus a `waypoints` list of known-safe poses."""
import os, time
from pathlib import Path
import numpy as np
import jaws as jw
import wrist_refine as wr

LOOK_H = 0.18            # look from 18 cm above the grasp point (camera ~32 cm up): fingertips clear of any item we pick
IN_VIEW = 0.08           # camera this far short of the item, toward the robot, so the item sits in the upper image
STOCK_FOVY_DEG = 65.0    # the stock wrist camera's vertical view, which LOOK_H and IN_VIEW were set for
FRAME_H_MAX = 0.45       # no close look higher than the wide one (the arm reaches it)


def framing(S):
    """how much further back and higher a close look stands so the wrist camera fitted frames the floor as the stock
    one did (the stereo pair sees 50 deg up and down, not 65: toys fell off the top of the frame, 29 Sep, look-closer [GMs])"""
    import mujoco
    m = getattr(S, "m", None)
    if m is None or mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "arm_wrist") < 0: return 1.0
    fovy = float(m.cam_fovy[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "arm_wrist")])
    return max(1.0, np.tan(np.radians(STOCK_FOVY_DEG / 2)) / np.tan(np.radians(fovy / 2)))
FLAG_TOL = 0.15          # a flag is at most 10 cm off (look-closer [hwo]); a find further than this is something else
WIDE_H, WIDE_TOL = 0.45, 0.30   # the wide first look: camera higher, and a flag may be further off (head-camera flags)
HIGH_CARRY = os.environ.get("HK_HIGH_CARRY", "0") == "1"   # sim-grasp [2ho]: carry for the longest hang, as high as reachable
BIN_EXIT = os.environ.get("HK_BIN_EXIT", "0") == "1"   # sim-grasp [2ho]: let go, leave the bin the way it came down
HANG_MAX = 0.30          # m: the longest an item the jaws fit may hang below them (a 30 cm toy gripped by one end)
WHOLE_ITEM = os.environ.get("HK_WHOLE_ITEM", "0") == "1"   # look-closer [TBu]: looks cut off by the frame don't decide
OPEN, CLOSED = 0.05, 0.0 # jaw joint targets
ARM_KG = 1.5              # the arm's rated payload (kg): anything heavier is put back ([XCl])
HELD_MIN = 0.0005        # jaw joint (half the gap) after closing: >= 1 mm between the jaws means something is between them
                         # (empty jaws read 0.0; a 7 mm pencil held at the tapered tips reads ~2.8 mm in the sim; calibrate on the real arm)
JAW_MAX = jw.max_m()  # widest item the open jaws take (m): less any pads
MIN_GRIP_M = 0.005    # narrower is a sliver of the mask, not a part (sim-grasp [Q8C]; the thinnest item gripped: a pencil, 7 mm)


BIN_TOL_M = 0.015     # the measure's error allowed before an item is called too big for the bin: 1.5 cm refused no more
                      # grippable toys than 3 cm on 103 saved looks, and one more too-big one of 83 (an offline replay, 29 Sep)


def fits_bin(ref) -> bool:
    """would the item, as the looks measured it, lie in the bin (look-closer [pYZ], sim-slice [QyZ]: the
    35 x 28 cm cart turtle is too big, a shoe fits)? Unmeasured: no refusal"""
    import bin_geom
    L, B = ref.get("length_cm"), ref.get("breadth_cm")
    if L is None or B is None: return True
    return bin_geom.footprint_fits(L / 100, B / 100, BIN_TOL_M)


def real_grips(opts):
    """grip options the jaws can really close on: wider than a sliver, narrower than the open jaws (less 1 cm)"""
    return [o_ for o_ in opts if MIN_GRIP_M <= o_[0] <= JAW_MAX - 0.01]


def wrist_light(S, on):
    """the wrist's white light ([J7G]) on or off: steady through a look, never pulsed ([0Py], safe for
    photosensitive viewers; the real one fades over ~0.5 s). No-op without one"""
    import mujoco
    m = getattr(S, "m", None)                                               # every LED in the holder, together (none on a
    if m is None: return                                                    # robot with no model: the real one, or a test's)
    for i in range(m.nlight):
        if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_LIGHT, i) or "").startswith("arm_wrist_light"): m.light_active[i] = int(on)


def look_closer(S, guess, detector=None, wide_tol=WIDE_TOL, _again=False):
    """the close look (below), the wrist's light on for it and off after however it ends ([J7G])"""
    if _again: return _look_closer(S, guess, detector, wide_tol, _again)
    wrist_light(S, True)
    try: return _look_closer(S, guess, detector, wide_tol, _again)
    finally: wrist_light(S, False)


def _look_closer(S, guess, detector=None, wide_tol=WIDE_TOL, _again=False):
    """Three looks (over the flag, and 6 cm to each side), grounded detection in each, fused by triangulation.
    Returns (ref, notes): ref has xy, short_axis (jaw closing direction), grip_width_cm, fits_jaws, label; or None.
    wide_tol: how far the wide first look may re-acquire the find from `guess` (WIDE_TOL for a head-camera flag; less
    once the standing look has placed it: the wide look took a slate tile, "a piece of cloth" 21 cm off, over a 4 cm
    toy the standing look had within 4 cm, 28 Sep)
    WHOLE_ITEM ([TBu]): a look whose item runs off the frame saw part of it; if every look did, the looks are made once
    more aimed at where it was cut (_again), and an item still not seen whole goes to review"""
    detector = detector or wr.detect6
    if not _again: S.cut_views = []
    out = np.asarray(guess) - S.base_xy(); out = out / (np.linalg.norm(out) + 1e-9); side = np.array([-out[1], out[0]])
    notes = []; caps = []; S.last_looks = []                                  # (capture, detection) per look, for offline benches
    S.state("LOOK CLOSER: find and name")                                    # one detector call locates and names ([JKS])
    import ir_camera
    if ir_camera.ON and not _again: S.grip_to(OPEN, 0.4)                    # the infrared trial: jaws open so the LEDs by them
                                                                             # light the floor ahead ([GWg])
    q = None                                                                  # one wide look first: re-acquire a flag that is off
    for back, h in (() if _again else ((IN_VIEW, WIDE_H), (IN_VIEW, WIDE_H - 0.06), (0.0, WIDE_H), (IN_VIEW * 2, WIDE_H - 0.06))):
        q = S.look_down(np.asarray(guess) - out * back, out, h_cam=h)       # (not again: the guess is where it was cut)
        if q is not None: break
    if q is None and not _again: notes.append("no wide look pose")
    if q is not None:
        S.arm_to(q, 1.5); S.step(40); cv = S.capture(); cv["aim"] = np.asarray(guess)
        try: rf = detector(cv, max_dist=wide_tol)                                # the wide look searches as far as a flag can be off
        except TypeError: rf = detector(cv)
        S.last_looks.append((dict(cv, wide=True), rf))
        if rf is not None and np.linalg.norm(rf["xy"] - guess) <= wide_tol:
            notes.append(f"re-acquired {np.linalg.norm(rf['xy'] - guess) * 100:.0f} cm from the flag"); guess = rf["xy"]
        else: notes.append("wide look: nothing" if rf is None else f"wide look: something {np.linalg.norm(rf['xy'] - guess) * 100:.0f} cm off")
    for k, dx in enumerate((0.0, 0.06, -0.06)):
        q = None
        k = framing(S)
        for back, h in ((IN_VIEW, LOOK_H + 0.14), (IN_VIEW / 2, LOOK_H + 0.14), (IN_VIEW, LOOK_H + 0.19), (0.0, LOOK_H + 0.09)):
            back, h = back * k, min(h * k, FRAME_H_MAX)
            q = S.look_down(np.asarray(guess) + side * dx - out * back, out, h_cam=h)       # fall back to other heights and offsets
            if q is not None: break
        if q is None:
            notes.append("no look pose"); continue
        S.arm_to(q, 1.5 if k == 0 else 0.8); S.step(40); cv = S.capture(); cv["aim"] = np.asarray(guess); caps.append(cv)
    views = []
    for cv in caps:
        rf = detector(cv); S.last_looks.append((cv, rf))
        if rf is not None: rf["_cv"] = cv                                    # the look it came from (width at the item's height)
        if rf is None:
            notes.append("nothing"); continue
        if np.linalg.norm(rf["xy"] - guess) > FLAG_TOL:
            notes.append("too far from the flag"); continue
        views.append(rf); notes.append(rf.get("label", "found"))
    if WHOLE_ITEM and views:                                                 # [TBu]: fragments don't decide width, grip or name
        cut = [v for v in views if v.get("edge_xy") is not None]
        S.cut_views = cut                                                # still measure the bin by them: a cut look under-reads ([GMs])
        if cut:
            notes.append(f"{len(cut)} of {len(views)} cut off by the frame"); views = [v for v in views if v.get("edge_xy") is None]
            if not views:
                if _again:                                                 # never seen whole: with the pair, what was seen is a
                    big = seen_part_too_big(cut)                           # lower bound; if even that won't fit the bin, it's
                    if big is not None: return big, notes + ["never seen whole; too big for the bin by the part seen"]   # too big ([GMs])
                    return None, notes + ["never seen whole"]
                e = np.mean([v["edge_xy"] for v in cut], 0)
                notes.append(f"re-aimed {np.linalg.norm(e - guess) * 100:.0f} cm toward the cut")
                ref, more = _look_closer(S, e, detector, wide_tol, _again=True)
                return ref, notes + more
    if not views: return None, notes
    if len(views) == 1:                                                      # one look of three saw it (a shadow's edge did, 25 Sep):
        k = framing(S)
        q = S.look_down(np.asarray(views[0]["xy"]) - out * IN_VIEW * k, out, h_cam=min((LOOK_H + 0.14) * k, FRAME_H_MAX))   # look again, aimed at it, and
        if q is not None:                                                    # pick only if that look sees it too
            S.arm_to(q, 0.8); S.step(40); cv = S.capture(); cv["aim"] = np.asarray(views[0]["xy"]); rf = detector(cv)
            S.last_looks.append((cv, rf))
            if rf is not None and np.linalg.norm(rf["xy"] - views[0]["xy"]) <= FLAG_TOL and not (WHOLE_ITEM and rf.get("edge_xy") is not None):
                views.append(rf); notes.append(f"again: {rf.get('label', 'found')}")
            else: notes.append("again: nothing")
        if len(views) == 1: return None, notes + ["seen by one look only"]
    xyz, kept = wr.fuse(views)
    if xyz is None:                                                          # the looks don't agree on a point: the first, but
        ref = dict(views[0])                                                  # still with every width opinion (28 Sep: a figure
        if not ref.get("fits_jaws", True):                                    # refused unfused, fitting by one leg at its next stops)
            ref, note = _width_at_height(ref, views)
            if note: notes = notes + [note]
        return ref, notes + ["unfused"]
    best = min(kept, key=lambda v: np.linalg.norm(v["xy"] - xyz[:2])); ref = dict(best); ref["xy"] = xyz[:2]; ref["z_est"] = float(xyz[2])
    sized = kept + [v for v in getattr(S, "cut_views", []) if v.get("whole_from_stereo") is not None]
    if any(v.get("whole_from_stereo") is not None for v in kept):         # the pair fitted: the bin is judged by the largest
        ref["length_cm"] = max(v["length_cm"] for v in sized); ref["breadth_cm"] = max(v["breadth_cm"] for v in sized)   # look,
                                                                          # cut ones too: they only under-read ([GMs])
    if not ref.get("fits_jaws", True):                                       # too wide, measured flat: measure again at its height
        ref, note = _width_at_height(ref, kept)
        if note: notes = notes + [note]
    return ref, notes + [f"fused {len(kept)}/{len(views)}"]


def seen_part_too_big(cut):
    """an item no look saw whole, measured by the parts the looks did see (the stereo pair's outlines; a look cut off by
    the frame only under-reads): the largest as its ref if even that won't fit the bin, else None (look-closer [GMs])"""
    cut = [v for v in cut if v.get("whole_from_stereo") is not None and v.get("length_cm") is not None]
    if not cut: return None
    ref = dict(max(cut, key=lambda v: v["length_cm"]))
    ref["length_cm"] = max(v["length_cm"] for v in cut); ref["breadth_cm"] = max(v["breadth_cm"] for v in cut)
    ref["seen_whole"] = False
    return ref if not fits_bin(ref) else None


def _width_at_height(ref, views):
    """a second opinion before calling a find too wide for the jaws: the width measured on each look's mask laid on
    the floor makes a standing toy's upper parts land further along the line of sight, and nearer the camera they look
    bigger, so it reads wide (28 Sep: a pony and a Transformers figure, grippable by their tiers, called too wide in the
    live nights). Laid instead at the height the looks triangulated it at (z_est, the median over the looks), the
    width is under-read but never calls a grippable item too wide (47 saved looks, replayed offline: 0 against the flat
    measure's 3; attempted wrongly 7 against 6). It is too wide only if both measures say so: a wrong attempt costs a
    failed grasp, a wrong refusal leaves the item on the floor."""
    z = max(float(ref.get("z_est") or 0.015), 0.015); flat = float(ref.get("grip_width_cm", 0.0)); per = []
    for v in views:
        cv = v.get("_cv")
        if cv is None or v.get("mask") is None: continue
        P = wr.footprint_at(cv, v["mask"], z); per.append(wr.grip_options(P - P.mean(0)))
    if not per: return ref, None
    w_med = float(np.median([o[0][0] for o in per]))
    rule = "at its height"
    if w_med >= JAW_MAX:                                                  # a third opinion: gripped by one part (a toy lying with
        part = []                                                         # its legs apart: every strip through its middle spans
        for v in views:                                                   # both legs, one leg is 5 cm, 28 Sep; part_grip_options)
            if v.get("_cv") is None or v.get("mask") is None: continue
            P = wr.footprint_at(v["_cv"], v["mask"], z); o_ = wr.part_grip_options(P - P.mean(0))
            if v["_cv"].get("stereo_h") is not None:                      # the pair: a part only where both fingers can land
                import wrist_stereo                                        # on floor beside it ([GMs])
                o_ = [o for o in o_ if wrist_stereo.fingers_land(P.mean(0) + o[2], o[1], o[0], v["_cv"]["stereo_P"], v["_cv"]["stereo_h"])]
            if o_: part.append(o_)
        if not part or float(np.median([o[0][0] for o in part])) >= JAW_MAX:
            return ref, f"too wide at its height too ({w_med * 100:.1f} cm), and by any one part"
        per, w_med, rule = part, float(np.median([o[0][0] for o in part])), "by one part"
    opts = min(per, key=lambda o: o[0][0])
    ref = dict(ref); ref["fits_jaws"] = True; ref["grip_options"] = opts; ref["grip_width_cm"] = float(opts[0][0] * 100)
    ref["width_rule"] = rule
    return ref, f"fits {rule} ({w_med * 100:.1f} cm; laid flat {flat:.1f} cm, too wide)"


def save_looks(S, path, notes=()):
    """every look of the last look_closer, as it was taken: image, camera pose and field of view, the robot's own pixels,
    where it aimed and what the detector made of it, so a miss can be replayed offline ([JKS])"""
    out = {"notes": np.array([str(n) for n in notes])}
    for k, (cv, rf) in enumerate(getattr(S, "last_looks", [])):
        for key, val in cv.items():
            if isinstance(val, (np.ndarray, float, int, bool)) or np.isscalar(val): out[f"{k}_{key}"] = np.asarray(val)
        if rf is not None:
            out[f"{k}_found_xy"] = np.asarray(rf["xy"]); out[f"{k}_label"] = np.asarray(str(rf.get("label", "")))
    Path(path).parent.mkdir(parents=True, exist_ok=True); np.savez_compressed(path, **out)


AUTO_DIR = Path(os.environ.get("HK_PICK_RECORD_DIR", Path(__file__).resolve().parents[2] / "data" / "demos_auto"))


class PickRecorder:
    """Records one pick attempt in the demonstration format ([95i]): wrist image (the policy camera), joint state and
    commanded joint targets at 10 Hz, the frame where the lift ended, and the outcome. The robot calls S.rec(S) at
    10 Hz while it moves. On by default; HK_PICK_RECORD=0 turns it off."""
    def __init__(self, S, source):
        self.S, self.source = S, source; self.f = {"img": [], "state": [], "action": []}; self.lift_end = None; S.rec = self.frame
        S.last_recording = None                                    # never annotate an earlier attempt's file
    def frame(self, S):
        if self.lift_end is not None and len(self.f["img"]) > self.lift_end + 40: return    # 4 s after the lift is plenty
        self.f["img"].append(S.small_image()); self.f["state"].append(np.asarray(S.joint_state(), np.float32))
        self.f["action"].append(np.asarray(S.target(), np.float32))
    def mark_lift_end(self): self.lift_end = len(self.f["img"])
    def save(self, fail, **extra):
        self.S.rec = None
        if len(self.f["img"]) < 5: return None
        d = AUTO_DIR / self.source; d.mkdir(parents=True, exist_ok=True); path = d / f"ep_{time.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}.npz"
        np.savez_compressed(path, img=np.array(self.f["img"]), state=np.array(self.f["state"]), action=np.array(self.f["action"]),
                            lift_end=-1 if self.lift_end is None else self.lift_end, fail=str(fail),
                            success=bool(fail is None), source=self.source, **extra)
        self.S.last_recording = path; return path


def annotate_last(S, **truth):
    """add the simulator's truth (e.g. truth_lifted, truth_in_bin, item) to the last recorded attempt"""
    p = getattr(S, "last_recording", None)
    if p is None: return
    z = dict(np.load(p, allow_pickle=True)); z.update(truth); np.savez_compressed(p, **z)


def grasp_to_bin(S, ref, q_ready, q_bin):
    """Grip across the narrowest side, lift, carry into the bin. Returns (fail reason or None, gripper width held).
    Every attempt is recorded for training ([95i])."""
    rec = PickRecorder(S, getattr(S, "source", "sim")) if os.environ.get("HK_PICK_RECORD", "1") != "0" else None
    try:
        fail, held = _grasp_to_bin(S, ref, q_ready, q_bin, rec)
    except StopIteration:                                      # a caller that only wanted the lift stopped it there
        if rec: rec.save("stopped by the caller after the lift", label=str(ref.get("label")))
        raise
    except Exception as e:
        if rec: rec.save(f"error: {e!r}", label=str(ref.get("label")))
        raise
    finally:
        set_payload(S, 0.0)                                    # released, put back or failed: nothing held ([qaS])
    if rec: rec.save(fail, label=str(ref.get("label")), grip_width_cm=float(ref.get("grip_width_cm", 0)))
    return fail, held


def payload_kg(weighed_kg):
    """what the arm's gravity feed-forward is told it holds: the weighed mass x (1 + maker_arm.payload_estimate_error),
    never below zero ([qaS])"""
    from houndkeeper import params as P
    return max(0.0, float(weighed_kg) * (1.0 + float(P.get("maker_arm.payload_estimate_error"))))


def set_payload(S, kg):
    """the held mass on the robot's gravity feed-forward ([cqG], [qaS]), where the robot has one"""
    if hasattr(S, "set_payload"): S.set_payload(kg)


MAX_LEAN = float(os.environ.get("HK_MAX_LEAN", "30"))   # deg the gripper may lean off vertical ([Hwc]); 0 = straight down
APPROACH = (0.12, 0.07, 0.03)                            # pre-grasp standoffs (m): back along the gripper's axis, or straight up
TIP_CLEAR = 0.003                                        # fingertips this far above the floor at the grasp: low as it goes, no dragging
THIN = 0.015                                             # items narrower than this (pencil, spoon) are gripped with the tips almost on
THIN_TIP, THIN_LEAN = 0.002, 20.0                        # 2 mm up (below the middle of a 7 mm pencil; never at the floor: that
                                                         # pressed rope into it at full torque and tipped the robot, [7Xd]); jaws nearer vertical
CONTACT_NM = 3.0                                         # a joint torque this far above the approach's own means contact: stop ([7Xd])


def _approach_poses(S, p, a, Rg, qg):
    """the pre-grasp poses at the APPROACH standoffs, far to near, each backed off along the gripper's axis or else
    straight up, clear of the robot and the floor; None if one can't be reached"""
    pre = []; qprev = qg
    for dd in APPROACH[::-1]:                                            # solved outward from the grasp, for continuity
        ok = False
        for off in (-a * dd, np.array([0, 0, dd])):                     # along the axis; else straight down onto it (a leaning
            qp, ep = S.ik_pose(p + off, Rg, qprev)                       # gripper backing off along its axis can meet the robot's head)
            if ep <= 0.01 and not S._hits(qp) and not S.floor_hit(qp): ok = True; break
        if not ok:
            if os.environ.get("HK_PICK_DEBUG") == "1": print(f"   approach {dd} fails (ik {ep * 100:.1f} cm, hits {S._hits(qp)}, floor {S.floor_hit(qp)})", flush=True)
            return None
        pre.append(qp); qprev = qp
    return pre[::-1]


def _toward_upright(R, deg):
    """link-7 orientation R turned deg toward the gripper pointing straight down, about the horizontal axis that does
    it (the jaw's closing direction moves least); R itself if already upright"""
    a = -R[:, 2]; ax = np.cross(a, [0, 0, -1.0]); n = np.linalg.norm(ax)
    if n < 1e-6: return R
    ax /= n; t = np.radians(min(deg, np.degrees(np.arccos(np.clip(a @ [0, 0, -1.0], -1, 1)))))
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return (np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K) @ R


def _dbg(S, tag):
    """HK_PICK_DEBUG=1: the robot's tilt and the arm's largest joint torque at a stage of the pick"""
    if os.environ.get("HK_PICK_DEBUG") != "1" or not hasattr(S, "d"): return
    R_ = S.d.xmat[S.tr].reshape(3, 3); tilt = np.degrees(np.arccos(np.clip(R_[2, 2], -1, 1)))
    extra = ""
    if getattr(S, "ob", None) is not None:              # sim truth: the item's centre of mass from the grip, and its turn since closing
        com = S.d.xipos[S.ob]; off = com - S.grasp_point()
        Rj = S.d.xmat[S.m.body("arm_gripper_left").id].reshape(3, 3); Ro = Rj.T @ S.d.xmat[S.ob].reshape(3, 3)   # in the jaws' frame:
        if tag == "closed": S._R_closed = Ro                                              # the arm's own turning doesn't count
        if tag in ("closed", "lifted"):                  # the jaw contacts on the item: how many, how hard, what friction
            import mujoco
            jaw_b = {S.m.body(n).id for n in ("arm_gripper_left", "arm_gripper_right")}; f6 = np.zeros(6); rows = []
            for i in range(S.d.ncon):
                c = S.d.contact[i]; bb = {S.m.geom_bodyid[c.geom1], S.m.geom_bodyid[c.geom2]}
                if S.ob in bb and bb & jaw_b:
                    mujoco.mj_contactForce(S.m, S.d, i, f6)
                    jb = next(iter(bb & jaw_b)); jg = c.geom1 if S.m.geom_bodyid[c.geom1] == jb else c.geom2
                    loc = (c.pos - S.d.xpos[jb]) @ S.d.xmat[jb].reshape(3, 3)          # in the jaw's frame: z down the finger
                    part = "pad" if "pad_" in (S.m.geom(jg).name or "") else "finger"
                    rows.append(f"{'L' if S.m.body('arm_gripper_left').id == jb else 'R'} {part} n={f6[0]:.1f}N mu={np.round(c.friction[:3], 3).tolist()} "
                                f"across {loc[0] * 1000:+.0f} down {-loc[2] * 1000:.0f}mm")
            print(f"   jaw contacts ({len(rows)}): " + "; ".join(rows), flush=True)
        turn = np.degrees(np.arccos(np.clip((np.trace(S._R_closed.T @ Ro) - 1) / 2, -1, 1))) if getattr(S, "_R_closed", None) is not None else 0.0
        extra = f", item CoM {np.linalg.norm(off[:2]) * 100:4.1f} cm off the grip sideways, {off[2] * 100:+5.1f} cm up; turned {turn:4.1f} deg in the jaws since closing"
    print(f"   {tag:14s} tilt {tilt:4.1f} deg, max joint torque {np.abs(S.joint_efforts()).max():5.1f} Nm, tips {S.finger_low(S.d.qpos[S.aq]) * 1000:5.1f} mm, jaws {S.gripper() * 2000:5.1f} mm{extra}", flush=True)


def carry_plan(S, ref, o, jaw, qu):
    """the carry pose over the item and the hang it allows for. An item can turn in the jaws and hang by its longest
    side (a shoe, 25 Sep); with HIGH_CARRY ([2ho]) by as much as any item could (HANG_MAX: the look may have seen a
    part, and one gripped by a leg hangs by nearly its length), as high as the arm reaches short of that, never lower
    than 0.35 m; else by its measured length. Falls back to the lift pose qu only if no carry height is reachable"""
    hang = HANG_MAX if HIGH_CARRY else float(np.clip(max(ref.get("length_cm", 15.0), ref.get("grip_width_cm", 0.0)) / 100, 0.05, HANG_MAX))
    top = max(0.35, S.bin_clear_z(hang) - o[2])
    for lift in (np.arange(top, 0.35 - 1e-9, -0.03) if HIGH_CARRY else [top]):
        for lim in (5.0, MAX_LEAN):                                          # carry with the gripper upright: a leaning grip holds
            q_carry, e_c, _ = S.ik_lean(o + [0, 0, lift], jaw, lim)          # the item on a slant, and 21% of 25-30 deg grips lost
            if q_carry is not None and e_c < 0.01: return q_carry, hang      # it on the way, against 1 in 17 below 25 (25 Sep)
    return qu, hang


def _grasp_to_bin(S, ref, q_ready, q_bin, rec=None):
    if not ref["fits_jaws"]: return "too wide to grip", None
    if not fits_bin(ref): return "too big for the bin", None                  # reported, not tried ([pYZ], 29 Sep)
    o = np.array([ref["xy"][0], ref["xy"][1], 0.0])
    import grasp_proposers as gp                                             # the grasp comes from the configured proposer
    prop = gp.proposer().propose(ref)                                        # (HK_GRASP; ours by default), best first ([WuR])
    if isinstance(prop, str): return prop, None
    plan = None; dbg = os.environ.get("HK_PICK_DEBUG") == "1"
    for cand, w_, al in zip(prop.grasps.candidates, prop.widths_m, prop.allowances):
        p, Rt = gp.pose_arrays(cand.pose); jaw = Rt[:2, 1] / (np.linalg.norm(Rt[:2, 1]) + 1e-9)
        if al.around == "vertical":                                          # jaw direction kept, lean free up to the allowance:
            for _ in range(6):                                               # the grasp point where the fingertips just clear the
                qg, e, lean = S.ik_lean(p, jaw, al.lean_deg)                 # floor (the tips reach ~3.4 cm past it; how far down
                if qg is None or e >= 0.01: break                            # depends on the lean)
                dz = al.tip_m - S.finger_low(qg)
                if al.height == "raise": dz = max(dz, 0.0)                    # the proposer's height, unless the tips would meet the floor
                if abs(dz) < 0.0005: break
                p = p + [0, 0, dz]
            if qg is None or e >= 0.01:
                if dbg: print(f"   jaw {np.round(jaw, 2)} w {w_ * 100:.1f} cm: no grasp pose (err {e * 100:.1f} cm, lean {lean:.0f})", flush=True)
                continue
        else:                                                                # the pose as given, turned toward upright if need be
            R0 = gp.link7_from_tcp(Rt); seed, _, _ = S.ik_lean(p, jaw, 60.0)
            seed = S.d.ctrl[12:18].copy() if seed is None else seed
            for tilt in np.arange(0.0, al.lean_deg + 1e-9, 5.0):
                qg, e = S.ik_pose(p, _toward_upright(R0, tilt), seed)
                if e < 0.01: break
            if e >= 0.01: continue
            fl = S.finger_low(qg)
            if fl < al.tip_m:                                                # the tips below the floor
                if al.tip_m - fl > al.raise_max_m: continue
                p = p + [0, 0, al.tip_m - fl]; qg, e = S.ik_pose(p, _toward_upright(R0, tilt), qg)
                if e >= 0.01: continue
            if S._hits(qg) or S.floor_hit(qg): continue
        a, Rg = S.gripper_frame(qg); pre = _approach_poses(S, p, a, Rg, qg)
        if pre is None: continue
        lean = float(np.degrees(np.arccos(np.clip(a @ [0, 0, -1.0], -1, 1)))); S.grasp_score = round(cand.score, 3)
        plan = (qg, pre, a, Rg, w_, lean, jaw, p); break
    if plan is None: return "no reachable grip direction", None
    qg, pre, a, Rg, w_, lean, jaw, p = plan; S.grip_width_used_cm = round(w_ * 100, 1); S.lean_used_deg = round(lean, 1); S.jaw_used = np.asarray(jaw)
    if os.environ.get("HK_PICK_DEBUG") == "1": print(f"   grasp: width {w_ * 100:.1f} cm, target z {p[2] * 1000:.0f} mm, tips at {S.finger_low(qg) * 1000:.1f} mm, lean {lean:.0f}", flush=True)
    S.state("GRASP"); S.grip_to(OPEN, 0.5)
    qu, _ = S.ik_pose(p + np.array([0, 0, 0.18]), Rg, pre[0])                  # the lift pose: weighed here empty, and again
    S.arm_to(qu, 1.0); S.step(60); tau0 = S.settled_efforts()                 # loaded, at the same pose ([XCl]), both once still
    for qp in pre: S.arm_to(qp, 0.6)                                         # in along the gripper's axis: no sweeping across the item
    S.step(20); tau_ref = S.joint_efforts(); S.contact_stop = None              # the last stretch in small steps, watching the torques:
    for k in range(1, 9):                                                    # meeting the item or the floor early stops the descent
        S.arm_to(pre[-1] + (qg - pre[-1]) * k / 8, 0.1); S.step(4)             # there instead of pressing on ([7Xd])
        rise = float(np.max(np.abs(S.joint_efforts() - tau_ref)))
        if rise > CONTACT_NM:
            S.contact_stop = round(k / 8, 2); S.arm_to(pre[-1] + (qg - pre[-1]) * (k - 1) / 8, 0.15); break
    _dbg(S, "at the item"); S.step(40); S.track_err_cm = round(float(np.linalg.norm(S.grasp_point() - p)) * 100, 1)
    S.grip_to(CLOSED, 0.8); S.step(60); _dbg(S, "closed")
    qb = pre[-1]                                                              # back out the way it came in, then straight up
    S.arm_to(qb, 0.8); _dbg(S, "backed out"); S.arm_to(qu, 1.2); S.step(60); _dbg(S, "lifted")
    held = S.gripper()
    if held < HELD_MIN or held > OPEN - 0.005: return "grasp (not lifted)", held      # closed on nothing, or never closed
    S.weighed_kg = round(S.held_mass(S.settled_efforts() - tau0, qu), 3)
    set_payload(S, payload_kg(S.weighed_kg))                                  # held from here; cleared by grasp_to_bin ([qaS])
    if S.weighed_kg > ARM_KG:                                                # too heavy to carry: put it back where it was
        S.arm_to(qb, 1.2); S.arm_to(qg, 1.0); S.grip_to(OPEN, 0.6); S.arm_to(qb, 0.8); S.arm_to(qu, 1.0)
        return f"too heavy (weighed {S.weighed_kg:.1f} kg)", held
    if rec: rec.mark_lift_end()
    S.state("INTO BIN"); q_carry = None
    q_carry, hang = carry_plan(S, ref, o, jaw, qu)
    S.arm_to(q_carry, 2.0); _dbg(S, "carry")
    if S.gripper() < HELD_MIN: return "dropped lifting", held
    q_over = S.over_bin(hang)                                                # cross the rim high enough for it, then lower into the
    S.waypoints = [q_ready, q_carry]; S.arm_to_safe(q_bin if q_over is None else q_over, 3.0)   # bin until it rests there
    _dbg(S, "over the bin")
    if S.gripper() < HELD_MIN: return "lost on the way to the bin (may be in it)", held   # the jaws can't tell where it fell
    release_in_bin(S, max_m=0.12 if q_over is None else 0.12 + hang)
    return None, held


def _lower_into_bin(S, max_m=0.12):
    """place, don't drop: from the release pose, lower the grasp point in 1 cm steps, gripper angle unchanged, until the
    joint torques show the item touching the tray or the arm can go no lower (a ball dropped from the 18 cm release pose
    bounced out, 25 Sep)"""
    q = S.target()[:6] if hasattr(S, "target") else None
    if q is None: return []
    a, R = S.gripper_frame(q); g0 = S.grasp_point(); tau_ref = S.joint_efforts(); qk = q; path = [q]
    for k in range(1, int(max_m * 100) + 1):
        qn, e = S.ik_pose(g0 - np.array([0, 0, 0.01 * k]), R, qk)
        if e > 0.01 or S._hits(qn): break
        S.arm_to(qn, 0.15); S.step(4); qk = qn; path.append(qn)
        if float(np.max(np.abs(S.joint_efforts() - tau_ref))) > CONTACT_NM / 2: break   # resting on the tray (or what's in it)
    return path


def release_in_bin(S, max_m=0.12):
    """lower it until it rests in the bin, let go, and with BIN_EXIT leave the way the arm came down, straight up, before
    any other move ([2ho]): the stow's planned path swept through the bin and knocked out a train set down against the
    rim (28 Sep). The poses back up are the ones the lowering passed through, so they clear the bin, and with the jaws
    open they rise off the item"""
    path = _lower_into_bin(S, max_m=max_m); S.grip_to(OPEN, 0.6); S.step(120)
    if BIN_EXIT:
        for q in reversed(path[:-1]): S.arm_to(q, 0.15)


RETRYABLE = ("grasp (not lifted)", "dropped lifting")         # failures another try could change ([m62])


def pick_with_tries(S, ref, q_ready, q_bin, relook=None, tries=3):
    """Up to `tries` grasps in one stop ([m62]): before each retry, look again (the item may have moved when the jaws
    brushed it) and grip a different way (jaw directions within 20 deg of one already tried go last). Retries only
    what another try could change; never "too wide" or "too heavy". relook(xy) -> (ref, notes). Returns (fail, held);
    S.tries says how many grasps were made."""
    used, fail, held = [], None, None; S.relook_notes = []; S.try_log = []   # each try's own outcome, for the report ([bOu])
    for k in range(tries):
        if k:
            if relook is None: break
            prev = ref; ref, notes = relook(np.asarray(ref["xy"])); S.relook_notes.append(notes)   # kept: a refusal is traced
            if ref is None: fail = "not seen again after a failed try"; break
            if not ref.get("fits_jaws", True) and prev.get("fits_jaws", True):   # an item doesn't change size between tries:
                ref = dict(ref, fits_jaws=True, grip_options=prev.get("grip_options"), grip_width_cm=prev.get("grip_width_cm"))
                S.relook_notes[-1] = list(notes) + ["kept the earlier look's fit"]   # a re-look calling it too wide after one
                                                                                  # found it fits is the measure's noise (28 Sep)
            if ref.get("grip_options"):
                near = lambda o: any(abs(float(np.dot(o[1], u))) > np.cos(np.radians(20)) for u in used)
                ref = dict(ref); ref["grip_options"] = sorted(ref["grip_options"], key=near)   # untried first (stable: narrowest first within)
        S.jaw_used = S.weighed_kg = S.grip_width_used_cm = S.lean_used_deg = None; fail, held = grasp_to_bin(S, ref, q_ready, q_bin); S.tries = k + 1
        jaw = getattr(S, "jaw_used", None)
        S.try_log.append({"fail": fail, "width_cm": getattr(S, "grip_width_used_cm", None), "lean_deg": getattr(S, "lean_used_deg", None),
                          "jaw": None if jaw is None else [round(float(x), 3) for x in jaw], "weighed_kg": getattr(S, "weighed_kg", None)})
        if getattr(S, "jaw_used", None) is not None: used.append(S.jaw_used)
        if fail not in RETRYABLE: break
    return fail, held


def same_as(xy, handled, tol=0.15, this=None):
    """the number of an object already handled tonight within tol (m) of xy, or None ([XfI]); `this`: the item being
    handled now, never the same as itself (a re-park looks at it again)"""
    for n, h in handled:
        if n != this and np.linalg.norm(np.asarray(xy)[:2] - np.asarray(h)[:2]) < tol: return n
    return None


def stow(S, q_ready):
    S.state("STOW"); S.arm_to_safe(q_ready, 2.0)


# --- learned pick: a trained policy drives the arm from the look-closer handover ---
class PolicyClient:
    """a LeRobot checkpoint served by training/sim/policy_server.py (the training environment) over a pipe"""
    def __init__(self, checkpoint):
        import pickle, subprocess
        from pathlib import Path
        hk = Path(__file__).resolve().parents[2]; self._pk = pickle
        self.p = subprocess.Popen([str(hk / "training/.venv/bin/python"), str(hk / "training/sim/policy_server.py"), str(checkpoint)],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.info = pickle.load(self.p.stdout)
    def ask(self, req): self._pk.dump(req, self.p.stdin); self.p.stdin.flush(); return self._pk.load(self.p.stdout)
    def close(self):
        try: self.p.stdin.close(); self.p.wait(5)
        except Exception: self.p.kill()


def ee_target(K, q6, act):
    """a gripper-frame move (dx, dy, dz, rotation vector) from the arm pose q6 -> joint targets (damped least squares).
    K: the robot's kinematic model (m, kin, aq, ad, rng); the base pose must already be in K.kin/K.d."""
    import mujoco
    from scipy.spatial.transform import Rotation as Rot
    m, k = K.m, K.kin; G = m.body("arm_grasp_center").id
    k.qpos[:] = K.d.qpos; k.qpos[K.aq] = q6; mujoco.mj_kinematics(m, k); mujoco.mj_comPos(m, k)
    R0 = k.xmat[G].reshape(3, 3).copy(); p_t = k.xpos[G] + R0 @ act[:3]; R_t = R0 @ Rot.from_rotvec(act[3:6]).as_matrix(); q = np.array(q6, float)
    for _ in range(30):
        k.qpos[K.aq] = q; mujoco.mj_kinematics(m, k); mujoco.mj_comPos(m, k)
        ep = p_t - k.xpos[G]; er = Rot.from_matrix(R_t @ k.xmat[G].reshape(3, 3).T).as_rotvec()
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(er) < 1e-3: break
        Jp = np.zeros((3, m.nv)); Jr = np.zeros((3, m.nv)); mujoco.mj_jacBody(m, k, Jp, Jr, G)
        J = np.vstack([Jp[:, K.ad], 0.3 * Jr[:, K.ad]])
        q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), np.r_[ep, 0.3 * er]), K.rng[:, 0], K.rng[:, 1])
    return q


def policy_pick(S, client, ee=True, steps=110, query_every=10):
    """From the handover pose, let the policy drive the arm at 10 Hz. S provides small_image() (96x128 RGB as in the
    demos), joint_state() (6 joints + jaw), kin_model() (its kinematic model) and command(q6, jaw) (hold for 0.1 s).
    Returns the jaw position at the end (the caller judges holding, as the scripted grasp does)."""
    client.ask({"reset": True}); t = 0
    while t < steps:
        st = np.asarray(S.joint_state(), np.float32); chunk = client.ask({"image": S.small_image(), "state": st})
        for act in chunk[:query_every]:
            st = np.asarray(S.joint_state(), np.float32)
            q6 = ee_target(S.kin_model(), st[:6], act) if ee else act[:6]
            S.command(q6, float(np.clip(act[6], 0.0, OPEN))); t += 1
            if t >= steps: break
    return S.gripper()


def carry_to_bin(S, q_ready, q_bin, hang=0.15):
    """after a pick by any means: up through the ready pose, over the bin high enough for an item hanging `hang` m
    below the jaws (its longest side, if known; with HIGH_CARRY as much as any item could, [2ho]), lowered in, released.
    Returns a fail reason or None."""
    if HIGH_CARRY: hang = HANG_MAX
    try:
        if S.gripper() < HELD_MIN or S.gripper() > OPEN - 0.005: return "grasp (not lifted)"
        q_over = S.over_bin(hang)
        S.state("INTO BIN"); S.waypoints = [q_ready]; S.arm_to_safe(q_bin if q_over is None else q_over, 3.0)
        if S.gripper() < HELD_MIN: return "lost on the way to the bin (may be in it)"
        if q_over is not None: release_in_bin(S, max_m=0.12 + hang)
        else: S.grip_to(OPEN, 0.6); S.step(120)
        return None
    finally:
        set_payload(S, 0.0)                                  # released or lost: nothing held ([qaS])


def handover_pose(S, xy):
    """the camera straight down just short of the item: where look-closer hands over, and where demonstrations start"""
    out = np.asarray(xy) - S.base_xy(); out = out / (np.linalg.norm(out) + 1e-9)
    k = framing(S)
    return S.look_down(np.asarray(xy) - out * IN_VIEW * k, out, h_cam=min((LOOK_H + 0.14) * k, FRAME_H_MAX))
