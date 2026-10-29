#!/usr/bin/env python3
"""Confirm on approach ([d8d], [QDc]), replayed on a recorded sweep: the robot judges each floor change from the head
camera's frames as it walks up to it, instead of waiting for the finished night's floor picture.

Each processed frame paints the floor it sees (0.5 to --rmax m ahead) and is compared with the baseline floor
(median of the previous nights' pictures), using the locked detector's own comparison (detect_changes2.feats,
tolerant_diff, TC; its own brightening threshold, CONFIRM['tb']) and its masks (floor near mapped structures skipped, the edge of the seen floor not trusted).
Every floor cell counts the frames that saw it and the frames that found it changed, and when. A spot is
  confirmed  once at least --hits frames found it changed, at least --ratio of the frames that saw it, over at least
             --span m of approach (views from different distances: a registration slip or a glint doesn't repeat);
             and the blob passes the detector's own "something is here" test on the closest view so far;
  unsure     if it's still undecided when the robot is --unsure-at m away: the cue for a sidestep ([d8d]);
  dropped    if the frames that saw it mostly didn't find it changed.
Only what the robot could know at that moment is used: previous nights' pictures, maps and tall layers.
Each frame is placed with its own camera's lag, the robot's stored one by default ([gJq]).
The replay here is the robot's side and scores nothing; score_confirm.py runs it and scores it against the sim's truth
as detect_changes2.py scores (an item is found if a confirmed blob falls within its size + 6 cm).
Usage: confirm_stream.py --run-dir runs/nights_s8_r1 [--seed 8] --night 5 --db <memory.db> [--every 6]   (dimos venv python)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper.lens
import argparse, json, time
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
from dimos.memory.store.sqlite import SqliteStore
import floor_sample
from detect_changes2 import feats, tolerant_diff, TC, TB, MIN_PX, BIG_PX, RES, HX, HY, w2rc
from floor_mosaic2 import arr, frame_lags, LAG_FILE, clock_of, add_clock_args

W, H = int(2 * HX / RES), int(2 * HY / RES)


def grid(res=RES):
    """the floor grid's (H, W) at res m a cell: the 1 cm grid's own, or a finer one over the same floor, each 1 cm cell
    split k x k (k = RES / res, a whole number) so a fine cell lies inside one coarse cell ([H9w], the near grid)"""
    k = int(round(RES / res))
    if k < 1 or abs(RES / k - res) > 1e-9: raise SystemExit(f"a grid of {res} m does not divide the 1 cm grid")
    return (H, W) if k == 1 else (H * k, W * k)


def upsample(x, k):
    """a 1 cm grid's array on a grid k times finer (each cell repeated k x k)"""
    return x if k == 1 else np.repeat(np.repeat(x, k, 0), k, 1)
RMIN = 0.5                                        # nearer than this the floor is under the head (est.)
# the confirmer's settings, shared by this replay and the live tidy (tidy_module): tuned on Filament's photographed,
# night-lit house (seed 70, nights 2-3, 26 Sep, decision in the sim-slice spec): the brightening cue at the detector's
# 0.30 fired on glossy wood's moving lamp highlights; 1.0 keeps it for strongly lighter things only. 27 Sep, realistic
# bench real1 in odometry's frame (night_tidy v7), candidates a night near no toy / toys: these settings 101 / 17 of 24;
# with the registration layers ([WFI]: smooth 1, nearest views, snap, noise-aware colour, the slop check) 62 / 16; with the
# baselines stitched and the mount calibrated (night_tidy.sh's stitch step), tracking on: 50 / 18-22 (the mount from
# odometry alone / from the sim's true path)
CONFIRM = dict(rmax=2.5, hits=4, ratio=0.6, span=0.9, unsure_at=1.2, small_px=8, small_peak=0.6, tb=1.0, tc=0.045, band_m=0.20, parallax=0.0, parallax_move=0.25, group_m=0.0, smooth=1.0, mean_score=0.0, slack=2, snap=True, noise_k=1.0, slop=6, slop_frac=0.3, track=True, track_gain=0.5, track_rot_sign=-1.0)
# named settings over CONFIRM, chosen with --setting NAME (the replay) or HK_DETECT_SETTING (the live tidy); explicit
# options still override them. "launch" is [tTD]'s: the full-resolution round's near choice (floor_arm's loose1 with
# the near-choice options of the full-resolution round, 11 Oct 2026 ([bFx]), r11 M/F_sl1_l3_N: the 5 mm near grid needs the baselines'
# near views, --near-views), then the light check per candidate ([Zaf]). Not the default: whether it becomes one is decided once confirmed
# The colour threshold is 0.045, not the round's stated 0.032: every launch number was measured at 0.045 (the cached
# replays judged at the paint's, [LQs]); painted live at 0.032 the lead keeps about 35 false alarms a night
# (docs/research/cascade-onboard-2026-10-13.md, "The honest launch arm")
SETTINGS = {"launch": dict(hits=2, ratio=0.4, span=0.3, unsure_at=1.6, slop_frac=0.2, view_noise=4.0, tc=0.045, tb=0.6,
                           slop=0, small_px=4, small_peak=0.4, slack=1, near_res=0.005, near_rmax=1.0,
                           near_opt=["span=0.3", "min_cm2=0.5"], check=True)}


class _Parser(argparse.ArgumentParser):
    """an option parser that lays a named setting (--setting NAME, SETTINGS) under the options given"""
    def parse_known_args(self, args=None, namespace=None):
        import sys as _s
        args = list(_s.argv[1:] if args is None else args)
        for i, x in enumerate(args):
            name = args[i + 1] if x == "--setting" and i + 1 < len(args) else x.split("=", 1)[1] if x.startswith("--setting=") else None
            if name is not None:
                if name not in SETTINGS: self.error(f"--setting: one of {sorted(SETTINGS)}")
                self.set_defaults(**SETTINGS[name])
        return super().parse_known_args(args, namespace)


def smooth(rgb, valid, sigma):
    """a floor patch averaged over sigma cells within where it was seen (the camera's grain and JPEG blocks, [nAH] kJ6,
    average out; a toy, several cm across, doesn't)"""
    if sigma <= 0: return rgb
    V = valid.astype(np.float32); den = np.maximum(ndi.gaussian_filter(V, sigma), 1e-3)
    f = np.stack([ndi.gaussian_filter(rgb[..., k].astype(np.float32) * V, sigma) / den for k in range(3)], -1)
    return np.where(valid[..., None], np.clip(f, 0, 255), 0).astype(np.uint8)


NB, BAND_M = 4, 0.5                               # distance bands 0.5-1, 1-1.5, 1.5-2, 2-2.5 m


def track_step(td, dr, dc, P, gain=CONFIRM["track_gain"], rot_sign=CONFIRM["track_rot_sign"]):
    """the floor snap's lock carried to the pose (world <- believed, 3x3), a share `gain` of it: the reference turned by
    td deg (ndi.rotate, rows north to south) and rolled by (-dr, -dc) cells lay on tonight's frame over the grid window
    P, so the frame's floor sits turned and shifted that much from the truth, and the pose is turned back about the
    window's middle. ndi.rotate's +td turns the floor anticlockwise with north up, so rot_sign is -1 (critique, 3 Oct)"""
    phi = np.radians(td) * rot_sign * gain
    pc = np.array([-HX + ((P[1].start + P[1].stop) / 2) * RES, HY - ((P[0].start + P[0].stop) / 2) * RES])
    c_, s_ = np.cos(phi), np.sin(phi); R2 = np.array([[c_, -s_], [s_, c_]])
    S = np.eye(3); S[:2, :2] = R2; S[:2, 2] = pc - R2 @ pc + gain * np.array([dc * RES, -dr * RES])
    return S


def lock_pose(td, dr, dc, P, rot_sign=CONFIRM["track_rot_sign"]):
    """a floor lock as a measurement ([x9X]): (S, centre): the house-frame correction the whole lock asks of the frame's
    pose (track_step at gain 1), and the point it turns about (the grid window's middle)"""
    pc = np.array([-HX + ((P[1].start + P[1].stop) / 2) * RES, HY - ((P[0].start + P[0].stop) / 2) * RES])
    return track_step(td, dr, dc, P, 1.0, rot_sign), pc


def poses(st):
    """the robot's pose at a frame's own time (world <- base) and the head camera on the body, as floor_mosaic2.py
    reads them ([Feg])"""
    from houndkeeper.rec_poses import RecordedPoses
    pose = RecordedPoses(st); T_bc, T_co = pose.mount()
    return pose, T_bc @ T_co


class Views:
    """the baseline nights' own views of the floor ([KkQ]): each kept frame's painted patch, where its camera stood and
    which way it faced. A frame tonight is compared with the baseline as seen from about the same place: the camera's
    mount and lens errors, the floor's relief and its gloss then fall the same way in both, where a picture blended from
    views at other distances smears them (27 Sep: the slate tile's baseline a mush of views cm apart)"""
    NEAR_M, NEAR_DEG, KEEP = 0.40, 25.0, 3

    def __init__(self, C, F, sls, patches, ts=None, Ts=None, res=RES):
        self.C, self.F, self.sls, self.patches = np.asarray(C, float), np.asarray(F, float), np.asarray(sls, int), patches
        self.res = float(res)                                                         # the grid the patches are painted on
        self.ts = np.asarray(ts if ts is not None else np.zeros(len(self.C)), float)   # each view's time (its pose's; stitch_floor)
        self.Ts = None if Ts is None or not len(Ts) else np.asarray(Ts, float)        # its camera (world <- optical): the
                                                                                        # stitch's mount solve needs the whole pose

    @staticmethod
    def of_frame(T):
        """where a camera (world <- optical) stood and faced, on the floor plane"""
        f = T[:3, 2][:2]; return T[:2, 3].copy(), f / max(np.linalg.norm(f), 1e-9)

    def save(self, path, cal=None):
        """cal: the calibration the views were painted with (calibrate.stamp), kept with them ([Q9k])"""
        flat = np.concatenate([p.reshape(-1) for p in self.patches]) if self.patches else np.zeros(0, np.uint8)
        np.savez_compressed(path, C=self.C, F=self.F, sls=self.sls, flat=flat, ts=self.ts,
                            Ts=self.Ts if self.Ts is not None else np.zeros((0, 4, 4)), **({} if self.res == RES else {"res": np.float64(self.res)}),
                            **{k: np.str_(v) for k, v in (cal or {}).items()})

    @classmethod
    def load(cls, paths):
        C, F, sls, patches, ts, Ts, res = [], [], [], [], [], [], set()
        for p in paths:
            z = np.load(p); o = 0; flat = z["flat"]                  # once: each z[...] decompresses the whole array again
            res.add(float(z["res"]) if "res" in z.files else RES)
            ts += list(z["ts"]) if "ts" in z else [0.0] * len(z["C"])
            Ts += list(z["Ts"]) if "Ts" in z and len(z["Ts"]) == len(z["C"]) else []
            for c, f, s_ in zip(z["C"], z["F"], z["sls"]):
                n = (s_[1] - s_[0]) * (s_[3] - s_[2]) * 3; patches.append(flat[o:o + n].reshape(s_[1] - s_[0], s_[3] - s_[2], 3)); o += n
                C.append(c); F.append(f); sls.append(s_)
        if len(res) > 1: raise SystemExit(f"views painted on different grids: {sorted(res)}")
        return cls(C, F, sls, patches, ts, Ts if len(Ts) == len(C) else None, res.pop() if res else RES) if C else None

    def reference(self, T, sl):
        """the baseline over slice sl as seen from nearest where this camera stands and faces: (rgb, seen) or None"""
        c, f = self.of_frame(T)
        d = np.hypot(*(self.C - c).T); ang = np.degrees(np.arccos(np.clip(self.F @ f, -1, 1)))
        ok = np.nonzero((d < self.NEAR_M) & (ang < self.NEAR_DEG))[0]
        if not len(ok): return None
        ok = ok[np.argsort(d[ok] + 0.01 * ang[ok])][:self.KEEP]
        r0, r1, c0, c1 = sl[0].start, sl[0].stop, sl[1].start, sl[1].stop
        out = np.zeros((r1 - r0, c1 - c0, 3), np.uint8); have = np.zeros((r1 - r0, c1 - c0), bool)
        for i in ok:                                                  # the nearest view first; the next fill its gaps
            a0, a1, b0, b1 = self.sls[i]; R0, R1, C0, C1 = max(a0, r0), min(a1, r1), max(b0, c0), min(b1, c1)
            if R1 <= R0 or C1 <= C0: continue
            src = self.patches[i][R0 - a0:R1 - a0, C0 - b0:C1 - b0]; m = src.any(-1) & ~have[R0 - r0:R1 - r0, C0 - c0:C1 - c0]
            out[R0 - r0:R1 - r0, C0 - c0:C1 - c0][m] = src[m]; have[R0 - r0:R1 - r0, C0 - c0:C1 - c0] |= m
        return (out, have) if have.any() else None

    def noise(self, sigma=0.0, min_views=3):
        """how much each floor cell's colour and brightness (feats, as the confirmer compares them) vary between the empty
        nights' own views of it ([H9w]): glare and gloss that move with the view, a pattern's edges where views register a
        cm apart, a dark floor's sensor noise all show here, from nights the robot knows were empty. Returns (colour sd,
        brightness sd, views) on the grid; cells seen by fewer than min_views views get sd 0 (no say)"""
        Hg, Wg = grid(self.res); fs = 25 * RES / self.res                # the surround: 25 cm on any grid
        n = np.zeros((Hg, Wg), np.float32); sc = np.zeros((Hg, Wg, 3), np.float32); sc2 = np.zeros((Hg, Wg), np.float32)
        sr = np.zeros((Hg, Wg), np.float32); sr2 = np.zeros((Hg, Wg), np.float32)
        for p, (a0, a1, b0, b1) in zip(self.patches, self.sls):
            V = p.any(-1)
            if V.sum() < 50: continue
            c, r = feats(smooth(p, V, sigma), V, fs); A = (slice(a0, a1), slice(b0, b1))
            n[A] += V; sc[A] += np.where(V[..., None], c, 0); sc2[A] += np.where(V, (c ** 2).sum(-1), 0)
            sr[A] += np.where(V, r, 0); sr2[A] += np.where(V, r ** 2, 0)
        k = np.maximum(n, 1)
        vc = np.maximum(sc2 / k - ((sc / k[..., None]) ** 2).sum(-1), 0); vr = np.maximum(sr2 / k - (sr / k) ** 2, 0)
        ok = n >= min_views
        return np.where(ok, np.sqrt(vc), 0).astype(np.float32), np.where(ok, np.sqrt(vr), 0).astype(np.float32), n


class Confirmer:
    """The evidence kept while walking: per floor cell, frames that saw it, frames that found it changed, the nearest
    and farthest robot distance of those, and the closest view's colour (a running floor picture for tonight)."""

    def __init__(self, base, Vb, band, hmap, K, img_wh, a, res=RES):
        self.a, self.K, (self.iw, self.ih) = a, K, img_wh
        self.res, self.k = float(res), int(round(RES / res))        # the grid (m a cell) and how many of its cells a 1 cm
        H, W = self.H, self.W = grid(res)                           # cell spans across; on a finer grid ([H9w], the near
        if base.shape[:2] != (H, W): raise SystemExit(f"a baseline of {base.shape[:2]} on a grid of {(H, W)}")   # grid) the
        self.fs = 25 * self.k                                       # comparison's sizes in cm keep their size in cm
        self.base, self.Vb, self.band, self.hmap = base, Vb, band, hmap
        self.cb, self.rb = feats(smooth(base, Vb, getattr(a, "smooth", 0.0)), Vb, self.fs)
        self.seen = np.zeros((H, W), np.int16); self.hit = np.zeros((H, W), np.int16)
        self.score = np.zeros((H, W), np.float32)                  # the sum over frames of how changed each cell looked
        self.ref_rgb = np.zeros((H, W, 3), np.uint8); self.ref_V = np.zeros((H, W), bool)   # the reference each cell's
                                                                    # closest view was compared with (for the slop check)
        self.snaps: list = []
        self.relief = None                                         # the floor's relief (m, the floor grid; floor_relief):
                                                                    # frames laid on the floor where it stands, not z = 0 ([4t7])
        self.Cf = np.eye(3)                                         # --track: believed -> the floor's frame, kept up by each lock                                       # each frame's snap (--snap): turn, shift, score, lead, locked
        self.seen_b = np.zeros((NB, H, W), np.int16); self.hit_b = np.zeros((NB, H, W), np.int16)   # by distance band
        self.dmin = np.full((H, W), np.inf, np.float32); self.dmax = np.zeros((H, W), np.float32)
        self.peak = np.zeros((H, W), np.float16)                   # strongest change seen, for the detector's tiny-and-faint rule
        self.rgb = np.zeros((H, W, 3), np.uint8); self.dbest = np.full((H, W), np.inf, np.float32)
        self.gx = -HX + (np.arange(W) + .5) * self.res; self.gy = HY - (np.arange(H) + .5) * self.res
        self.decided = {}                                          # blob key -> decision record; "unsure" can still be settled
        if getattr(a, "fuse", 0) > 0:                              # --fuse ([H9w]): each cell's change pooled over every view
            self.wsum = np.zeros((H, W), np.float32); self.wscore = np.zeros((H, W), np.float32)   # before any threshold:
            self.smin = np.full((H, W), np.inf, np.float32); self.smax = np.zeros((H, W), np.float32)   # weights, scores, seen range
        if getattr(a, "logodds", False):                           # [H9w]: the binary Bayes filter's log-odds of "changed",
            self.L = np.zeros((H, W), np.float32)                  # per cell (occupancy-grid form, Thrun et al.), clamped
            self.Lsum = np.zeros((H, W), np.float32)               # and the same sum unclamped: a ranking score ([HGT] curves)
        if getattr(a, "parallax", 0) > 0:                          # tonight's first view of each cell (normalised colour and
            self.v1_cn = np.zeros((H, W, 3), np.float16)           # brightness) and where the camera was: a later view from
            self.v1_rn = np.zeros((H, W), np.float16)              # far enough away that disagrees has seen something with
            self.v1_cam = np.full((H, W, 2), np.nan, np.float32)   # height (the floor looks the same from anywhere)

    def cells(self, n):
        """n cells of the 1 cm grid as cells of this one (the same size on the floor)"""
        return int(round(n * self.k))

    def frame(self, img, T, K=None, wh=None, mask=None):
        """Paint and compare one frame; returns the patch's slice (for timing / debugging). K, wh, mask: another camera's
        pinhole, image size and pixels showing the robot itself (the wrist camera's, beside the head's)"""
        if getattr(self.a, "track", False):                          # the floor's running correction to the pose ([fXD])
            M4 = np.eye(4); M4[:2, :2] = self.Cf[:2, :2]; M4[:2, 3] = self.Cf[:2, 2]; T = M4 @ T
        painted = self.paint(img, T, K, wh, mask)
        if painted is None: return None
        sl, pr, pc_, dd, patch, V = painted; ph, pw = V.shape; C = T[:3, 3]; self._near = None
        sm = getattr(self.a, "smooth", 0.0)
        cn, rn = feats(smooth(patch, V, sm), V, self.fs)
        cb, rb = self.cb[sl], self.rb[sl]; self._refp = (self.base[sl], self.Vb[sl])
        views, snapping = getattr(self, "views", None), getattr(self.a, "snap", False)
        if views is not None or snapping:                            # the reference: the baseline as seen from about here
            ref = self._reference(T, sl, patch, V)                  # ([KkQ]), laid on this frame's floor where it locks
            if ref is not None:
                rp, rv = ref; cbv, rbv = feats(smooth(rp, rv, sm), rv, self.fs)
                self._refp = (np.where(rv[..., None], rp, self.base[sl]), rv | self.Vb[sl])
                cb = np.where(rv[..., None], cbv, cb); rb = np.where(rv, rbv, rb)
        dC, bright = tolerant_diff(cn, rn, cb, rb, slack=int(getattr(self.a, "slack", 2)))
        if getattr(self.a, "dark", 0) > 0:                           # [H9w]: where the floor is near black (the map's picture or
            g_ = np.minimum(ndi.gaussian_filter(self._refp[0].astype(np.float32).mean(-1), 1.0),   # tonight's) its colour is the
                            ndi.gaussian_filter(patch.astype(np.float32).mean(-1), 1.0))           # sensor's noise, not the
            dC = np.where(g_ < self.a.dark, 0.0, dC)                 # floor's: only a brightening counts there (96's dark tile)
        self._tc = self._colour_threshold(patch, V)
        if getattr(self, "cache", None) is not None:               # the comparison's output, for replaying the evidence and
            self.cache.add(sl, pr, pc_, dd, patch, V, C, dC, bright, self._tc, self._refp, self._near)   # decisions alone
        return self._judge(sl, pr, pc_, dd, patch, V, ph, pw, C, cn, rn, dC, bright)

    def _colour_threshold(self, patch, V):
        """the colour shift that counts as a change, cell by cell: TC, or more where the floor is dark enough for the
        frame's own noise to shift its colour that much ([nAH] kJ6: at night the sensor's gain rises; dark carpet away
        from the lamp was half the stops for nothing, 27 Sep). The frame's noise is measured from the frame itself (the
        spread of each cell against its neighbours), as a real camera's would be"""
        k = getattr(self.a, "noise_k", 0.0)
        if k <= 0: return self.a.tc
        g = patch.astype(np.float32).mean(-1); v = V.astype(np.float32)
        near = ndi.gaussian_filter(g * v, 1.0) / np.maximum(ndi.gaussian_filter(v, 1.0), 1e-3)
        sig = 1.4826 * float(np.median(np.abs(g - near)[V])) if V.any() else 0.0
        sm = max(getattr(self.a, "smooth", 0.0), 0.5)                        # smoothing over ~sm cells averages it down
        lvl = ndi.gaussian_filter(g * v, 3.0 * self.k) / np.maximum(ndi.gaussian_filter(v, 3.0 * self.k), 1e-3)
        return np.maximum(self.a.tc, k * sig / (2.5 * sm) / np.maximum(lvl, 4.0))

    SNAP_TAU, SNAP_FLOOR_CELLS, SNAP_Q_CELLS, SNAP_MIN_NCC = 0.05, 0.5, 0.5, 0.10

    def _snap_weighted(self, Fp, Fm, M, Mm):
        """the reference laid on this frame's floor by every frame's say, weighed (the
        strict lock took 22 of 4,770 frames): each frame's whole correlation surface gives a belief
        over the shift (fr.surface_belief: a sharp peak firm, a ridge firm across it only, two peaks hardly at all), and a
        running estimate along the path takes each in by its weight (a Kalman filter: the offset drifts a little frame to
        frame, the floor's evidence pulls it back). The filtered shift, fractional, is applied every frame.
        Returns (M, Mm, applied)"""
        from houndkeeper import floor_register as fr
        if not hasattr(self, "_off"): self._off, self._Poff = np.zeros(2), np.eye(2) * 16.0     # a 4 cm prior on the pose's slip
        self._Poff = self._Poff + np.eye(2) * self.SNAP_Q_CELLS ** 2                            # it drifts between frames
        reach = int(getattr(self.a, "snap_reach", fr.SNAP_MAX_CELLS))
        S = fr.ncc_surfaces(Fp, Fm, M.mean(-1).astype(np.float32), Mm, reach, (0.0,), 1.0)
        rec = None
        if S is not None:
            c = S[0][1]; best = float(c.max())
            if best >= self.SNAP_MIN_NCC:
                b = fr.surface_belief(c, float(getattr(self.a, "snap_tau", self.SNAP_TAU)))
                if b is not None:
                    z = b[0] - reach; Rm = b[1] + np.eye(2) * self.SNAP_FLOOR_CELLS ** 2
                    Kg = self._Poff @ np.linalg.inv(self._Poff + Rm); self._off = self._off + Kg @ (z - self._off)
                    self._Poff = (np.eye(2) - Kg) @ self._Poff
                    pk = np.array(np.unravel_index(np.argmax(c), c.shape)) - reach
                    rec = (0.0, round(float(z[0]), 2), round(float(z[1]), 2), round(best, 3), round(float(np.sqrt(np.trace(b[1]) / 2)), 2),
                           bool(np.sqrt(np.trace(b[1]) / 2) <= 2.0), int(pk[0]), int(pk[1]))   # informative: within ~2 cm; the peak
        self.snaps.append(rec)
        dr, dc = float(self._off[0]), float(self._off[1])
        if abs(dr) < 0.25 and abs(dc) < 0.25: return M, Mm, False
        M = np.stack([ndi.shift(M[..., k].astype(np.float32), (-dr, -dc), order=1, mode="nearest") for k in range(3)], -1)
        Mm = ndi.shift(Mm.astype(np.float32), (-dr, -dc), order=0) > 0.5
        return M, Mm, True

    def _reference(self, T, sl, patch, V):
        """the baseline under this frame's patch: the nearest earlier views where there are any (Views), the blended
        picture elsewhere; then, with --snap, turned and shifted onto the frame's own floor where the floor's edges lock
        (floor_register.snap: the pose is a few cm off, and a tile's edge says by how much). Cells already found changed
        are left out of the matching, so a find doesn't drag it. Returns (rgb, seen) over sl, or None: no change"""
        from houndkeeper import floor_register as fr
        pad = max(fr.SNAP_MAX_CELLS, int(getattr(self.a, "snap_reach", 0))) + 3   # room round the patch for the widest search
        P = (slice(max(sl[0].start - pad, 0), min(sl[0].stop + pad, self.H)), slice(max(sl[1].start - pad, 0), min(sl[1].stop + pad, self.W)))
        o0, o1 = sl[0].start - P[0].start, sl[1].start - P[1].start; h, w = V.shape
        M = self.base[P].copy(); Mm = self.Vb[P].copy(); changed = False
        if getattr(self, "views", None) is not None:
            vr = self.views.reference(T, P)
            if vr is not None: M[vr[1]] = vr[0][vr[1]]; Mm |= vr[1]; changed = True
            self._near = None if vr is None else vr[1][o0:o0 + h, o1:o1 + w]   # cells a near view covers (the sensor model's quality)
            if getattr(self, "dirmodel", None) is not None and getattr(self.a, "dir_ref", False):   # [4t7]: where no near view
                dr_, dh_ = self.dirmodel.mean_rgb(P, T[:3, 3])        # covers, the floor as seen from this direction instead of
                fill = dh_ & ~(vr[1] if vr is not None else np.zeros(dh_.shape, bool))   # the picture blended from every one
                if fill.any():
                    M[fill] = dr_[fill]; Mm |= fill; changed = True
                    self._near = (self._near if self._near is not None else np.zeros((h, w), bool)) | fill[o0:o0 + h, o1:o1 + w]
        if getattr(self.a, "snap", False) and getattr(self.a, "snap_mode", "lock") == "weighted":
            Fp = np.zeros(M.shape[:2], np.float32); Fm = np.zeros(M.shape[:2], bool)
            Fp[o0:o0 + h, o1:o1 + w] = patch.mean(-1); Fm[o0:o0 + h, o1:o1 + w] = V & (self.hit[sl] < 2)
            M, Mm, ch = self._snap_weighted(Fp, Fm, M, Mm)
            changed = changed or ch
        elif getattr(self.a, "snap", False):
            Fp = np.zeros(M.shape[:2], np.float32); Fm = np.zeros(M.shape[:2], bool)
            Fp[o0:o0 + h, o1:o1 + w] = patch.mean(-1); Fm[o0:o0 + h, o1:o1 + w] = V & (self.hit[sl] < 2)
            s_ = fr.snap(Fp, Fm, M.mean(-1).astype(np.float32), Mm)
            lock = s_ is not None and s_[3] >= fr.SNAP_MIN_SCORE and s_[4] >= fr.SNAP_MIN_LEAD
            self.snaps.append(None if s_ is None else (round(float(np.degrees(s_[0])), 2), s_[1], s_[2], round(s_[3], 3), round(s_[4], 3), lock))
            if lock:
                td, dr, dc = float(np.degrees(s_[0])), int(s_[1]), int(s_[2])
                if td:
                    M = np.stack([ndi.rotate(np.where(Mm, M[..., k], 0).astype(np.float32), td, reshape=False, order=1) for k in range(3)], -1)
                    Mm = ndi.rotate(Mm.astype(np.float32), td, reshape=False, order=0) > 0.5
                M = np.roll(M, (-dr, -dc), (0, 1)); Mm = np.roll(Mm, (-dr, -dc), (0, 1)); changed = True
                if getattr(self, "on_lock", None) is not None:       # [x9X]: the lock, whole, a constraint in the house's
                    self.on_lock(*lock_pose(td, dr, dc, P, float(getattr(self.a, "track_rot_sign", CONFIRM["track_rot_sign"]))),
                                 float(s_[3]), float(s_[4]))         # graph (live_place), the changed cells left out above
                if getattr(self.a, "track", False):                  # the pose is that far off: carry it to the next frame
                    self.Cf = track_step(td, dr, dc, P, float(getattr(self.a, "track_gain", CONFIRM["track_gain"])),
                                         float(getattr(self.a, "track_rot_sign", CONFIRM["track_rot_sign"]))) @ self.Cf
        if not changed: return None
        return np.clip(M[o0:o0 + h, o1:o1 + w], 0, 255).astype(np.uint8), Mm[o0:o0 + h, o1:o1 + w]

    def paint(self, img, T, K=None, wh=None, mask=None):
        """one frame's floor, painted on the grid where it sees it: (slice, rows, cols, distances, patch, seen) or None"""
        fx, fy, cx, cy = K or self.K; iw, ih = wh or (self.iw, self.ih); R, C = T[:3, :3], T[:3, 3]; rmax = self.a.rmax
        c0, c1 = np.searchsorted(self.gx, C[0] - rmax), np.searchsorted(self.gx, C[0] + rmax)
        r0, r1 = np.searchsorted(-self.gy, -(C[1] + rmax)), np.searchsorted(-self.gy, -(C[1] - rmax))
        if c1 <= c0 or r1 <= r0: return None
        X, Y = np.meshgrid(self.gx[c0:c1], self.gy[r0:r1])
        Zf = self.relief[r0:r1, c0:c1] if getattr(self, "relief", None) is not None else 0.0
        P = np.stack([X - C[0], Y - C[1], np.full_like(X, -C[2]) + Zf], -1); pc = P @ R; z = pc[..., 2]; ok = z > 0.1
        u = fx * pc[..., 0] / np.where(ok, z, 1) + cx; v = fy * pc[..., 1] / np.where(ok, z, 1) + cy
        d = np.hypot(P[..., 0], P[..., 1])
        ok &= (u >= 0) & (u < iw - 1) & (v >= 0) & (v < ih - 1) & (d < rmax) & (d > RMIN)
        if mask is not None: ok &= ~mask[np.clip(np.rint(v), 0, ih - 1).astype(int), np.clip(np.rint(u), 0, iw - 1).astype(int)]   # [AKY]
        if not ok.any(): return None
        rr, cc = np.nonzero(ok); tx, ty, dd = X[rr, cc], Y[rr, cc], d[rr, cc]; vis = np.ones(len(rr), bool)
        for f in np.linspace(0.03, 0.97, 24):                      # line of sight vs the height map, as floor_mosaic2
            sx, sy = tx + (C[0] - tx) * f, ty + (C[1] - ty) * f; zr = C[2] * f; near = dd * f < 0.04
            hi_, hj_ = ((HY - sy) / self.res).astype(int), ((sx + HX) / self.res).astype(int)
            inb = (hi_ >= 0) & (hi_ < self.H) & (hj_ >= 0) & (hj_ < self.W)
            hgt = np.zeros(len(rr), np.float32); hgt[inb] = self.hmap[hi_[inb], hj_[inb]]
            vis &= ~((hgt > zr + 0.01) & ~near)
        rr, cc, dd = rr[vis], cc[vis], dd[vis]
        if len(rr) < 50: return None
        # the frame's own floor patch, then the detector's comparison on it
        sl = (slice(r0 + rr.min(), r0 + rr.max() + 1), slice(c0 + cc.min(), c0 + cc.max() + 1))
        pr, pc_ = rr - rr.min(), cc - cc.min(); ph, pw = sl[0].stop - sl[0].start, sl[1].stop - sl[1].start
        patch = np.zeros((ph, pw, 3), np.uint8); V = np.zeros((ph, pw), bool); V[pr, pc_] = True
        patch[pr, pc_] = np.clip(floor_sample.sample(floor_sample.pyramid(arr(img)), u[rr, cc], v[rr, cc],   # each cell the
                                 floor_sample.px_per_cell(self.res, np.hypot(dd, C[2]), fy, C[2])), 0, 255).astype(np.uint8)   # mean of
                                                                   # its pixels, as the floor picture reads them ([FHA])
        return sl, pr, pc_, dd, patch, V

    def _judge(self, sl, pr, pc_, dd, patch, V, ph, pw, C, cn, rn, dC, bright):
        """a painted, compared frame's evidence, added to the cells it saw"""
        region = V & self.Vb[sl] & ~self.band[sl]
        core = ndi.binary_erosion(region, iterations=self.cells(6)) & region
        tc = getattr(self, "_tc", self.a.tc); tb = self.a.tb
        if getattr(self, "dirmodel", None) is not None:              # the empty nights' spread here as seen from this
            k_ = self.a.view_noise; sc_, sr_ = self.dirmodel.spread(sl, C, int(getattr(self.a, "dir_min_views", 3)), float(getattr(self.a, "dir_prior", 0.0)))   # direction ([4t7])
            if getattr(self.a, "dir_floor", 0) > 0 and getattr(self, "vnoise", None) is not None:   # never under this share of
                f_ = self.a.dir_floor; sc_ = np.maximum(sc_, f_ * self.vnoise[0][sl]); sr_ = np.maximum(sr_, f_ * self.vnoise[1][sl])   # every direction's
            nr_ = getattr(self, "_near", None)
            if getattr(self.a, "dir_near_only", False) and getattr(self, "vnoise", None) is not None:   # the reference was a near
                nr_ = nr_ if nr_ is not None else np.zeros(sc_.shape, bool)     # view only where one covers: elsewhere the
                sc_ = np.where(nr_, sc_, self.vnoise[0][sl]); sr_ = np.where(nr_, sr_, self.vnoise[1][sl])   # blended picture,
            tc = np.maximum(tc, k_ * sc_); tb = np.maximum(tb, k_ * sr_)                                      # every direction's spread
        elif getattr(self, "vnoise", None) is not None:               # the empty nights' own spread here ([H9w], --view-noise)
            k_ = self.a.view_noise; tc = np.maximum(tc, k_ * self.vnoise[0][sl]); tb = np.maximum(tb, k_ * self.vnoise[1][sl])
        changed = (dC > tc) | (bright > tb)
        dist = np.full((ph, pw), np.inf, np.float32); dist[pr, pc_] = dd
        if getattr(self.a, "logodds", False):                      # [H9w]: each frame's log-likelihood ratio, added up
            l_ = self.log_likelihood(sl, dC, bright, dist)
            region0 = V & self.Vb[sl] & ~self.band[sl]; core0 = ndi.binary_erosion(region0, iterations=self.cells(6)) & region0
            Ls = self.L[sl]; Ls[core0] = np.clip(Ls[core0] + l_[core0], self.a.lo_clamp_min, self.a.lo_clamp_max)
            self.Lsum[sl][core0] += l_[core0]
            changed = l_ > 0                                         # a frame "found it changed" where its evidence is for
        if getattr(self.a, "parallax", 0) > 0:                     # the view check ([d8d]'s other viewpoints): no baseline
            v1c, v1r, v1p = self.v1_cn[sl], self.v1_rn[sl], self.v1_cam[sl]
            have = core & np.isfinite(v1p[..., 0])
            moved = have & (np.hypot(v1p[..., 0] - C[0], v1p[..., 1] - C[1]) >= self.a.parallax_move)
            if moved.any():
                d3c, _ = tolerant_diff(cn, rn, v1c.astype(np.float32), v1r.astype(np.float32))
                d3r = np.abs(rn - v1r.astype(np.float32))
                changed |= moved & ((d3c > self.a.parallax) | (d3r > 2.5 * self.a.parallax))
            new = core & ~np.isfinite(v1p[..., 0])
            v1c[new] = cn[new]; v1r[new] = rn[new]; v1p[new] = C[:2]
        cand = ndi.binary_closing(core & changed, iterations=self.cells(2)) & core
        self.seen[sl] += core; self.hit[sl] += cand
        self.score[sl] += np.where(core, np.minimum(np.maximum(dC / tc, bright / tb), 3.0), 0.0)   # 1: at the threshold
        if getattr(self.a, "fuse", 0) > 0:                         # nearer views weigh more: a cell spans more pixels there
            dd_ = np.where(np.isfinite(dist), dist, self.a.rmax); w = np.where(core, (1.0 / np.maximum(dd_, RMIN)) ** 2, 0.0)
            self.wsum[sl] += w; self.wscore[sl] += w * np.where(core, np.minimum(np.maximum(dC / tc, bright / tb), 3.0), 0.0)
            self.smin[sl] = np.where(core, np.minimum(self.smin[sl], dd_), self.smin[sl])
            self.smax[sl] = np.where(core, np.maximum(self.smax[sl], dd_), self.smax[sl])
        bi = np.clip(((np.where(np.isfinite(dist), dist, RMIN) - RMIN) / BAND_M).astype(int), 0, NB - 1)
        for b in range(NB):
            self.seen_b[b][sl] += core & (bi == b); self.hit_b[b][sl] += cand & (bi == b)
        self.peak[sl] = np.where(cand, np.maximum(self.peak[sl], np.maximum(dC, bright)), self.peak[sl])
        self.dmin[sl] = np.where(cand, np.minimum(self.dmin[sl], dist), self.dmin[sl])
        self.dmax[sl] = np.where(cand, np.maximum(self.dmax[sl], dist), self.dmax[sl])
        closer = V & (dist < self.dbest[sl])
        self.dbest[sl] = np.where(closer, dist, self.dbest[sl]); self.rgb[sl][closer] = patch[closer]
        rp_, rv_ = self._refp; self.ref_rgb[sl][closer] = rp_[closer]; self.ref_V[sl][closer] = rv_[closer]
        return sl

    def log_likelihood(self, sl, dC, bright, dist):
        """the inverse sensor model ([H9w]; occupancy-grid form): one frame's log-likelihood ratio of "changed" against
        "unchanged" for each cell of its patch, given the cell's change. The change is measured in the cell's own noise:
        the spread of the empty nights' views of it (Views.noise, --view-noise's) combined with the frame's own (its
        colour threshold over --view-noise, so z = k at the vote's threshold on a quiet floor), the frame's part growing
        with distance (--lo-dist per m past RMIN: fewer pixels a cell, a pose error's lever) and by --lo-blend where no
        near baseline view covers the cell (a blended reference). z = max(colour, brightening) in those units; the
        ratio rises linearly from 0 at --lo-z0 by --lo-slope per unit, between --lo-min and --lo-max (the per-frame
        clamp that keeps one glint or one occlusion from deciding)"""
        a = self.a; k = max(float(getattr(a, "view_noise", 0) or 6.0), 1.0)
        tc = getattr(self, "_tc", a.tc); tc = tc if np.isscalar(tc) else np.asarray(tc, np.float32)
        g = 1.0 + a.lo_dist * np.maximum(np.where(np.isfinite(dist), dist, a.rmax) - RMIN, 0.0)
        if getattr(self, "_near", None) is not None and a.lo_blend != 1.0:
            g = g * np.where(self._near, 1.0, a.lo_blend)
        s0c, s0b = (tc / k) * g, (a.tb / k) * g
        vn = getattr(self, "vnoise", None)
        sc_ = np.sqrt(s0c ** 2 + (vn[0][sl] ** 2 if vn is not None else 0.0)); sb_ = np.sqrt(s0b ** 2 + (vn[1][sl] ** 2 if vn is not None else 0.0))
        z = np.maximum(dC / np.maximum(sc_, 1e-6), bright / np.maximum(sb_, 1e-6))
        l_ = np.clip(a.lo_slope * (z - a.lo_z0), a.lo_min, a.lo_max)
        if getattr(a, "lo_miss_dist", 0) > 0:                       # a frame that sees no change from far is weak evidence that
            dd_ = np.maximum(np.where(np.isfinite(dist), dist, a.rmax) - RMIN, 0.0)   # nothing is there: a small toy covers
            l_ = np.where(l_ < 0, l_ / (1.0 + a.lo_miss_dist * dd_), l_)            # too few pixels to show (the vote's
        return l_.astype(np.float32)                                              # "as near as the farthest hit")

    def spots(self):
        """the spots the frames so far have found changed: blobs of cells found changed at least twice, in at least
        `ratio` of the frames that saw them. The share counts only frames as near as the farthest one that found the
        change: a string too thin to show at 2 m isn't evidence against it. Yields (sl, m, area, x, y, seen_eff, group):
        group is the area of the blob's group: blobs within `group_m` of each other are one change for judging floor-sized, since a new
        patterned rug breaks into dozens of toy-sized blobs (21 stops for nothing, tuning night 70/2, 26 Sep)."""
        a = self.a
        far = np.where(self.hit_b.any(0), NB - 1 - np.argmax(self.hit_b[::-1] > 0, axis=0), 0)
        seen_eff = np.take_along_axis(np.cumsum(self.seen_b, 0), far[None], 0)[0]
        cand = (self.hit >= 2) & (self.hit >= a.ratio * seen_eff)
        if getattr(a, "fuse", 0) > 0:                              # [H9w]: the pooled change, not the count of frames that
            cand = (self.seen >= 2) & (self.wscore >= a.fuse * np.maximum(self.wsum, 1e-6))   # each found it alone
        if getattr(a, "logodds", False):                           # [H9w]: the cells whose log-odds of changed are high
            cand = self.L >= a.lo_cand
        if getattr(a, "mean_score", 0.0) > 0:                      # and changed on average, not just often: a frame's grain and
            cand &= self.score >= a.mean_score * np.maximum(self.seen, 1)   # JPEG blocks cross the threshold in 13% of cells
                                                                   # (27 Sep, realistic camera), and 4 of 6 frames by chance in 0.3%
        lab, _ = ndi.label(ndi.binary_dilation(cand, iterations=self.cells(2))); lab = lab * cand
        g = int(round(getattr(a, "group_m", 0.0) / self.res / 2))
        if g > 2:
            grp, _ = ndi.label(ndi.binary_dilation(cand, iterations=g)); gsum = np.bincount((grp * cand).ravel())
        for i, sl in enumerate(ndi.find_objects(lab), 1):
            if sl is None: continue
            m = lab[sl] == i; area = int(m.sum())
            if area < getattr(a, "min_cm2", MIN_PX) * self.k ** 2: continue   # the smallest blob, in cm2 (MIN_PX cells at 1 cm)
            group = max(area, int(gsum[grp[sl][m][0]])) if g > 2 else area
            rows, cols = np.nonzero(m); y = HY - (rows.mean() + sl[0].start + .5) * self.res; x = -HX + (cols.mean() + sl[1].start + .5) * self.res
            yield sl, m, area, x, y, seen_eff, group

    def keepout(self, margin=0.05):
        """discs (x, y, radius m) over every spot with evidence, to keep the robot's feet off: an item not yet confirmed
        is still an item (the lion was kicked on the way to another find, 25 Sep). Floor-sized changes (light, a rug)
        are not kept off."""
        return [(x, y, float(np.sqrt(area / np.pi)) * self.res + margin) for sl, m, area, x, y, _, group in self.spots() if group <= BIG_PX * self.k ** 2]

    def decide(self, t, robot_xy):
        """each spot with evidence is confirmed, dropped or (close in) marked unsure, once"""
        a = self.a; out = []
        for sl, m, area, x, y, seen_eff, group in self.spots():
            rows, cols = np.nonzero(m)
            key = (round(x / 0.15), round(y / 0.15))
            was = [k for k in self.decided if abs(key[0] - k[0]) <= 1 and abs(key[1] - k[1]) <= 1]
            if any(self.decided[k]["decision"] != "unsure" for k in was): continue
            fused = getattr(a, "fuse", 0) > 0
            if fused:                                              # [H9w]: views pooled; evidence is the views that saw it
                fm = float((self.wscore[sl][m] / np.maximum(self.wsum[sl][m], 1e-6)).max())
                if area < a.small_px * self.k ** 2 and fm < a.fuse * (1 + a.small_peak): continue
                hits = int(self.seen[sl][m].max()); seen = hits
                span = float(self.smax[sl][m].max() - self.smin[sl][m].min())
            else:
                if area < a.small_px * self.k ** 2 and float(self.peak[sl][m].max()) < a.small_peak: continue   # tiny AND faint, as detect_changes2
                hits = int(self.hit[sl][m].max()); seen = int(seen_eff[sl][m].max())
                span = float(self.dmax[sl][m].max() - self.dmin[sl][m].min())
            rng = float(np.hypot(x - robot_xy[0], y - robot_xy[1]))
            rec = {"x": round(x, 3), "y": round(y, 3), "t": round(t, 1), "range_m": round(rng, 2), "hits": hits, "seen": seen,
                   "span_m": round(span, 2), "area_cm2": area if self.k == 1 else round(area / self.k ** 2, 2)}
            lo_ok = None
            if getattr(a, "logodds", False):                       # [H9w]: confirmed on the summed evidence, over the span
                rec["logodds"] = round(float(self.L[sl][m].max()), 2); lo_ok = rec["logodds"] >= a.lo_confirm
            rec.update(self.where(sl, m))                              # other estimates of where it is: the find is aimed
            rec["x_centre"], rec["y_centre"] = rec["x"], rec["y"]      # at the cells seen from nearest (within 10 cm of the
            rec["x"], rec["y"] = rec["x_near10"], rec["y_near10"]       # nearest view): a toy's top and shadow smear a blob
                                                                       # away from it; its plain centre sat 27 cm from the
                                                                       # toy at the median, the near cells 13 (27 Sep)
            if (lo_ok if lo_ok is not None else hits >= a.hits) and span >= a.span:
                rec["kind"] = self.what(sl, m, group)                       # "gone" still gets a look: the calibration nights
                rec["decision"] = "confirmed" if rec["kind"] in ("present", "gone") else "not_a_find"   # are empty floors, and
                                                                           # the split called dark toys on dark wood gone (26 Sep)
                quiet = getattr(a, "slop_quiet", 0) > 0 and getattr(self, "vnoise", None) is not None and area <= a.slop_quiet_cells * self.k ** 2 \
                    and float(np.median(self.vnoise[0][sl][m])) <= a.slop_quiet * a.tc   # [H9w]: where the empty nights' views
                                                                       # agree, a misplaced edge has no texture to fool the check
                if rec["decision"] == "confirmed" and getattr(a, "slop", 0) > 0 and not quiet:
                    rec["slop"] = self.slop(sl, m)
                    if rec["slop"] is not None and rec["slop"] < a.slop_frac:
                        rec["decision"] = "not_a_find"; rec["why_not"] = "the floor matches a few cm off"
            elif rng < a.unsure_at and not was and group <= BIG_PX * self.k ** 2: rec["decision"] = "unsure"   # a floor change needs no look
            else: continue
            if rec["decision"] in ("confirmed", "unsure") and getattr(a, "merge_m", 0) > 0:   # [H9w]: a second blob of a find
                prior = self.behind(rec["x"], rec["y"], robot_xy)                              # (a tall toy's smear, its
                if prior is not None:                                                          # shadow) is not another stop
                    rec["decision"] = "not_a_find"; rec["why_not"] = f"part of the find at {prior['x']:.2f}, {prior['y']:.2f}"
            for k in was: rec["unsure_at_range_m"] = self.decided.pop(k)["range_m"]
            ii = np.linspace(0, len(rows) - 1, min(len(rows), 60)).astype(int)            # the blob's cells, for scoring by overlap
            rec["cells"] = [[round(-HX + (c + sl[1].start + .5) * self.res, 3 if self.k > 1 else 2), round(HY - (r + sl[0].start + .5) * self.res, 3 if self.k > 1 else 2)]
                            for r, c in zip(rows[ii], cols[ii])]
            self.decided[key] = rec; out.append(rec)
        return out

    def behind(self, x, y, robot_xy):
        """an earlier confirmed find this spot is the far side of, as the camera sees it ([H9w], --merge-m): within
        --merge-m of it, beyond it from where the robot stands now and within --merge-deg of the same line of sight
        (a toy smears and shadows away from the camera); None if there is none"""
        a = self.a; v = np.array([x - robot_xy[0], y - robot_xy[1]]); dv = np.hypot(*v)
        for r in self.decided.values():
            if r["decision"] != "confirmed": continue
            u = np.array([r["x"] - robot_xy[0], r["y"] - robot_xy[1]]); du = np.hypot(*u)
            if np.hypot(x - r["x"], y - r["y"]) > a.merge_m or dv <= du: continue
            if np.degrees(np.arccos(np.clip(u @ v / max(du * dv, 1e-9), -1, 1))) <= a.merge_deg: return r
        return None

    def slop(self, sl, m):
        """the slop check (a potential hit is checked for matching floor nearby):
        the spot's neighbourhood in tonight's closest views, slid over the reference those views were compared with, up
        to --slop cells each way, as one piece. Returns the share of the spot's cells still changed at the placement that
        matches best: a misplaced edge matches somewhere (near 0), a toy nowhere (near 1); None if too little to judge"""
        a = self.a; s_ = int(a.slop); rows, cols = np.nonzero(m); e_ = self.cells(12)
        R0, R1 = max(sl[0].start + rows.min() - s_ - e_, 0), min(sl[0].start + rows.max() + s_ + e_ + 1, self.H)
        C0, C1 = max(sl[1].start + cols.min() - s_ - e_, 0), min(sl[1].start + cols.max() + s_ + e_ + 1, self.W)
        F, Fv = self.rgb[R0:R1, C0:C1], np.isfinite(self.dbest[R0:R1, C0:C1])
        M, Mv = self.ref_rgb[R0:R1, C0:C1], self.ref_V[R0:R1, C0:C1]
        if Fv.sum() < 200 * self.k ** 2 or Mv.sum() < 200 * self.k ** 2: return None
        cf, rf = feats(F, Fv, self.fs); cm, rm = feats(M, Mv, self.fs)
        br, bc = rows + sl[0].start - R0, cols + sl[1].start - C0; ok = (br >= s_) & (br < R1 - R0 - s_) & (bc >= s_) & (bc < C1 - C0 - s_)
        br, bc = br[ok], bc[ok]
        if len(br) < 3: return None
        best = 1.0
        for dr in range(-s_, s_ + 1):
            for dc in range(-s_, s_ + 1):
                v = Mv[br + dr, bc + dc] & Fv[br, bc]
                if v.sum() < 0.6 * len(br): continue
                d = np.linalg.norm(cf[br, bc] - cm[br + dr, bc + dc], axis=-1)[v]; b = (rf[br, bc] - rm[br + dr, bc + dc])[v]
                best = min(best, float(((d > a.tc) | (b > a.tb)).mean()))
        return round(best, 3)

    def where(self, sl, m):
        """where in a blob the thing most likely is: weighted by how many frames found each cell changed (hits), by how
        strongly (peak), and the centre of the cells seen from nearest (a far view smears a blob along its line of sight)"""
        rows, cols = np.nonzero(m); R_, C_ = rows + sl[0].start, cols + sl[1].start
        xs, ys = -HX + (C_ + .5) * self.res, HY - (R_ + .5) * self.res; out = {}
        for key, w in (("hits", self.hit[R_, C_].astype(float)), ("peak", self.peak[R_, C_].astype(float) ** 2)):
            if w.sum() > 0: out[f"x_{key}"], out[f"y_{key}"] = round(float((xs * w).sum() / w.sum()), 3), round(float((ys * w).sum() / w.sum()), 3)
        dm = self.dmin[R_, C_]
        for tag, cut in (("near", 0.25), ("near10", 0.10), ("near05", 0.05), ("near50", 0.50)):
            near = dm <= np.nanmin(dm) + cut if np.isfinite(dm).any() else np.ones(len(dm), bool)
            out[f"x_{tag}"], out[f"y_{tag}"] = round(float(xs[near].mean()), 3), round(float(ys[near].mean()), 3)
        return out

    def what(self, sl, m, area):
        """The locked detector's split, on tonight's closest views so far: something here, something gone, or a floor change."""
        if area > BIG_PX * self.k ** 2: return "floor_change"
        p_ = self.cells(8); pad = tuple(slice(max(s.start - p_, 0), s.stop + p_) for s in sl)
        V = np.isfinite(self.dbest[pad]); cn, rn = feats(self.rgb[pad], V, self.fs); cb, rb = self.cb[pad], self.rb[pad]
        mm = np.zeros(V.shape, bool); mm[sl[0].start - pad[0].start: sl[0].stop - pad[0].start, sl[1].start - pad[1].start: sl[1].stop - pad[1].start] = m
        ring = ndi.binary_dilation(mm, iterations=self.cells(6)) & ~mm & V & self.Vb[pad]
        if ring.sum() < 5 * self.k ** 2: return "floor_change"
        here = np.linalg.norm(cn[mm].mean(0) - cn[ring].mean(0)) + max(0, rn[mm].mean() - rn[ring].mean())
        left = np.linalg.norm(cb[mm].mean(0) - cb[ring].mean(0)) + abs(rb[mm].mean() - rb[ring].mean())
        return "present" if here >= 0.8 * left else "gone"


class FrameCache:
    """each processed frame's comparison (the patch, where it was seen and from how far, the colour and brightening
    change, the frame's colour threshold, the reference and the near views' cover), with every decision tick's time,
    robot position and floor correction, written as it goes (--cache-out). Replayed (--cache-in), the evidence and the
    decisions run again without painting, referencing or snapping (the expensive part): what acts after the diff can
    vary. The snap's mask (cells already found changed) is the recorded run's, so a cached variant is a screen, and a
    chosen one is confirmed by a full replay"""

    PRECISION = "float32"     # [LQs]: the paint's precision, as the robot paints live (float16 caches, before 14 Oct
                              # 2026, moved stage 1's false alarms by a third: docs/research/cascade-onboard-2026-10-13.md)

    def __init__(self, path, mode="wb", options=None):
        import zlib, pickle
        self.zlib, self.pickle, self.f, self._f, self._first = zlib, pickle, open(path, mode), None, None
        if "w" in mode:                                          # a header record: what the cache holds, how precisely,
            self.precision, self.options = self.PRECISION, dict(options or {})   # and the options it was painted with
            self._write({"frame_cache": 2, "precision": self.precision, "options": self.options})
        else:                                                    # a cache without one is an old float16 cache
            first = self._read(); self.precision, self.options = "float16", None
            if isinstance(first, dict) and "frame_cache" in first: self.precision, self.options = first["precision"], first.get("options")
            else: self._first = first

    def _write(self, rec):
        z = self.zlib.compress(self.pickle.dumps(rec, protocol=5), 1); self.f.write(len(z).to_bytes(8, "little")); self.f.write(z)

    def _read(self):
        n = self.f.read(8)
        return None if len(n) < 8 else self.pickle.loads(self.zlib.decompress(self.f.read(int.from_bytes(n, "little"))))

    def add(self, sl, pr, pc_, dd, patch, V, C, dC, bright, tc, refp, near):
        dt = np.dtype(self.precision)
        fx = lambda x: None if x is None else (x if np.isscalar(x) else np.asarray(x, dt))
        self._f = ((sl[0].start, sl[0].stop, sl[1].start, sl[1].stop), np.asarray(dd, dt), patch, V, np.asarray(C, np.float32),
                   fx(np.where(np.isfinite(dC), dC, 0)), fx(np.where(np.isfinite(bright), bright, 0)), fx(tc), refp[0], refp[1], near)

    def tick(self, ts, xy, Cf, used, decide):
        self._write((float(ts), np.asarray(xy, np.float64), Cf.copy(), self._f if used else None, bool(decide))); self._f = None

    def __iter__(self):
        if self._first is not None: first, self._first = self._first, None; yield first
        while True:
            r = self._read()
            if r is None: return
            yield r

    def close(self):
        self.f.close()


CACHE_FREE = {"out", "dump", "cache_in", "cache_out", "plant", "poses_out", "place_out", "check", "check_live",
              "check_threshold", "setting"}             # options that say where results go, not what is compared


def cache_options(a):
    """the options a saved comparison is painted with ([LQs]): every option but where results go"""
    return {k: v for k, v in sorted(vars(a).items()) if k not in CACHE_FREE}


def cache_differs(fc, a):
    """[LQs]: the options a replay from a saved comparison asks for that its paint did not use. Any is a screen, not a
    measurement: the comparison holds the paint's own per-frame colour threshold (its --tc, raised by the frame's noise),
    floor locks and tracking (its snap mask, from its own decisions), so a replay's --tc or vote options do not reach
    them (14 Oct 2026: the near choice replayed from a paint at tc 0.045 judged at 0.045, not its 0.032). An old cache
    with no record: ["unknown"]"""
    if fc.options is None: return ["unknown"]
    want = cache_options(a)
    return sorted(k for k in set(want) | set(fc.options) if str(want.get(k)) != str(fc.options.get(k)))


def _judge_cached(cf, fr, plants, tc0):
    """one cached frame's comparison judged again by cf (FrameCache.add's record), with any planted changes"""
    (r0, r1, c0, c1), dd, patch, V, C, dC, bright, tc, rp, rv, near = fr
    sl = (slice(r0, r1), slice(c0, c1)); pr, pc_ = np.nonzero(V); ph, pw = V.shape
    for px_, py_, pr_, lv in plants:                                 # a planted change (positive control): every view of it
        yy, xx = np.mgrid[r0:r1, c0:c1]                              # sees its colour shifted by lv x tc
        disc = np.hypot(-HX + (xx + .5) * cf.res - px_, HY - (yy + .5) * cf.res - py_) <= pr_
        if disc.any(): dC = np.where(disc & V, np.maximum(dC.astype(np.float32), lv * tc0), dC)
    cf._tc = tc if np.isscalar(tc) else tc.astype(np.float32); cf._refp = (rp, rv); cf._near = near
    cf._judge(sl, pr, pc_, dd.astype(np.float32), patch, V, ph, pw, C, None, None, dC.astype(np.float32), bright.astype(np.float32))


def replay_cached(a, cf, cfn=None):
    """the evidence and decisions from a saved comparison (FrameCache): each frame's change judged and every tick's
    decisions made as the live replay made them; cfn: the near grid's layer, from its own cache (<cache-in>.near), tick
    for tick; returns (recs, used, per_frame, the cache's precision)"""
    recs, used, per_frame, tstart = [], 0, [], None
    fc = FrameCache(a.cache_in, "rb"); fcn = FrameCache(a.cache_in + ".near", "rb") if cfn is not None else None
    plants = [tuple(float(v) for v in p_.split(",")) for p_ in (getattr(a, "plant", None) or [])]   # x, y, radius m, change (x tc)
    for ts_, xy, Cf, fr, dec in fc:
        tstart = tstart if tstart is not None else ts_; t1 = time.time()
        if fr is not None:
            _judge_cached(cf, fr, plants, a.tc); used += 1
        cf.Cf = Cf
        if fcn is not None:
            _, _, Cfn, frn, _ = next(iter(fcn))
            if frn is not None: _judge_cached(cfn, frn, plants, cfn.a.tc)
            cfn.Cf = Cfn
        if dec:
            new = cf.decide(ts_ - tstart, xy); Ci = np.linalg.inv(cf.Cf)
            if cfn is not None: new += near_decide(cfn, cf, ts_ - tstart, xy, a)
            for r_ in new:
                r_["ts"] = round(float(ts_), 3)
                r_["x_robot"], r_["y_robot"] = (round(float(v), 3) for v in Ci[:2, :2] @ [r_["x"], r_["y"]] + Ci[:2, 2])
            recs += new
        per_frame.append(time.time() - t1)
    prec = fc.precision if fcn is None or fcn.precision == fc.precision else "mixed"
    a._cache_differs = cache_differs(fc, a)
    fc.close()
    if fcn is not None: fcn.close()
    return recs, used, per_frame, prec


def baseline(run, n, window=7, tonight=False, band_m=None, band_from=None):
    """what the robot knows before night n's sweep: the floor (median of up to `window` earlier nights' pictures), the
    structures mapped on 2+ nights (floor within 40 cm of them isn't judged), and what stood up on earlier nights (for
    lines of sight). tonight=True also counts tonight's finished map as mapped, a stand-in for the live lidar map, which
    reaches past the camera's 2.5 m (as detect_changes2); a robot sweeping live has only the earlier nights'."""
    prev = [k for k in range(max(1, n - window), n) if (run / f"mosaic_n{k}.npz").exists()]
    M = {k: np.load(run / f"mosaic_n{k}.npz") for k in prev}
    stack = np.ma.masked_array(np.stack([M[k]["rgb"] for k in prev]).astype(np.float32),
                               mask=~np.stack([np.isfinite(M[k]["dist"].astype(np.float32)) for k in prev])[..., None].repeat(3, -1))
    base = np.ma.median(stack, axis=0); Vb = ~np.ma.getmaskarray(base)[..., 0]; base = base.filled(0).astype(np.uint8)
    live = [k for k in prev + ([n] if tonight else []) if (run / f"mosaic_n{k}.npz").exists()]
    occ = [M[k]["occ"] if k in M else np.load(run / f"mosaic_n{k}.npz")["occ"] for k in live]
    static = np.sum(occ, axis=0) >= min(2, len(occ))                              # mapped on 2+ nights
    band_m = CONFIRM["band_m"] if band_m is None else band_m
    if (band_from or CONFIRM.get("band_from")) == "lidar":                  # what the lidar saw standing up on earlier nights,
        tall = np.zeros((H, W), bool)                                       # as it stands (no costmap margin): the strip is then
        for k in prev:                                                      # only for pose error
            if (run / f"tall_n{k}.npz").exists():
                h = np.load(run / f"tall_n{k}.npz")["h"].astype(np.float32)[:H, :W]
                tall[:h.shape[0], :h.shape[1]] |= ndi.binary_opening(h > 0.10, iterations=1)
        band = ndi.distance_transform_edt(~tall) * RES < band_m
    else:
        band = ndi.distance_transform_edt(~static) * RES < band_m
    hmap = np.zeros((H, W), np.float32)                       # what the lidar saw standing up on earlier nights
    for k in prev:                                            # earlier nights only: tonight's would include new tall items,
        if (run / f"tall_n{k}.npz").exists():                 # which would then hide their own footprint from the camera
            h = np.load(run / f"tall_n{k}.npz")["h"].astype(np.float32)[:H, :W]   # > 10 cm as floor_mosaic2 --hmap; lone specks
            h = np.where(ndi.binary_opening(h > 0.10, iterations=1), h, 0)          # of lidar noise on the floor aren't walls
            hmap[:h.shape[0], :h.shape[1]] = np.maximum(hmap[:h.shape[0], :h.shape[1]], h)
    return base, Vb, band, ndi.maximum_filter(hmap, size=3), prev


def parser():
    """the replay's options: the confirmer's settings (CONFIRM), the night and its files, and the frames' lag"""
    ap = _Parser(); ap.add_argument("--run-dir", required=True); ap.add_argument("--seed", type=int, default=None, help="the sim house's seed, a label only (unused: a real night has none)")
    ap.add_argument("--night", type=int, required=True); ap.add_argument("--db", required=True)
    C = CONFIRM
    ap.add_argument("--every", type=int, default=4); ap.add_argument("--rmax", type=float, default=C["rmax"])
    ap.add_argument("--window", type=int, default=7)
    ap.add_argument("--hits", type=int, default=C["hits"]); ap.add_argument("--ratio", type=float, default=C["ratio"])
    ap.add_argument("--span", type=float, default=C["span"]); ap.add_argument("--unsure-at", type=float, default=C["unsure_at"])
    ap.add_argument("--small-px", type=int, default=C["small_px"]); ap.add_argument("--small-peak", type=float, default=C["small_peak"])
    ap.add_argument("--tb", type=float, default=C["tb"], help="brightening that counts as a change (the detector's own is TB)")
    ap.add_argument("--tc", type=float, default=C["tc"], help="colour shift that counts as a change (the detector's own is TC)")
    ap.add_argument("--parallax", type=float, default=C["parallax"], help="colour disagreement between two views of a cell (0: off)")
    ap.add_argument("--parallax-move", type=float, default=C["parallax_move"], help="how far apart the two views' cameras must be (m)")
    ap.add_argument("--group-m", type=float, default=C["group_m"], help="blobs this close are one change when judging floor-sized (m; 0: off)")
    ap.add_argument("--band-m", type=float, default=C["band_m"], help="floor this close to mapped structures isn't judged (m)")
    ap.add_argument("--band-from", default=C.get("band_from", "costmap"), choices=["costmap", "lidar"], help="structures: the costmap's (inflated) or the lidar's own")
    ap.add_argument("--decide-every", type=float, default=1.0, help="seconds of robot time between decisions")
    ap.add_argument("--camera", default="head", choices=["head", "wrist", "both"], help="the head camera, the stowed arm's wrist camera, or both into one evidence map")
    ap.add_argument("--mount", default=None, help="the head camera's calibrated mount (.json, calibrate_mount.py): tonight's frames placed with it, as the views were")
    ap.add_argument("--relief", default="auto", help="the floor's relief (.npy on the floor grid, floor_selfcal.py relief): tonight's frames laid on the floor where it stands, as the views were. 'auto' (default): the house's own (relief.npy in --run-dir), else the plane z = 0, said; 'none': the plane; or a path")
    ap.add_argument("--stale-map", action="store_true", help="compare even with a house map made before its loops were verified (analysis only; refused by default, [dmT])")
    ap.add_argument("--mixed-calibration", action="store_true", help="compare even with baselines painted under another calibration (analysis only; refused by default, [Q9k])")
    ap.add_argument("--views", nargs="*", default=None, help="the baseline nights' own views (views_n<k>.npz, baseline_views.py): each frame compared with the baseline as seen from about there")
    ap.add_argument("--view-noise", type=float, default=C.get("view_noise", 0.0), help="raise each cell's change thresholds to this many times how much the empty nights' own views of it vary (Views.noise; 0: off) [H9w]")
    ap.add_argument("--dir-noise", type=int, default=C.get("dir_noise", 0), help="with --view-noise: keep the empty nights' spread per direction, in this many azimuth bins of the camera as seen from each cell, and read each frame's from its own direction (dirfloor.DirFloor; 0: off, every direction at once) [4t7]")
    ap.add_argument("--dir-min-views", type=int, default=3, help="--dir-noise: a direction bin speaks for a cell from this many views; fewer: its neighbours pooled, then every direction")
    ap.add_argument("--dir-prior", type=float, default=0.0, help="--dir-noise: each direction's variance shrunk toward every direction's as if that had lent it this many views (0: off; the own / neighbours / every-direction fallback instead)")
    ap.add_argument("--dir-ref", action=argparse.BooleanOptionalAction, default=False, help="--dir-noise: where no near baseline view covers a cell, compare with its mean from this frame's direction (dirfloor.DirFloor.mean_rgb) instead of the blended picture [4t7]")
    ap.add_argument("--dir-near-only", action=argparse.BooleanOptionalAction, default=False, help="--dir-noise: a direction's spread only where the frame was compared with a near baseline view (seen from about here); where the reference is the blended picture, every direction's")
    ap.add_argument("--dir-floor", type=float, default=0.0, help="--dir-noise: a direction's spread never below this share of every direction's (0: off)")
    ap.add_argument("--dark", type=float, default=C.get("dark", 0.0), help="where the mapped floor's grey (0-255) is under this, a colour shift doesn't count, only a brightening (0: off) [H9w]")
    ap.add_argument("--fuse", type=float, default=C.get("fuse", 0.0), help="pool each cell's change over every view that saw it (weighted by nearness) and call it changed where the pooled score, 1 at the threshold, reaches this; the evidence a spot needs is then views (--hits) over --span m (0: off, the frame-by-frame vote) [H9w]")
    ap.add_argument("--logodds", action=argparse.BooleanOptionalAction, default=C.get("logodds", False), help="decide on each cell's log-odds of changed, summed over the frames that saw it (the binary Bayes filter of occupancy grids), instead of the frames' vote (--hits, --ratio) [H9w]")
    ap.add_argument("--lo-z0", type=float, default=4.0, help="log-odds: the change (in the cell's noise) at which a frame's evidence is neutral")
    ap.add_argument("--lo-slope", type=float, default=0.5, help="log-odds: a frame's evidence per unit of change past --lo-z0")
    ap.add_argument("--lo-min", type=float, default=-0.4, help="log-odds: one frame's strongest evidence against")
    ap.add_argument("--lo-max", type=float, default=1.0, help="log-odds: one frame's strongest evidence for")
    ap.add_argument("--lo-clamp-min", type=float, default=-2.0, help="log-odds: the sum's floor (how far a cell can be believed unchanged)")
    ap.add_argument("--lo-clamp-max", type=float, default=4.0, help="log-odds: the sum's ceiling")
    ap.add_argument("--lo-cand", type=float, default=1.0, help="log-odds: a cell is part of a spot from this sum")
    ap.add_argument("--lo-confirm", type=float, default=2.5, help="log-odds: a spot is confirmed when its strongest cell reaches this")
    ap.add_argument("--lo-dist", type=float, default=0.5, help="log-odds: the frame's noise grows by this share per m past 0.5 m")
    ap.add_argument("--lo-miss-dist", type=float, default=0.0, help="log-odds: evidence against change shrinks by 1 / (1 + this x m past 0.5 m): a far view misses a small toy (0: off)")
    ap.add_argument("--lo-blend", type=float, default=1.5, help="log-odds: the frame's noise times this where no near baseline view covers the cell")
    ap.add_argument("--slop-quiet", type=float, default=0.0, help="skip the slop check for a spot of at most --slop-quiet-cells cells whose empty nights' views agree: their colour spread (median) at most this times tc (0: off) [H9w]")
    ap.add_argument("--slop-quiet-cells", type=int, default=60)
    ap.add_argument("--merge-m", type=float, default=0.0, help="a spot within this of an earlier confirmed find and beyond it on about the same line of sight is part of it, not a find (0: off) [H9w]")
    ap.add_argument("--merge-deg", type=float, default=25.0)
    ap.add_argument("--min-cm2", type=float, default=float(MIN_PX), help="the smallest blob a spot can be (cm2 of floor; MIN_PX cells at 1 cm)")
    ap.add_argument("--near-res", type=float, default=0.0, help="the near floor compared again on a finer grid (m a cell, dividing 1 cm: 0.005), a second layer beside the 1 cm one: frames placed as the 1 cm layer places them (its floor lock), compared with --near-views (0: off) [H9w]")
    ap.add_argument("--near-rmax", type=float, default=1.0, help="--near-res: the near layer's reach (m)")
    ap.add_argument("--near-views", nargs="*", default=None, help="--near-res: the baseline nights' views painted on that grid (baseline_views.py --res)")
    ap.add_argument("--near-opt", action="append", default=None, help="--near-res: the near layer's own setting, key=value over this replay's (e.g. view_noise=8, span=0.3; sizes in cells are its grid's)")
    ap.add_argument("--near-merge-m", type=float, default=0.30, help="--near-res: a near find this close to a 1 cm find (confirmed or unsure) is the same find")
    ap.add_argument("--near-max-cm2", type=float, default=0.0, help="--near-res: a near find larger than this is left to the 1 cm layer (0: no limit)")
    ap.add_argument("--cache-out", default=None, help="save each frame's comparison (after the paint, reference and diff) for --cache-in")
    ap.add_argument("--plant", action="append", default=None, help="with --cache-in: a planted change x,y,radius_m,level (level x tc in every view of it), a positive control")
    ap.add_argument("--cache-in", default=None, help="replay the evidence and decisions from a saved comparison (--cache-out), skipping the paint: any option that acts after the diff (the vote, view noise's thresholds, the log-odds, the slop check) can vary")
    ap.add_argument("--slop", type=int, default=C["slop"], help="the slop check: slide a spot's neighbourhood this many cells each way over the reference; 0: off")
    ap.add_argument("--slop-frac", type=float, default=C["slop_frac"], help="a spot whose cells still differ under this share at the best placement is floor, not a find")
    ap.add_argument("--noise-k", type=float, default=C["noise_k"], help="raise the colour threshold where the frame's own noise over the floor's brightness exceeds it, by this factor (0: off)")
    ap.add_argument("--track", action=argparse.BooleanOptionalAction, default=C["track"], help="carry each lock's offset to the next frames' poses (the floor as the localizer, [fXD]); needs --snap")
    ap.add_argument("--track-gain", type=float, default=C["track_gain"]); ap.add_argument("--track-rot-sign", type=float, default=C["track_rot_sign"])
    ap.add_argument("--snap-reach", type=int, default=8, help="weighted snap: how far it searches (cells, cm)")
    ap.add_argument("--snap-tau", type=float, default=0.05, help="weighted snap: the belief's softness (NCC units): smaller, sharper")
    ap.add_argument("--snap-mode", default=C.get("snap_mode", "lock"), choices=["lock", "weighted"], help="lock: a snap only when clearly best; weighted: every frame's belief, filtered along the path")
    ap.add_argument("--snap", action=argparse.BooleanOptionalAction, default=C["snap"], help="lay the baseline on each frame's own floor where its edges lock, before comparing ([KkQ])")
    ap.add_argument("--slack", type=int, default=C["slack"], help="cells of misalignment between tonight and the baseline the comparison forgives")
    ap.add_argument("--mean-score", type=float, default=C["mean_score"], help="a changed cell's change, averaged over every frame that saw it, relative to the threshold (0: off)")
    ap.add_argument("--smooth", type=float, default=C["smooth"], help="average both floors over this many cells (sd) before comparing (0: off)")
    ap.add_argument("--dt", default="robot", help="the frames' stamp lag ([uKM] 2, [gJq]): 'robot' (default; 'auto' the same): the robot's stored lag for each camera (--lag-file), tonight's own estimate from its turning only checked against it; 'night': tonight's estimate stands; or a number: that lag (s), each frame placed where the camera was that much earlier")
    ap.add_argument("--lag-file", default=str(LAG_FILE), help="the robot's frame lag (.json from calibrate_lag.py)")
    ap.add_argument("--no-lag-check", action="store_true", help="--dt robot: skip tonight's estimate (the check)")
    add_clock_args(ap)
    ap.add_argument("--corrections", default=None, help="tonight's pose corrections (.npz: times, Ts 3x3), from the localizer ([KkQ])")
    ap.add_argument("--place", default=None, help="place tonight's frames live by the house's pose graph ([x9X]; live_place.py): the house's keyframes (.npz, live_place.py build), tonight's lidar and odometry and the confirmer's own floor locks added as the night runs; needs --no-track (the locks are constraints, not a gain)")
    ap.add_argument("--place-floor", action=argparse.BooleanOptionalAction, default=True, help="--place: the floor locks in the graph (--no-place-floor: lidar and odometry alone)")
    ap.add_argument("--place-out", default=None, help="--place: the graph's record (.npz: its nodes, constraints and compute time per sweep and lock)")
    ap.add_argument("--poses-out", default=None, help="each judged frame's time and the correction it was placed with (.npz: times, Ts 3x3 house <- odometry)")
    ap.add_argument("--setting", default=None, choices=sorted(SETTINGS), help="a named setting under the options given: 'launch', [tTD]'s near-floor grid and the light check (needs --near-views); default: CONFIRM")
    ap.add_argument("--check", action=argparse.BooleanOptionalAction, default=False, help="the light check on each candidate after the night's decisions (houndkeeper.candidate_check: YOLO-E on crops from the frames nearest in time; GPU if there is one, else CPU; off by default) [Zaf]")
    ap.add_argument("--check-live", action="store_true", help="--check: as the live tidy runs it, frames kept candidate_check.KEEP_S and a candidate scored after WAIT_S with the views it has (default: every frame of the night, as the bench)")
    ap.add_argument("--check-threshold", type=float, default=None, help="--check: keep a candidate at this score or above (default: the chosen threshold)")
    ap.add_argument("--out", default=None); ap.add_argument("--dump", default=None, help="save the evidence arrays (.npz) for inspection")
    return ap


def options(setting=None):
    """[BW3]: the replay's options as its parser defaults them, a named setting over them (as --setting lays it): the
    live tidy's, so the robot judges with what the bench measured. The night's own files (--run-dir, --night, --db,
    --views, --near-views) are left for the caller"""
    d = {x.dest: x.default for x in parser()._actions if x.dest != "help"}
    if setting: d.update(SETTINGS[setting]); d["setting"] = setting
    return argparse.Namespace(**d)


def use_views(a, cf, paths):
    """the baseline nights' own views a Confirmer compares with (--views), and what they give it: each cell's view
    noise (--view-noise, [H9w]) and its spread by direction (--dir-noise). The replay and the live tidy both ([BW3])"""
    if paths: cf.views = Views.load(paths)
    if getattr(a, "view_noise", 0) > 0 and getattr(cf, "views", None) is not None: cf.vnoise = cf.views.noise(getattr(a, "smooth", 0.0))
    dir_model(a, cf)


def dir_model(a, cf):
    """--dir-noise: the baseline views' spread kept per direction (dirfloor.DirFloor, [4t7]); view noise then reads the
    spread from the direction each frame sees a cell, not from every direction at once"""
    if getattr(a, "dir_noise", 0) > 0 and getattr(a, "view_noise", 0) > 0 and getattr(cf, "views", None) is not None:
        import dirfloor
        cf.dirmodel = dirfloor.DirFloor.build(cf.views, int(a.dir_noise), getattr(a, "smooth", 0.0), log=lambda m: print(m, flush=True))


def calibration_used(a, run, prev):
    """the floor relief tonight is laid on (the house's own by default, [yFg]), after checking that every baseline
    picture and view set was painted with the calibration tonight's frames are ([Q9k]): one painted with another
    mount, lag or relief is refused (a SystemExit naming what differs) unless --mixed-calibration; a stitched house's
    map made before its loops were verified likewise, unless --stale-map ([dmT]). Returns (relief or None, its path or
    None)"""
    from houndkeeper import floor_relief as _frl, calibrate as _cal
    relief, rpath, note = _frl.resolve(getattr(a, "relief", "auto"), run)
    print(f"floor: {note}", flush=True)
    robot_lag = str(a.dt) in ("robot", "auto")
    want = _cal.stamp(a.mount, a.lag_file if robot_lag else None, rpath)
    names = [f"mosaic_n{k}.npz" for k in prev]
    why = _cal.stale_products(run, want, names) + [r for v in (a.views or []) + (getattr(a, "near_views", None) or [])
                                                   for r in _cal.stale(_cal.stamp_of(v), want, Path(v).name)]
    if why and not getattr(a, "mixed_calibration", False):
        raise SystemExit("the baselines were painted under another calibration than tonight's frames ([Q9k]); make the house's "
                         "floor map again (stitch_baselines.sh): " + "; ".join(why))
    if why: print("warning: compared across calibrations, as asked: " + "; ".join(why), flush=True)
    old = _cal.stale_map(run)                                  # a house's map made before its loops were verified ([dmT])
    if old and not getattr(a, "stale_map", False):
        raise SystemExit("the house's map is older than its loop closures' check ([dmT]); make it again "
                         "(stitch_baselines.sh): " + "; ".join(old))
    if old: print("warning: compared with an older house map, as asked: " + "; ".join(old), flush=True)
    return relief, rpath


def near_layer(a, base, Vb, band, hmap, K, wh, relief=None):
    """[H9w]: the near floor (out to --near-rmax) compared on a finer grid (--near-res), a second Confirmer beside the
    1 cm one: the 1 cm baselines repeated onto it, the baseline nights' views painted on it (--near-views), its own
    settings (--near-opt) over this replay's. It does not snap: each frame is placed as the 1 cm layer placed it (that
    layer's floor lock, copied in before each frame). None when off"""
    if not getattr(a, "near_res", 0): return None
    k = int(round(RES / a.near_res)); grid(a.near_res)
    an = argparse.Namespace(**vars(a)); an.rmax, an.snap, an.track = a.near_rmax, False, True
    an.cache_out = an.cache_in = None; an.dir_noise = 0
    for kv in a.near_opt or []:
        key, v = kv.split("=", 1); key = key.replace("-", "_"); cur = getattr(an, key)
        setattr(an, key, (v.lower() in ("1", "true", "yes")) if isinstance(cur, bool) else type(cur)(v) if cur is not None else v)
    cf = Confirmer(upsample(base, k), upsample(Vb, k), upsample(band, k), upsample(hmap, k), K, wh, an, res=a.near_res)
    if relief is not None: cf.relief = upsample(relief, k)
    if a.near_views:
        cf.views = Views.load(a.near_views)
        if abs(cf.views.res - a.near_res) > 1e-9: raise SystemExit(f"--near-views are painted at {cf.views.res} m a cell, not {a.near_res}")
    if getattr(an, "view_noise", 0) > 0 and getattr(cf, "views", None) is not None: cf.vnoise = cf.views.noise(getattr(an, "smooth", 0.0))
    return cf


def near_decide(cfn, cf, t, xy, a):
    """the near layer's decisions this tick, each marked layer "near": a find within --near-merge-m of the 1 cm layer's
    (confirmed or unsure) is that find, and one larger than --near-max-cm2 is left to the 1 cm layer (not_a_find, why)"""
    out = []
    for r in cfn.decide(t, xy):
        r["layer"] = "near"
        if r["decision"] in ("confirmed", "unsure"):
            if getattr(a, "near_max_cm2", 0) > 0 and r["area_cm2"] > a.near_max_cm2:
                r["decision"] = "not_a_find"; r["why_not"] = "larger than the near grid keeps: the 1 cm grid's"
            elif any(d["decision"] in ("confirmed", "unsure") and np.hypot(d["x"] - r["x"], d["y"] - r["y"]) <= a.near_merge_m
                     for d in cf.decided.values()):
                r["decision"] = "not_a_find"; r["why_not"] = "the 1 cm grid's find"
        out.append(r)
    return out


def near_frame(cfn, cf, img, T, K=None, wh=None, mask=None):
    """[H9w]: one frame on the near grid, placed as the 1 cm layer placed it (its floor lock copied in first)"""
    cfn.Cf = cf.Cf.copy(); return cfn.frame(img, T, K, wh, mask)


class Layers:
    """[BW3]: the 1 cm Confirmer and its near layer judged as one, as replay() judges them: each frame to both (the near
    layer placed by the 1 cm one's lock), decisions the 1 cm layer's then the near layer's (near_decide), spots kept out
    from both. Anything else is the 1 cm layer's; tonight's tall layer set on it reaches the near layer on its grid"""

    def __init__(self, cf, cfn, a):
        object.__setattr__(self, "cf", cf); object.__setattr__(self, "cfn", cfn); object.__setattr__(self, "a", a)

    def __getattr__(self, name): return getattr(self.cf, name)

    def __setattr__(self, name, value):
        setattr(self.cf, name, value)
        if name == "hmap": self.cfn.hmap = upsample(value, self.cfn.k)

    def frame(self, img, T, K=None, wh=None, mask=None):
        out = self.cf.frame(img, T, K, wh, mask); near_frame(self.cfn, self.cf, img, T, K, wh, mask); return out

    def decide(self, t, robot_xy):
        return self.cf.decide(t, robot_xy) + near_decide(self.cfn, self.cf, t, robot_xy, self.a)

    def keepout(self, *a, **kw): return self.cf.keepout(*a, **kw) + self.cfn.keepout(*a, **kw)


def replay(a, pose_hook=None, scan_hook=None):
    """the confirmer run over a recorded sweep as the robot would have run it, from what it knew then: its decisions
    (recs, each with where it was and when), the Confirmer, the frames used and each one's time. pose_hook: wraps the
    pose lookup (t -> world<-base) before the corrections; scan_hook (t, points) -> points: the lidar sweeps --place
    reads, moved as pose_hook moves the poses; the robot never sets either (an offline ablation's)"""
    t0 = time.time(); run = Path(a.run_dir); n = a.night
    base, Vb, band, hmap, prev = baseline(run, n, a.window, tonight=True, band_m=a.band_m, band_from=a.band_from)
    relief, rpath = calibration_used(a, run, prev)
    if getattr(a, "cache_in", None):                             # the saved comparison: no frames, no poses ([H9w] screens)
        cf = Confirmer(base, Vb, band, hmap, (1.0, 1.0, 0.0, 0.0), (1, 1), a)
        use_views(a, cf, a.views)
        cfn = near_layer(a, base, Vb, band, hmap, (1.0, 1.0, 0.0, 0.0), (1, 1))
        if cfn is not None and not Path(a.cache_in + ".near").exists(): raise SystemExit(f"--near-res: no {a.cache_in}.near")
        recs, used, per_frame, prec = replay_cached(a, cf, cfn)
        if getattr(a, "check", False): check_stream(a, recs)    # [Zaf]: the light check on each candidate
        return dict(recs=recs, cf=cf, cfn=cfn, used=used, per_frame=per_frame, prev=prev, lags={}, t0=t0, precision=prec,
                    cache_differs=getattr(a, "_cache_differs", ["unknown"]))
    st = SqliteStore(path=a.db); st.start()
    ci = next(iter(st.stream("camera_info"))).data
    pose, T_bo = poses(st)
    streams = ["color_image"] * (a.camera != "wrist") + ["arm_color_image"] * (a.camera in ("wrist", "both"))
    clock = clock_of(a.db, getattr(a, "clock", "auto"), getattr(a, "traj", None)) if str(a.dt) in ("robot", "auto") else None
    lags = frame_lags(st, a.dt, streams, a.lag_file, check=not a.no_lag_check, clock=clock)   # each camera its own ([gJq]),
                                                                 # on the recording's clock ([55G])
    for s_, (lag_, note) in lags.items(): print(f"{s_}: frame lag {lag_:.3f} s, {note}", flush=True)
    if pose_hook: pose = pose_hook(pose)
    loc = None; placer = pfeed = None
    if getattr(a, "place", None):                                # [x9X]: the house's graph places tonight, live
        if a.track: raise SystemExit("--place takes the floor locks into the graph: give --no-track as well")
        if a.corrections: raise SystemExit("--place and --corrections both place the frames: one or the other")
        import live_place as lp
        placer = lp.Placer(lp.HouseKeyframes.load(a.place), floor=getattr(a, "place_floor", True))
        pfeed = lp.Feed(st, placer, pose, scan_hook); odom0 = pose; ctx = {}   # the frame being judged, for its lock
    if a.corrections:                                            # the robot's belief moved into the house's frame ([KkQ])
        from houndkeeper.localize import Localizer
        z_ = np.load(a.corrections); loc = Localizer(times=list(z_["times"]), Ts=list(z_["Ts"]), fits=[(0.0, 0)] * len(z_["times"]))
        _pose0 = pose; pose = lambda t: (lambda M: None if M is None else loc.apply(M, t))(_pose0(t))
    from houndkeeper.lens import Rectifier
    rect = Rectifier.from_camera_info(ci)                        # the head camera's lens, straightened ([kJ6])
    cf = Confirmer(base, Vb, band, hmap, rect.K, (ci.width, ci.height), a)
    if placer is not None and getattr(a, "place_floor", True):  # each lock of this frame's floor, to the graph
        cf.on_lock = lambda S, at, sc, ld: pfeed.lock_timed(ctx["t"], ctx["O"], S, at, ctx["C"])
    cf.relief = relief                                           # tonight's floor laid where the views' was ([yFg])
    use_views(a, cf, a.views)                                    # [H9w]
    cfn = near_layer(a, base, Vb, band, hmap, rect.K, (ci.width, ci.height), relief)   # [H9w]: the near grid, or None
    if getattr(a, "cache_out", None):
        cf.cache = FrameCache(a.cache_out, options=cache_options(a))
        if cfn is not None: cfn.cache = FrameCache(a.cache_out + ".near", options=cache_options(a))
    recs, used, last, tstart, per_frame, per_pose = [], 0, -1e9, None, [], []
    if a.mount:                                                  # the camera truly sits a little turned ([KkQ])
        import math as _m; from houndkeeper import calibrate as _cal
        Rm, off_m = _cal.mount(json.loads(Path(a.mount).read_text())); T_bo = T_bo.copy()   # its turn, and its place (body frame)
        T_bo[:3, :3] = T_bo[:3, :3] @ Rm; T_bo[:3, 3] = T_bo[:3, 3] + off_m
    cams = {"head": [("color_image", T_bo, None, None, None, rect)]}
    if a.camera in ("wrist", "both"):                         # the stowed arm's camera, fixed on the trunk while walking
        import floor_mosaic2 as fm
        wi = next(iter(st.stream("arm_color_image"))).data; wimg = arr(wi); wh = (wimg.shape[1], wimg.shape[0])
        cams["wrist"] = [("arm_color_image", fm.wrist_extrinsic(*wh), fm.wrist_intrinsics(*wh), wh, fm.wrist_robot_mask(*wh), None)]
    feeds = cams["head"] * (a.camera != "wrist") + cams.get("wrist", [])
    import heapq
    def feed(f): return ((o.ts, i, o.data, f) for k, o in enumerate(st.stream(f[0])) if k % a.every == 0 for i in [feeds.index(f)])
    for ts_, _, data, (stream, T_bx, Kx, whx, maskx, lens_x) in heapq.merge(*(feed(f) for f in feeds), key=lambda x: (x[0], x[1])):   # in time order
        lag = lags[stream][0]                                    # this camera's own lag
        if placer is not None:                                   # every sweep up to this frame's arrival, then the frame
            pfeed.until(ts_); Cp = placer.correction(); O_ = pfeed.odom(ts_ - lag)   # placed from the newest estimate
            if O_ is None: continue
            M_ = np.eye(4); M_[:2, :2] = Cp[:2, :2]; M_[:2, 3] = Cp[:2, 2]; Twb = M_ @ odom0(ts_ - lag)
            ctx.update(t=ts_ - lag, O=O_, C=Cp)
        else:
            Twb = pose(ts_ - lag)
        if Twb is None: continue
        if getattr(a, "poses_out", None) and stream == "color_image":   # the correction this frame is placed with
            Cu = cf.Cf @ (Cp if placer is not None else loc.correction(ts_ - lag) if loc is not None else np.eye(3))
            per_pose.append((ts_ - lag, Cu))
        tstart = tstart or ts_; t1 = time.time()
        if lens_x is not None: data = lens_x(arr(data))
        ok_ = cf.frame(data, Twb @ T_bx, Kx, whx, maskx) is not None; used += ok_
        okn = False
        if cfn is not None and stream == "color_image":           # the near grid: placed as the 1 cm layer placed it
            okn = near_frame(cfn, cf, data, Twb @ T_bx, Kx, whx, maskx) is not None
        if getattr(cf, "cache", None) is not None: cf.cache.tick(ts_, Twb[:2, 3], cf.Cf, ok_, ts_ - last >= a.decide_every)
        if cfn is not None and getattr(cfn, "cache", None) is not None: cfn.cache.tick(ts_, Twb[:2, 3], cfn.Cf, okn, ts_ - last >= a.decide_every)
        if ts_ - last >= a.decide_every:
            new = cf.decide(ts_ - tstart, Twb[:2, 3]); last = ts_
            if cfn is not None: new += near_decide(cfn, cf, ts_ - tstart, Twb[:2, 3], a)
            Cr = cf.Cf @ (placer.correction() if placer is not None else loc.correction(ts_ - lag) if a.corrections else np.eye(3))   # the Confirmer's frame <- odometry's, now
            Ci = np.linalg.inv(Cr)
            for r_ in new:                                           # where the robot would walk to: odometry's frame, at this
                r_["ts"] = round(float(ts_), 3)                      # moment (the sim's truth then says where that really is)
                r_["x_robot"], r_["y_robot"] = (round(float(v), 3) for v in Ci[:2, :2] @ [r_["x"], r_["y"]] + Ci[:2, 2])
            recs += new
        per_frame.append(time.time() - t1)
    st.stop()
    if getattr(cf, "cache", None) is not None: cf.cache.close(); cf.cache = None
    if cfn is not None and getattr(cfn, "cache", None) is not None: cfn.cache.close(); cfn.cache = None
    if a.dump: np.savez_compressed(a.dump, seen_b=cf.seen_b, hit_b=cf.hit_b, rgb=cf.rgb)
    if getattr(a, "poses_out", None):
        np.savez(a.poses_out, times=np.array([p_[0] for p_ in per_pose]), Ts=np.array([p_[1] for p_ in per_pose]).reshape(-1, 3, 3))
    if placer is not None and getattr(a, "place_out", None):
        fs = placer.factors
        np.savez(a.place_out, node_t=np.array([n_[0] for n_ in placer.nodes]), node_kind=np.array([n_[2] for n_ in placer.nodes]),
                 factor_kind=np.array([f_[0] for f_ in fs]), factor_t=np.array([f_[1] for f_ in fs], float),
                 factor_Z=np.array([f_[2] if f_[2] is not None else (np.nan,) * 3 for f_ in fs], float).reshape(-1, 3),
                 factor_ok=np.array([f_[3] for f_ in fs], bool), factor_why=np.array([f_[4] for f_ in fs]),
                 sec_t=np.array([x_[0] for x_ in pfeed.seconds], float), sec_kind=np.array([str(x_[1]) for x_ in pfeed.seconds]),
                 sec_s=np.array([x_[2] for x_ in pfeed.seconds], float))
    if getattr(a, "check", False): check_stream(a, recs)        # [Zaf]: the light check on each candidate
    return dict(recs=recs, cf=cf, cfn=cfn, used=used, per_frame=per_frame, prev=prev, lags=lags, t0=t0, precision="float32")


def check_stream(a, recs, recognizer=None):
    """[Zaf]: the light check (houndkeeper.candidate_check) run over the night as the robot runs it live: the head
    camera's frames (every candidate_check.EVERY-th, placed as the confirmer places them: odometry at the frame's time
    less the camera's lag, the calibrated mount) and the decisions in time order, a decision after the frame it was made
    on; each candidate's crops cut from the robot's own record of its frames (the recording) when chosen. Each candidate
    decision gets its verdict (rec["check"]: score, kept, the views' frame stamps). Reads no truth"""
    from houndkeeper import candidate_check as cc
    t0 = time.time(); st = SqliteStore(path=a.db); st.start()
    ci = next(iter(st.stream("camera_info"))).data; pose, T_bo = poses(st)
    if a.mount:
        from houndkeeper import calibrate as _cal
        Rm, off_m = _cal.mount(json.loads(Path(a.mount).read_text())); T_bo = T_bo.copy()
        T_bo[:3, :3] = T_bo[:3, :3] @ Rm; T_bo[:3, 3] = T_bo[:3, 3] + off_m
    clock = clock_of(a.db, getattr(a, "clock", "auto"), getattr(a, "traj", None)) if str(a.dt) in ("robot", "auto") else None
    lag = frame_lags(st, a.dt, ["color_image"], a.lag_file, check=False, clock=clock)["color_image"][0]
    def image(ts):
        r = list(st.stream("color_image").time_range(ts - 1e-4, ts + 1e-4))
        return arr(r[0].data) if r else None
    chk = cc.CandidateCheck(np.array(ci.K, float).reshape(3, 3), list(ci.D or []), ci.distortion_model or "", (ci.width, ci.height),
                            recognizer=recognizer, threshold=getattr(a, "check_threshold", None) or cc.THRESHOLD, images=image,
                            **(dict(keep_s=cc.KEEP_S, max_wait_s=cc.WAIT_S) if getattr(a, "check_live", False) else dict(keep_s=None)))
    order = sorted((r["ts"], j) for j, r in enumerate(recs) if cc.is_candidate(r)); q = 0
    for k, o in enumerate(st.stream("color_image")):
        while q < len(order) and order[q][0] < o.ts:            # decisions made on earlier frames
            chk.candidate(recs[order[q][1]], ("x_robot", "y_robot")); q += 1
        if k % cc.EVERY: continue
        P = pose(o.ts - lag)
        if P is None: continue
        chk.frame(float(o.ts), P @ T_bo, None, k)               # its image read back only if a candidate chooses it
    for _, j in order[q:]: chk.candidate(recs[j], ("x_robot", "y_robot"))
    chk.finish(); st.stop()
    cc.annotate(recs, chk.verdicts, ("x_robot", "y_robot"))
    kept = sum(v["kept"] for v in chk.verdicts.values())
    print(f"check: {len(chk.verdicts)} candidates, {kept} kept at {chk.threshold}; {time.time() - t0:.0f} s on {getattr(chk.rec, 'device', '?')}", flush=True)
    return chk


def summary(r):
    """the replay's own account: how the floor locked, the decisions, the floor judged, the frames (no score)"""
    cf, recs = r["cf"], r["recs"]
    if cf.snaps:                                                 # how often the floor locked, and how far off the pose was
        L = [x for x in cf.snaps if x and x[5]]
        print(f"snap: {len(L)}/{len(cf.snaps)} frames locked; shift median {np.median([np.hypot(x[1], x[2]) for x in L]) * 10 if L else 0:.0f} mm, "
              f"turn median {np.median([abs(x[0]) for x in L]) if L else 0:.2f} deg", flush=True)
    return {d: sum(x["decision"] == d for x in recs) for d in ("confirmed", "not_a_find", "unsure")}


def main(argv=None):
    """the replay alone (the robot's side): its decisions, unscored; score_confirm.py scores them against the sim"""
    a = parser().parse_args(argv); r = replay(a); n = a.night; cf = r["cf"]; dec = summary(r)
    print(f"night {n} (baseline {r['prev']}): decisions {dec}; floor judged {(cf.seen > 0).sum() * RES * RES:.0f} m2, "
          f"found changed twice {(cf.hit >= 2).sum() * RES * RES:.1f} m2; frames {r['used']}, "
          f"{np.median(r['per_frame'])*1000:.0f} ms each (median); {time.time()-r['t0']:.0f}s")
    if a.out:
        Path(a.out).write_text(json.dumps({"night": n, "decisions": r["recs"], "snaps": cf.snaps, "args": vars(a)}, indent=1))


if __name__ == "__main__":
    main()
