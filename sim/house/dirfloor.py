#!/usr/bin/env python3
"""The floor's appearance by the direction it is seen from ([4t7]).
One model, built from the baseline nights' own views of the floor (baseline_views.py,
confirm_stream.Views), serves both the detection cascade's first stage ([H9w]) and the pose graph's front end ([JzW]):

  for every floor cell (the 1 cm grid) and every direction bin (the azimuth of the camera as seen from the cell, NB
  bins round the circle; optionally also the range from the camera, which on a flat floor is the elevation), the
  number of views, the mean and spread (sd) of the confirmer's own features (detect_changes2.feats: colour against the
  surrounding floor, brightness against its level) and the mean grey (what floor registration matches on).

  spread(): a cell's spread from the direction a frame sees it, falling back to the neighbouring bins pooled, then to
            every direction (Views.noise's figure) where a direction was seen too few times;
  decompose(): how much of each cell's variation between views is direction (between bins) and how much is not
            (within a bin): the measurement that justifies the bins;
  bias():   how far the floor seen from one direction sits from the floor seen from another (the bins' mean grey
            registered on each other, house-wide): the directional displacement a cross-direction floor match carries.

Nothing here reads the sim's floor plan ([1B2]). (dimos venv python)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import math, time

import numpy as np

from detect_changes2 import feats, RES, HX, HY

W, H = int(2 * HX / RES), int(2 * HY / RES)
NB = 8                                  # 45 deg bins: chosen from decompose() (direction's share stops growing past 8)
MIN_VIEWS = 3                           # a bin speaks for a cell from this many views (Views.noise's own minimum)


def azimuth_bin(rows, cols, C, nb):
    """the direction bin of the camera at C (x, y) as seen from each grid cell (rows, cols; arrays)"""
    x = -HX + (cols + 0.5) * RES; y = HY - (rows + 0.5) * RES
    phi = np.arctan2(C[1] - y, C[0] - x)
    return (np.floor((phi + math.pi) / (2 * math.pi) * nb + 0.5).astype(np.int32)) % nb


class DirFloor:
    """per cell and direction bin: views n, colour sums (3) and its summed squares, brightness sum and squares, grey sum;
    over the box (r0, r1, c0, c1) the views cover. Bins are azimuth (nb) x range (edges, m; one bin by default)"""

    def __init__(self, nb=NB, redges=None, box=None):
        self.nb, self.redges = nb, list(redges or []); self.nr = len(self.redges) + 1
        self.box = box or (0, H, 0, W); r0, r1, c0, c1 = self.box; shp = (nb * self.nr, r1 - r0, c1 - c0)
        self.n = np.zeros(shp, np.float32); self.sc = np.zeros(shp + (3,), np.float32); self.sc2 = np.zeros(shp, np.float32)
        self.sr = np.zeros(shp, np.float32); self.sr2 = np.zeros(shp, np.float32); self.sg = np.zeros(shp, np.float32)
        self.srgb = np.zeros(shp + (3,), np.float32)                 # colour sums: the floor's picture from each direction

    @classmethod
    def build(cls, views, nb=NB, sigma=1.0, redges=None, log=None, grey_only=False):
        """from a confirm_stream.Views (its patches, where each camera stood); grey_only: the views' count and grey
        alone (what registration needs), not the comparison's features"""
        import confirm_stream as cs
        sls = np.asarray(views.sls); box = (int(sls[:, 0].min()), int(sls[:, 1].max()), int(sls[:, 2].min()), int(sls[:, 3].max()))
        m = cls(nb, redges, box); t0 = time.time(); r0b, _, c0b, _ = box; WB = box[3] - box[2]; HB = box[1] - box[0]
        fl = lambda a: a.reshape(a.shape[0] * HB * WB, *a.shape[3:])
        N, SC, SC2, SR, SR2, SG, SRGB = (fl(a) for a in (m.n, m.sc, m.sc2, m.sr, m.sr2, m.sg, m.srgb))
        for k, (p, (a0, a1, b0, b1)) in enumerate(zip(views.patches, sls)):
            V = p.any(-1)
            if V.sum() < 50: continue
            g = p.astype(np.float32).mean(-1); rr, cc = np.nonzero(V); R, Cc = rr + a0, cc + b0
            b = azimuth_bin(R, Cc, views.C[k], nb)
            if m.nr > 1:
                d = np.hypot(views.C[k][0] - (-HX + (Cc + 0.5) * RES), views.C[k][1] - (HY - (R + 0.5) * RES))
                b = b * m.nr + np.searchsorted(m.redges, d)
            idx = (b * HB + (R - r0b)) * WB + (Cc - c0b)          # each cell once a view: plain fancy-index adds are exact
            N[idx] += 1; SG[idx] += g[rr, cc]; SRGB[idx] += p[rr, cc]
            if grey_only: continue
            c, r = feats(cs.smooth(p, V, sigma), V); cv = c[rr, cc]; rv = r[rr, cc]
            SC[idx] += cv; SC2[idx] += (cv ** 2).sum(-1); SR[idx] += rv; SR2[idx] += rv ** 2
        if log: log(f"directional floor: {len(views.patches)} views, {nb} x {m.nr} bins over {HB} x {WB} cells, {time.time() - t0:.0f} s")
        return m

    # --- the spread a frame's comparison expects ([H9w]'s view noise, by direction) --------------------------------

    @staticmethod
    def _sd(n, sc, sc2, sr, sr2):
        k = np.maximum(n, 1)
        vc = np.maximum(sc2 / k - ((sc / k[..., None]) ** 2).sum(-1), 0); vr = np.maximum(sr2 / k - (sr / k) ** 2, 0)
        return np.sqrt(vc), np.sqrt(vr)

    def tables(self, min_views=MIN_VIEWS, prior=0.0):
        """per azimuth bin (range bins pooled), the spread to use: its own where it has min_views, else its two
        neighbours pooled with it, else every direction; (colour sd, brightness sd, source 0 none/1 own/2 neighbours/3
        all), each (nb, h, w); cached. prior > 0: instead each bin's variance shrunk toward every direction's as if
        every direction had lent it `prior` views ((n var_bin + prior var_all) / (n + prior)): a bin seen four times
        gives a spread as unsure as four samples are, and this keeps one low draw from setting a hair trigger"""
        if getattr(self, "_tab", None) is not None and self._tab_key == (min_views, prior): return self._tab
        self._tab_key = (min_views, prior)
        nb, nr = self.nb, self.nr; sh = (nb, nr) + self.n.shape[1:]
        n = self.n.reshape(sh).sum(1); sc = self.sc.reshape(sh + (3,)).sum(1); sc2 = self.sc2.reshape(sh).sum(1)
        sr = self.sr.reshape(sh).sum(1); sr2 = self.sr2.reshape(sh).sum(1)
        roll = lambda a: a + np.roll(a, 1, 0) + np.roll(a, -1, 0)
        own = self._sd(n, sc, sc2, sr, sr2); nei = self._sd(roll(n), roll(sc), roll(sc2), roll(sr), roll(sr2))
        alln = n.sum(0); alls = self._sd(alln, sc.sum(0), sc2.sum(0), sr.sum(0), sr2.sum(0))
        src = np.where(n >= min_views, 1, np.where(roll(n) >= min_views, 2, np.where(alln >= min_views, 3, 0))).astype(np.int8)
        pick = lambda i: np.where(src == 1, own[i], np.where(src == 2, nei[i], np.where(src == 3, alls[i][None], 0.0))).astype(np.float32)
        if prior > 0:
            ok = alln >= min_views
            shrink = lambda i: np.where(ok[None], np.sqrt((n * own[i] ** 2 + prior * alls[i][None] ** 2) / (n + prior)), 0.0).astype(np.float32)
            self._tab = (shrink(0), shrink(1), np.where(ok[None], 1, 0).astype(np.int8)); return self._tab
        self._tab = (pick(0), pick(1), src); return self._tab

    def spread(self, sl, C, min_views=MIN_VIEWS, prior=0.0):
        """the colour and brightness spread of the cells in sl (grid slices) as seen from a camera at C (x, y): (sc, sr)
        over sl; zero where nothing was seen"""
        tc, tr, _ = self.tables(min_views, prior); r0b, r1b, c0b, c1b = self.box
        h, w = sl[0].stop - sl[0].start, sl[1].stop - sl[1].start
        out_c = np.zeros((h, w), np.float32); out_r = np.zeros((h, w), np.float32)
        R0, R1, C0, C1 = max(sl[0].start, r0b), min(sl[0].stop, r1b), max(sl[1].start, c0b), min(sl[1].stop, c1b)
        if R1 <= R0 or C1 <= C0: return out_c, out_r
        rr, cc = np.mgrid[R0:R1, C0:C1]; b = azimuth_bin(rr, cc, C, self.nb)
        ii, jj = rr - r0b, cc - c0b
        out_c[R0 - sl[0].start:R1 - sl[0].start, C0 - sl[1].start:C1 - sl[1].start] = tc[b, ii, jj]
        out_r[R0 - sl[0].start:R1 - sl[0].start, C0 - sl[1].start:C1 - sl[1].start] = tr[b, ii, jj]
        return out_c, out_r

    def mean_rgb(self, sl, C, min_views=MIN_VIEWS):
        """the floor's colour over sl as seen from a camera at C: each cell's mean over the views from its direction
        bin, else its neighbours pooled, else every direction; (rgb uint8 over sl, seen)"""
        nb = self.nb; sh = (nb, self.nr) + self.n.shape[1:]
        if getattr(self, "_rgbtab", None) is None:
            n = self.n.reshape(sh).sum(1); s3 = self.srgb.reshape(sh + (3,)).sum(1)
            roll = lambda a: a + np.roll(a, 1, 0) + np.roll(a, -1, 0); n3, s33 = roll(n), roll(s3); na, sa = n.sum(0), s3.sum(0)
            mean = np.where((n >= min_views)[..., None], s3 / np.maximum(n, 1)[..., None],
                            np.where((n3 >= min_views)[..., None], s33 / np.maximum(n3, 1)[..., None], (sa / np.maximum(na, 1)[..., None])[None]))
            self._rgbtab = (np.clip(mean, 0, 255).astype(np.uint8), na >= min_views)
        tab, ok = self._rgbtab; r0b, r1b, c0b, c1b = self.box
        h, w = sl[0].stop - sl[0].start, sl[1].stop - sl[1].start
        out = np.zeros((h, w, 3), np.uint8); have = np.zeros((h, w), bool)
        R0, R1, C0, C1 = max(sl[0].start, r0b), min(sl[0].stop, r1b), max(sl[1].start, c0b), min(sl[1].stop, c1b)
        if R1 <= R0 or C1 <= C0: return out, have
        rr, cc = np.mgrid[R0:R1, C0:C1]; b = azimuth_bin(rr, cc, C, nb); ii, jj = rr - r0b, cc - c0b
        win = (slice(R0 - sl[0].start, R1 - sl[0].start), slice(C0 - sl[1].start, C1 - sl[1].start))
        out[win] = tab[b, ii, jj]; have[win] = ok[ii, jj]
        return out, have

    def mean_grey(self, b, pool_ranges=True):
        """bin b's mean grey over the box, and where it has views"""
        sh = (self.nb, self.nr) + self.n.shape[1:]
        n = self.n.reshape(sh)[b].sum(0); g = self.sg.reshape(sh)[b].sum(0)
        return np.where(n > 0, g / np.maximum(n, 1), 0.0).astype(np.float32), n

    # --- how much is direction ------------------------------------------------------------------------------------

    def decompose(self, min_total=12, min_bin=3):
        """per cell, the analysis of variance of its views by direction bin (all nb x nr bins; bins with fewer than
        min_bin views left out): (cells mask, SS within, SS between, N, K) for colour and brightness"""
        n = self.n; use = n >= min_bin; nk = np.where(use, n, 0)
        N = nk.sum(0); K = use.sum(0)
        out = {}
        for name, s1, s2 in (("colour", self.sc, self.sc2), ("bright", self.sr, self.sr2)):
            if s1.ndim == 4:
                s1u = np.where(use[..., None], s1, 0); mu_k2 = (s1u ** 2).sum(-1) / np.maximum(nk, 1); tot1 = s1u.sum(0); mu2 = (tot1 ** 2).sum(-1) / np.maximum(N, 1)
            else:
                s1u = np.where(use, s1, 0); mu_k2 = s1u ** 2 / np.maximum(nk, 1); tot1 = s1u.sum(0); mu2 = tot1 ** 2 / np.maximum(N, 1)
            s2u = np.where(use, s2, 0).sum(0)
            ssw = np.maximum(s2u - mu_k2.sum(0), 0); sst = np.maximum(s2u - mu2, 0); ssb = np.maximum(sst - ssw, 0)
            out[name] = (ssw, ssb, sst)
        ok = (N >= min_total) & (K >= 2)
        return ok, out, N, K

    # --- where the floor seen from one direction sits against another ([JzW]'s front end) -------------------------

    def bias(self, tile_m=1.2, step_m=0.6, maxs=14, min_cells=3000, log=None):
        """each pair of azimuth bins (a, b): how far bin b's floor must move to lie on bin a's (x, y m), registered tile
        by tile on the bins' mean grey (floor_register's masked edge correlation) and summed house-wide as a robust
        median: {(a, b): (dx, dy, tiles)}. A pose error averages out over the many views of each bin; what is left is
        the floor's direction-dependent displacement (the camera's geometry, the floor's relief, view-dependent shine)"""
        from houndkeeper import floor_register as fr
        G = [self.mean_grey(b) for b in range(self.nb)]
        hb, wb = self.n.shape[1:]; t = int(tile_m / RES); s = int(step_m / RES); out = {}
        for a in range(self.nb):
            for b in range(self.nb):
                if b <= a: continue
                shifts = []
                for i in range(0, max(hb - t, 1), s):
                    for j in range(0, max(wb - t, 1), s):
                        Ga, na = G[a][0][i:i + t, j:j + t], G[a][1][i:i + t, j:j + t] >= MIN_VIEWS
                        Gb, nb_ = G[b][0][i:i + t, j:j + t], G[b][1][i:i + t, j:j + t] >= MIN_VIEWS
                        if na.sum() < min_cells or nb_.sum() < min_cells: continue
                        S = fr.ncc_surfaces(Gb, nb_, Ga, na, maxs, (0.0,), 1.0)
                        if S is None: continue
                        c = S[0][1]; k = np.unravel_index(np.argmax(c), c.shape)
                        if c[k] < 0.3 or k[0] in (0, 2 * maxs) or k[1] in (0, 2 * maxs): continue
                        bl = fr.surface_belief(c, 0.05)
                        if bl is None or np.sqrt(np.trace(bl[1]) / 2) > 1.5: continue
                        dr, dc = bl[0][0] - maxs, bl[0][1] - maxs     # Ga[r + dr] lies under Gb[r]: b's floor sits dr, dc
                        shifts.append((dc * RES, -dr * RES))           # cells from a's; moving b by (dc, -dr) m lays it on a
                if shifts:
                    S_ = np.array(shifts); out[(a, b)] = (float(np.median(S_[:, 0])), float(np.median(S_[:, 1])), len(S_))
                    out[(b, a)] = (-out[(a, b)][0], -out[(a, b)][1], len(S_))
        if log: log(f"directional bias: {len(out) // 2} bin pairs registered")
        return out

    def displacements(self, pairs, min_tiles=3):
        """each bin's own displacement d_k (x, y m; their mean held at zero) from the pairs' registrations (bias()):
        the least-squares d_a - d_b = pair (a, b), weighed by its tiles (a pair from fewer than min_tiles left out).
        One vector a direction, the whole house's: a single pair's registration is noisy, and a pose error of the views
        that happen to fill one bin cannot pass for direction once every pair must agree. Returns (nb, 2), or None"""
        rows, rhs, w = [], [], []
        for (a, b), (dx, dy, n) in pairs.items():
            if a >= b or n < min_tiles: continue
            r = np.zeros(self.nb); r[a], r[b] = 1.0, -1.0; rows.append(r); rhs.append((dx, dy)); w.append(math.sqrt(n))
        if len(rows) < self.nb - 1: return None
        A = np.vstack(rows + [np.ones(self.nb)]); Y = np.vstack(rhs + [(0.0, 0.0)]); sw = np.r_[w, 10.0][:, None]
        return np.linalg.lstsq(A * sw, Y * sw, rcond=None)[0]

    def radial(self, pairs, min_tiles=3):
        """the same displacements with two numbers for the whole house: the floor seen from a camera sits `along` m
        toward it (along the line of sight; a pitch, height or range error does this) and `across` m to its left (a
        yaw error); d_k = along u_k + across u_k', u_k the unit vector from the cell to the camera in bin k. Returns
        ((nb, 2), (along, across)) or None"""
        rows, rhs = [], []
        u = lambda k: np.array([math.cos(-math.pi + 2 * math.pi * k / self.nb), math.sin(-math.pi + 2 * math.pi * k / self.nb)])
        for (a, b), (dx, dy, n) in pairs.items():
            if a >= b or n < min_tiles: continue
            ua, ub = u(a), u(b); pa, pb = np.array([-ua[1], ua[0]]), np.array([-ub[1], ub[0]])
            w = math.sqrt(n); rows += [w * np.c_[ua - ub, pa - pb]]; rhs += [w * np.array([dx, dy])]
        if not rows: return None
        k = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)[0]
        D = np.array([k[0] * u(b) + k[1] * np.array([-u(b)[1], u(b)[0]]) for b in range(self.nb)])
        return D, (float(k[0]), float(k[1]))
