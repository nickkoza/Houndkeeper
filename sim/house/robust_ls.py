"""A linear least-squares system built row by row, and its robust solve on scipy.optimize.least_squares or on a
GTSAM factor graph (decision [2Ke]): the springs' solve (stitch_floor.py) and the lidar's shape solve (house_shape.py) are both linear in
their small corrections, whitened row by row by each measurement's information. Rows in a robust group (a pair's
match) take a true robust loss (Huber, soft_l1 or Cauchy) whose scale is measured from the rows themselves; every other row
(odometry, priors, the frame's anchor) stays quadratic. The Jacobian is the system's own sparse matrix, exact.
The old path (iteratively reweighted lsqr) builds the same rows and scales a group's rows by its weight."""
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import coo_matrix, diags
from scipy.sparse.linalg import lsqr, spsolve

HUBER_K = 1.345                     # Huber's tuning constant, in sds: 95% efficient on Gaussian noise
MAD_SD = 1.4826                     # median absolute deviation -> sd, for Gaussian noise


class Rows:
    """rows of a sparse linear system A x = b, each with the robust group it belongs to (-1: none)"""

    def __init__(self):
        self.rows, self.cols, self.vals, self.rhs, self.groups = [], [], [], [], []

    def eq(self, terms, b, w=1.0, group=-1):
        """one row: sum(v * x[c] for c, v in terms) = b, the whole row times w"""
        r = len(self.rhs)
        for c_, v_ in terms: self.rows.append(r); self.cols.append(c_); self.vals.append(v_ * w)
        self.rhs.append(b * w); self.groups.append(group)

    def system(self, nx):
        """(A csr, b, group per row)"""
        A = coo_matrix((self.vals, (self.rows, self.cols)), shape=(len(self.rhs), nx)).tocsr()
        return A, np.array(self.rhs, float), np.array(self.groups, int)


def lsqr_weighted(A, b, groups, w):
    """the old path's one pass: each robust group's rows scaled by its weight w[group], solved by lsqr"""
    s = np.ones(len(b)); g = groups >= 0; s[g] = w[groups[g]]
    return lsqr(diags(s) @ A, b * s, atol=1e-10, btol=1e-10, iter_lim=20000)[0]


def plain(A, b):
    """the plain least-squares answer by a sparse direct solve of the normal equations (the robust solves' start)"""
    return spsolve((A.T @ A).tocsc(), A.T @ b)


def pinned(cov):
    """each block's marginal covariance (n, 3, 3) -> (sd of its position along its weakest direction m, sd of its turn
    rad): how loosely the solve holds it"""
    return np.sqrt(np.clip(np.linalg.eigvalsh(cov[:, :2, :2])[:, -1], 0, None)), np.sqrt(np.clip(cov[:, 2, 2], 0, None))


def masked_loss(robust, kind="huber"):
    """a least_squares loss (z = f^2 -> rho, rho', rho'') that is `kind` on the robust rows and quadratic on the rest"""
    def loss(z):
        out = np.empty((3, len(z))); out[0], out[1], out[2] = z, 1.0, 0.0
        zr = z[robust]
        if kind == "huber":
            big = zr > 1.0; sq = np.sqrt(np.where(big, zr, 1.0))
            out[0, robust] = np.where(big, 2 * sq - 1, zr); out[1, robust] = np.where(big, 1 / sq, 1.0)
            out[2, robust] = np.where(big, -0.5 / (sq * np.where(big, zr, 1.0)), 0.0)
        elif kind == "soft_l1":
            t = 1 + zr; out[0, robust] = 2 * (np.sqrt(t) - 1); out[1, robust] = t ** -0.5; out[2, robust] = -0.5 * t ** -1.5
        else:
            raise ValueError(kind)
        return out
    return loss


def noise_sd(r):
    """the rows' noise, measured robustly: 1.4826 x their median absolute residual (outliers barely move it)"""
    return MAD_SD * float(np.median(np.abs(r))) if len(r) else 1.0


def robust_start(A, b, groups, k=HUBER_K, rounds=3):
    """a start and the robust rows' noise: the plain answer, then a few Huber-weighted direct solves, each with the
    noise measured again where the last left it (measured at the plain answer alone, the outliers' smear inflates it:
    4.6x on a test graph with 10% outliers). Returns (x, sd in the rows' whitened units)"""
    robust = groups >= 0; x = plain(A, b)
    for _ in range(rounds):
        r = A @ x - b; sd = max(noise_sd(r[robust]), 1e-12)
        w = np.ones(len(b)); w[robust] = np.minimum(1.0, k * sd / np.maximum(np.abs(r[robust]), 1e-12))   # Huber's
        x = spsolve((A.T @ diags(w) @ A).tocsc(), A.T @ (w * b))                                          # own weight
    return x, max(noise_sd((A @ x - b)[robust]), 1e-12)


def solve_robust(A, b, groups, loss="huber", k=HUBER_K, max_nfev=1000):
    """the robust solve: from robust_start's answer, the loss's scale k x the robust rows' noise measured there; then
    least_squares (trf, lsmr, the exact sparse Jacobian A). Returns (x, info dict)"""
    robust = groups >= 0
    if not robust.any(): return plain(A, b), {"f_scale": float("nan"), "sd": float("nan"), "nfev": 0, "status": 0}
    x0, sd = robust_start(A, b, groups, k); f_scale = k * sd
    sol = least_squares(lambda x: A @ x - b, x0, jac=lambda x: A, method="trf", tr_solver="lsmr",
                        loss=masked_loss(robust, loss), f_scale=f_scale, x_scale=1.0, max_nfev=max_nfev,
                        ftol=1e-10, xtol=1e-10, gtol=1e-10,   # lsmr's own defaults stop 18 cm short (house 90's shape,
                        tr_options={"atol": 1e-12, "btol": 1e-12, "maxiter": 5000})   # 200 evaluations); this, 3.5-4 mm short in 1000 ([2Ke])
    return sol.x, {"f_scale": f_scale, "sd": sd, "nfev": int(sol.nfev), "status": int(sol.status)}


def group_norms(r, groups, n):
    """each robust group's whitened residual norm (r per row -> sqrt(sum r^2) per group)"""
    g = groups >= 0; out = np.zeros(n); np.add.at(out, groups[g], r[g] ** 2); return np.sqrt(out)



def _factors(A, groups):
    """the system's rows as factors over 3-variable blocks: each robust group one factor, and the other rows gathered
    by the blocks they touch: [(row indices, blocks, robust)]"""
    A = A.tocsr(); by = {}
    for r in range(A.shape[0]):
        blocks = tuple(sorted(set((A.indices[A.indptr[r]:A.indptr[r + 1]] // 3).tolist())))
        key = ("g", int(groups[r])) if groups[r] >= 0 else ("b", blocks)
        e = by.setdefault(key, [[], set()]); e[0].append(r); e[1].update(blocks)
    return [(np.array(rs), tuple(sorted(bs)), k[0] == "g") for k, (rs, bs) in by.items()]


def solve_gtsam(A, b, groups, loss="huber", k=HUBER_K, marginals=False, max_iter=100):
    """the same system as a GTSAM factor graph solved by Levenberg-Marquardt ([2Ke]): each 3-variable block one
    Vector3, each robust group one factor whose whole whitened residual takes the robust noise model (Huber or
    Cauchy, its threshold k x the rows' noise as robust_start measures it x sqrt(its rows)), the other rows plain unit factors. Returns
    (x, info dict); with marginals, info["cov"] is each block's 3x3 marginal covariance (n_blocks, 3, 3)"""
    import gtsam
    A = A.tocsr(); nx = A.shape[1]; nb = nx // 3; robust = groups >= 0
    if loss not in ("huber", "cauchy"): raise ValueError(loss)
    x0, sd = robust_start(A, b, groups, k) if robust.any() else (plain(A, b), 1.0)
    graph = gtsam.NonlinearFactorGraph(); init = gtsam.Values()
    for i in range(nb): init.insert(i, np.asarray(x0[3 * i:3 * i + 3], float))
    for rs, blocks, is_rob in _factors(A, groups):
        sub = A[rs].toarray(); Js = [np.ascontiguousarray(sub[:, 3 * q:3 * q + 3]) for q in blocks]; bb = b[rs].copy()
        model = gtsam.noiseModel.Unit.Create(len(rs))
        if is_rob:
            c = k * sd * math.sqrt(len(rs))
            est = gtsam.noiseModel.mEstimator.Huber.Create(c) if loss == "huber" else gtsam.noiseModel.mEstimator.Cauchy.Create(c)
            model = gtsam.noiseModel.Robust.Create(est, model)

        def err(this, v, H, Js=Js, bb=bb):
            e = -bb.copy()
            for q, J in zip(this.keys(), Js): e += J @ v.atVector(q)
            if H is not None:
                for n_, J in enumerate(Js): H[n_] = J
            return e
        graph.add(gtsam.CustomFactor(model, list(blocks), err))
    p = gtsam.LevenbergMarquardtParams(); p.setMaxIterations(max_iter); p.setRelativeErrorTol(1e-7)   # 11 um from converged (house 90's shape)
    res = gtsam.LevenbergMarquardtOptimizer(graph, init, p).optimize()
    x = np.concatenate([res.atVector(i) for i in range(nb)])
    info = {"f_scale": k * sd, "sd": sd, "nfev": 0, "status": 0}
    if marginals:                                   # how firmly each block is pinned, robust weights as solved
        M = gtsam.Marginals(graph, res); info["cov"] = np.array([M.marginalCovariance(i) for i in range(nb)])
    return x, info
