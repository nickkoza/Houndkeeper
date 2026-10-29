"""The robust solves the springs and the shape can select ([2Ke]): a true robust loss, never a redescending one; the
robust rows only; outliers held off; GTSAM's answer the converged Huber's; the old path's pass unchanged."""
import sys
from pathlib import Path

import numpy as np
import pytest
import scipy.sparse as sp

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "house"))
import robust_ls as rl  # noqa: E402


def _chain(n=15, seed=0, outliers=0.1, sd=0.01):
    """a small pose graph, linear: blocks of 3, relative measurements between random pairs (whitened by sd), some of
    them gross outliers; odometry-like ties between neighbours; block 0 anchored. (A, b, groups, truth)"""
    rng = np.random.default_rng(seed); x = rng.normal(0, 0.1, 3 * n); x[:3] = 0; S = rl.Rows(); g = 0
    for _ in range(6 * n):
        i, j = sorted(rng.choice(n, 2, replace=False)); d = x[3 * j:3 * j + 3] - x[3 * i:3 * i + 3] + rng.normal(0, sd, 3)
        if rng.random() < outliers: d = d + rng.normal(0, 0.3, 3)
        for k in range(3): S.eq([(3 * j + k, 1.0), (3 * i + k, -1.0)], d[k], 1 / sd, g)
        g += 1
    for i in range(n - 1):
        for k in range(3): S.eq([(3 * (i + 1) + k, 1.0), (3 * i + k, -1.0)], x[3 * (i + 1) + k] - x[3 * i + k] + rng.normal(0, 0.05), 1 / 0.05)
    for k in range(3): S.eq([(k, 1.0)], 0.0, 1e4)
    A, b, groups = S.system(3 * n)
    return A, b, groups, x


def test_rows_build_the_system_with_weights_and_groups():
    S = rl.Rows(); S.eq([(0, 1.0), (2, -2.0)], 3.0, 2.0, 5); S.eq([(1, 1.0)], 1.0)
    A, b, g = S.system(3)
    assert np.allclose(A.toarray(), [[2, 0, -4], [0, 1, 0]]) and np.allclose(b, [6, 1]) and list(g) == [5, -1]


@pytest.mark.parametrize("kind", ["huber", "soft_l1"])
def test_loss_is_robust_on_robust_rows_only_and_its_derivatives_hold(kind):
    z = np.array([0.25, 4.0, 25.0, 0.25, 25.0]); robust = np.array([True, True, True, False, False])
    rho = rl.masked_loss(robust, kind)(z)
    assert np.allclose(rho[:, 3:], [[0.25, 25.0], [1, 1], [0, 0]])          # the other rows stay quadratic
    h = 1e-6; num = (rl.masked_loss(robust, kind)(z + h)[0] - rl.masked_loss(robust, kind)(z - h)[0]) / (2 * h)
    assert np.allclose(rho[1], num, atol=1e-6)
    num2 = (rl.masked_loss(robust, kind)(z + h)[1] - rl.masked_loss(robust, kind)(z - h)[1]) / (2 * h)
    assert np.allclose(rho[2], num2, atol=1e-5)


@pytest.mark.parametrize("kind", ["huber", "soft_l1"])
def test_loss_influence_never_falls_with_the_residual(kind):
    """a true robust loss: the pull of a residual r (r * rho'(r^2)) rises and then levels off; the old path's squared
    weights gave k^2 / r, which falls (critique #12)"""
    r = np.linspace(0.01, 50, 500); psi = r * rl.masked_loss(np.ones(len(r), bool), kind)(r ** 2)[1]
    assert np.all(np.diff(psi) >= -1e-12)
    old = r * np.minimum(1, 1.0 / r) ** 2
    assert old[-1] < old[np.searchsorted(r, 1.0)]                            # the old one redescends


def test_noise_is_measured_robustly():
    rng = np.random.default_rng(1); r = rng.normal(0, 2.0, 20000); r[:2000] = 100.0
    assert rl.noise_sd(r) == pytest.approx(2.0 * 1.13, rel=0.1)              # 10% outliers move the MAD by ~13%, not 50x


def test_old_path_pass_with_unit_weights_is_plain_least_squares():
    A, b, g, _x = _chain(outliers=0.0)
    x1 = rl.lsqr_weighted(A, b, g, np.ones(g.max() + 1)); x2 = rl.plain(A, b)
    assert np.max(np.abs(x1 - x2)) < 1e-6


def test_least_squares_holds_off_outliers():
    A, b, g, x = _chain()
    e_plain = np.abs(rl.plain(A, b) - x).max(); xs, info = rl.solve_robust(A, b, g)
    assert info["sd"] == pytest.approx(1.0, rel=0.35)                        # whitened rows: their noise is ~1 sd
    assert np.abs(xs - x).max() < 0.5 * e_plain


def _group_huber(A, b, g, c):
    """converged IRLS with Huber's weight on each group's whole residual, threshold c x sqrt(its rows)"""
    rob = g >= 0; d = np.bincount(g[rob]); x = rl.plain(A, b)
    for _ in range(500):
        r = A @ x - b; w = np.ones(len(b)); wg = np.minimum(1, c * np.sqrt(d) / np.maximum(rl.group_norms(r, g, len(d)), 1e-12))
        w[rob] = wg[g[rob]]; xn = sp.linalg.spsolve((A.T @ sp.diags(w) @ A).tocsc(), A.T @ (w * b))
        if np.max(np.abs(xn - x)) < 1e-12: return xn
        x = xn
    return x


def test_gtsam_gives_the_converged_huber_and_marginals():
    pytest.importorskip("gtsam")
    A, b, g, x = _chain()
    xg, info = rl.solve_gtsam(A, b, g, marginals=True)
    assert np.abs(xg - _group_huber(A, b, g, info["f_scale"])).max() < 1e-4
    assert np.abs(xg - x).max() < 0.5 * np.abs(rl.plain(A, b) - x).max()
    sxy, sth = rl.pinned(info["cov"])
    assert info["cov"].shape == (15, 3, 3) and sxy[0] < 1e-3 and np.all(sxy[1:] > sxy[0])   # the anchored block is pinned
