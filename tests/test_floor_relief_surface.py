"""The floor's relief as a surface ([yFg], [4t7], [JzW]): from view-pair registrations that move as a floor's height
moves each view's copy of it (away from its camera by r z / h), the solve recovers a raised step and a lowered one, keeps
the step's edge, and leaves the floor around them level."""
import math
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def _pairs(Z, node_m=0.1, h=0.3, noise=0.004, seed=0):
    """synthetic view-pair registrations over a 5.4 x 1.8 m patch of floor whose true node heights are Z: each 0.3 m tile
    seen by views from four sides at 1-2 m, each pair's displacement - (z / h) (r_k u_k - r_j u_j) plus noise"""
    import floor_selfcal as fs
    rng = np.random.default_rng(seed); NA, NB = Z.shape; rows, feet = [], []
    for ti in range(1, 7):
        for tj in range(1, 19):
            rr = np.arange(ti * 30, ti * 30 + 30); cc = np.arange(tj * 30, tj * 30 + 30)
            R, C = [a.ravel() for a in np.meshgrid(rr, cc, indexing="ij")]
            ni, w = fs._footprint(R, C, node_m, NB); z = float(w @ Z.reshape(-1)[ni])
            views = [(rng.uniform(1.0, 2.0), np.array([math.cos(a), math.sin(a)])) for a in rng.uniform(0, 2 * math.pi, 6)]
            for k in range(len(views)):
                for j in range(k + 1, len(views)):
                    (rk, uk), (rj, uj) = views[k], views[j]; g = rk * uk - rj * uj
                    e = -(z / h) * g + rng.normal(0, noise, 2)
                    rows.append((ti, tj, k, j, rk, uk[0], uk[1], rj, uj[0], uj[1], e[0], e[1], 0.8)); feet.append((ni, w))
    return np.array(rows), feet


def _truth(NA=40, NB=60):
    Z = np.zeros((NA, NB)); Z[10:22, 12:30] = 0.010; Z[12:20, 45:58] = -0.006   # a 1 cm raised rectangle, a 6 mm dip
    return Z


def test_a_planted_raised_step_and_a_dip_are_recovered():
    """[yFg]: the positive control in small: both ways, and the level floor stays level"""
    import floor_selfcal as fs
    Z = _truth(); pr, feet = _pairs(Z)
    dz, st, sup = fs.solve_surface_pairs(pr, feet, 0.3, np.zeros_like(Z))
    seen = sup > 0.5
    inner_up = np.zeros_like(seen); inner_up[12:20, 14:28] = True; inner_dn = np.zeros_like(seen); inner_dn[14:18, 47:56] = True
    level = seen & (Z == 0)
    assert abs(np.median(dz[inner_up & seen]) - 0.010) < 0.0015
    assert abs(np.median(dz[inner_dn & seen]) + 0.006) < 0.0015
    assert abs(np.median(dz[level])) < 0.001


def test_the_step_keeps_its_edge():
    """[yFg]: the edge-keeping smoothness: within two nodes (20 cm) of the step's edge the height goes most of the way"""
    import floor_selfcal as fs
    Z = _truth(); pr, feet = _pairs(Z, noise=0.002)
    dz, st, sup = fs.solve_surface_pairs(pr, feet, 0.3, np.zeros_like(Z))
    row = dz[16, 6:36]                                        # across the raised step's left and right edges (cols 12, 30)
    assert row[12 - 6 + 2] > 0.007 and row[12 - 6 - 3] < 0.003
