"""The floor's appearance by the direction it is seen from ([4t7]), on synthetic floors: the direction bins, the spread a
frame reads from its own direction, the fallback where a direction was never seen, and the displacement the floor
seen from one side carries against the floor seen from the other."""
import math
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def _cell_xy(r, c):
    import dirfloor as d
    return -d.HX + (c + 0.5) * d.RES, d.HY - (r + 0.5) * d.RES


def test_azimuth_bin_is_where_the_camera_stands_seen_from_the_cell():
    """[4t7]: bin 0 is the camera to the west of the cell (-180 deg), bins counted anticlockwise"""
    import dirfloor as d
    r, c = 300, 500; x, y = _cell_xy(r, c)
    b = lambda dx, dy: int(d.azimuth_bin(np.array([r]), np.array([c]), (x + dx, y + dy), 8)[0])
    assert b(-1, 0) == 0 and b(0, -1) == 2 and b(1, 0) == 4 and b(0, 1) == 6 and b(1, 1) == 5


def _views(glint_from_east, n_each=4):
    """a plain floor seen from the east and from the west; from the east a glint on part of it that varies view to view"""
    import confirm_stream as cs
    box = (300, 360, 500, 560); x, y = _cell_xy(330, 530); patches, C = [], []
    for side in (+1, -1):
        for k in range(n_each):
            p = np.full((60, 60, 3), 120, np.uint8)
            if side == +1 and glint_from_east: p[20:40, 20:40] = 120 + 50 * (k % 2)
            patches.append(p); C.append([x + side * 1.5, y])
    return cs.Views(C=C, F=[[-s, 0] for s in [1] * n_each + [-1] * n_each], sls=[box] * len(patches), patches=patches)


def test_spread_is_read_from_the_frames_own_direction():
    """[4t7]: the glint varies only from the east, so a frame from the west expects no spread there, one from the
    east does; every direction at once (Views.noise) mixes the two"""
    import dirfloor as d
    V = _views(True); m = d.DirFloor.build(V, 8, 0.0)
    x, y = _cell_xy(330, 530); sl = (slice(325, 335), slice(525, 535))
    _, sr_e = m.spread(sl, (x + 1.5, y)); _, sr_w = m.spread(sl, (x - 1.5, y))
    assert sr_e.mean() > 0.1 and sr_w.max() < 0.01
    _, nr, _ = V.noise(0.0)
    assert 0.0 < nr[330, 530] < sr_e.mean()


def test_an_unseen_direction_falls_back_to_every_direction():
    """[4t7]: from the north no view was taken, nor from its neighbouring bins: the spread is every direction's"""
    import dirfloor as d
    V = _views(True); m = d.DirFloor.build(V, 8, 0.0); x, y = _cell_xy(330, 530); sl = (slice(330, 331), slice(530, 531))
    _, sr_n = m.spread(sl, (x, y + 1.5)); _, nr, _ = V.noise(0.0)
    assert abs(float(sr_n[0, 0]) - float(nr[330, 530])) < 1e-4
    _, sr_w = m.spread(sl, (x - 1.5, y), prior=8)                # with a prior, the west's zero spread is pulled up
    assert 0.0 < float(sr_w[0, 0]) < float(nr[330, 530])


def test_displacements_solve_one_vector_a_direction_from_the_pairs():
    """[4t7]/[JzW]: pair registrations d_a - d_b, noise-free, give back the bins' own displacements (mean zero), and the
    two-number form recovers a floor that sits toward each camera"""
    import dirfloor as d
    m = d.DirFloor(8, box=(0, 2, 0, 2)); along = 0.03
    u = lambda k: np.array([math.cos(-math.pi + 2 * math.pi * k / 8), math.sin(-math.pi + 2 * math.pi * k / 8)])
    D = np.array([along * u(k) for k in range(8)])
    pairs = {(a, b): (*(D[a] - D[b]), 10) for a in range(8) for b in range(8) if a != b}
    assert np.allclose(m.displacements(pairs), D, atol=1e-9)
    Dr, (al, ac) = m.radial(pairs)
    assert abs(al - along) < 1e-9 and abs(ac) < 1e-9 and np.allclose(Dr, D, atol=1e-9)
