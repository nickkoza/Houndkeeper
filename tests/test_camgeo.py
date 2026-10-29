"""The head camera's floor geometry ([4t7], [JzW]): the range laws of a pitch, height and place error, the views laid
again with a changed camera, the self-calibration's solve on registrations planted from a known camera and floor, and
the floor's relief on the painters' grid."""
import math
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def _camera(h=0.3, pitch_down=0.0, x=0.0, y=0.0, heading=0.0):
    """world <- optical for a camera at (x, y, h) facing `heading`, turned down by pitch_down (rad)"""
    from houndkeeper import calibrate as cal
    ch, sh = math.cos(heading), math.sin(heading)
    R0 = np.array([[sh, 0, ch], [-ch, 0, sh], [0, -1, 0]])        # optical x right, y down, z forward (level)
    T = np.eye(4); T[:3, :3] = R0 @ cal._rot(-pitch_down, 0.0, 0.0); T[:3, 3] = [x, y, h]
    return T


def test_range_laws_match_the_projection():
    """[4t7]: a camera truly turned down by p more than believed lays the floor p (r^2 + h^2)/h too near; truly dh
    higher, dh r / h too near... (predict_dr's signs: the picture's floor toward the camera, +)"""
    import camgeo as cg
    h, r = 0.3, np.array([0.8, 1.5, 2.3])
    Tb = _camera(h); d = np.c_[np.zeros(3), np.ones(3) * h, r]       # sight lines to the floor at r (believed camera)
    d = d / np.linalg.norm(d, axis=1)[:, None]; Pb = cg.floor_points(Tb[:3, 3], Tb[:3, :3], d)
    assert np.allclose(Pb[:, 0], r, atol=1e-9)
    for kw, Tt in (({"dpitch": math.radians(0.2)}, _camera(h, math.radians(0.2))), ({"dup": 0.005}, _camera(h + 0.005))):
        Pt = cg.floor_points(Tt[:3, 3], Tt[:3, :3], d)              # where the same pixels truly look
        toward = Pt[:, 0] - Pb[:, 0]                                 # the true floor further out: the picture toward the camera
        assert np.allclose(toward, cg.predict_dr(r, h, **kw), rtol=0.03), (kw, toward, cg.predict_dr(r, h, **kw))
    assert cg.predict_dr(2.0, 0.3, dfwd=0.004) == 0.004
    assert cg.predict_dr(1.0, 0.3, zfloor=0.012) == -0.04           # a carpet 12 mm proud: 4 cm out at 1 m


def _view(T, size=240):
    """one view of a striped floor patch painted from camera T, on the floor grid"""
    import confirm_stream as cs
    from detect_changes2 import RES, HX, HY
    C = T[:3, 3]; r0 = int((HY - C[1]) / RES) - size // 2; c0 = int((C[0] + 0.5 + HX) / RES)
    rr, cc = np.mgrid[r0:r0 + size, c0:c0 + size]; X = -HX + (cc + 0.5) * RES
    g = (128 + 100 * np.sign(np.sin(X * 2 * math.pi / 0.2))).astype(np.uint8)     # stripes across the line of sight
    p = np.repeat(g[..., None], 3, -1)
    f = T[:3, 2][:2] / np.linalg.norm(T[:3, 2][:2])
    return cs.Views([C[:2]], [f], [(r0, r0 + size, c0, c0 + size)], [p], [0.0], [T])


def test_regeom_moves_the_floor_along_the_line_of_sight_by_the_range_law():
    """[4t7]: the views laid again as a camera taken 1 cm higher would have laid them: each stripe edge moves away from
    the camera by dh r / h"""
    import camgeo as cg
    from detect_changes2 import RES, HX
    T = _camera(0.3); v = _view(T); w = cg.regeom(v, dup=0.01)
    def edges(vv):
        p = vv.patches[0][..., 0].astype(int); r0, _, c0, _ = vv.sls[0]; row = p[p.shape[0] // 2]
        k = np.nonzero((row[1:] > 128) & (row[:-1] <= 128) & (row[:-1] > 0))[0]
        return -HX + (c0 + k + 1) * RES
    e0, e1 = edges(v), edges(w)
    n = min(len(e0), len(e1)); assert n >= 3
    moved = e1[:n] - e0[:n]; want = 0.01 * e0[:n] / 0.3
    assert np.allclose(moved, want, atol=0.012), (moved, want)       # to a cell or so
    assert moved[-1] > moved[0] > 0


def test_solve_tiles_recovers_a_planted_camera_and_floor():
    """[4t7]: registrations made from a known geometry (pitch, yaw, height, a raised tile, each tile's consensus
    offset) give it back; the floor's level is the house's lowest floor"""
    import camgeo as cg
    rng = np.random.default_rng(1); h = 0.3; dp, dy, du = math.radians(-0.2), math.radians(0.4), 0.003
    rows = []
    tiles = [(i, j) for i in range(6) for j in range(6)]; raised = set(tiles[:8])
    for t in tiles:
        c = rng.normal(0, 0.01, 2)
        for k in range(16):
            ang = rng.uniform(-math.pi, math.pi); r = rng.uniform(0.6, 2.4); u = np.array([math.cos(ang), math.sin(ang)])
            f = -u; l = np.array([-f[1], f[0]]); z = 0.012 if t in raised else 0.0
            g = -dp * (r * r + h * h) / h + (du - z) * r / h
            e = g * u - dy * r * l - c
            rows.append((t[0], t[1], k, r, u[0], u[1], f[0], f[1], e[0], e[1], 0.9))
    glob, a_t = cg.solve_tiles(np.array(rows), h)
    assert abs(glob["dpitch_deg"] - math.degrees(dp)) < 0.01 and abs(glob["dyaw_deg"] - math.degrees(dy)) < 0.01
    up = cg.floor_level(a_t)
    assert abs(up - du) < 0.0005
    rel = {t: up - a for t, (a, _) in a_t.items()}
    assert all(abs(rel[t] - (0.012 if t in raised else 0.0)) < 0.001 for t in tiles)


def test_relief_is_read_on_the_painters_grid():
    """[4t7]: the relief's cell for a point is the floor grid's (north up, centred), 0 off it or without a relief"""
    from houndkeeper import floor_relief as frl
    g = np.zeros((980, 1680), np.float32); g[100, 200] = 0.012
    x, y = -8.4 + 200.5 * 0.01, 4.9 - 100.5 * 0.01
    assert frl.at(g, np.array([x]), np.array([y]))[0] == np.float32(0.012)
    assert frl.at(g, np.array([x + 0.01]), np.array([y]))[0] == 0.0
    assert frl.at(g, np.array([20.0]), np.array([0.0]))[0] == 0.0 and frl.at(None, np.array([x]), np.array([y]))[0] == 0.0
