"""[FHA]: a floor cell reads the average of the pixels it covers, not the one nearest its centre (the nearest one alone
aliased fine texture and edges: two maps of the same floor correlated 0.60-0.94, 26 Sep 2026)."""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "house")); sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import floor_sample as fs


def test_a_cell_spanning_many_pixels_reads_their_average():
    img = np.zeros((64, 64, 3), np.uint8); img[::2, :] = 200; img[1::2, :] = 0     # 1-pixel stripes: mean 100
    pyr = fs.pyramid(img)
    near = fs.sample(pyr, np.array([32.0, 20.0]), np.array([32.0, 41.0]), np.array([4.0, 8.0]))
    assert np.allclose(near, 100.0, atol=1.0)
    one = fs.sample(pyr, np.array([32.0, 32.0]), np.array([32.0, 33.0]), np.array([1.0, 1.0]))   # a cell of one pixel:
    assert one[:, 0].tolist() == [200.0, 0.0]                                    # that pixel ([AKY]: centres at integers)


def test_cells_span_fewer_pixels_further_away_and_at_a_graze():
    near, far = fs.px_per_cell(0.01, np.array([0.5, 2.0]), 370.0, 0.3)
    assert near > far
    assert fs.px_per_cell(0.01, 1.0, 370.0, 1.0) > fs.px_per_cell(0.01, 1.0, 370.0, 0.3)   # looking down: more pixels


def _ray_frame():
    """the head camera's real lens over a scene whose every point shows its own ray: a centred pinhole render (as the sim
    renders the head camera before warping it) holding each pixel's ray (x, y) x 1000, warped into the fisheye the lens
    delivers and straightened as every consumer straightens it (lens.Rectifier)"""
    from houndkeeper import lens
    W, H = 640, 360; K, D = lens.calibration(W, H); fovy = lens.render_fovy(W, H)
    sw, sh = 1280, 720; f = (sh / 2) / np.tan(np.radians(fovy) / 2)
    c, r = np.meshgrid(np.arange(sw, dtype=np.float32), np.arange(sh, dtype=np.float32))
    img = np.stack([(c + 0.5 - sw / 2) / f * 1000, (r + 0.5 - sh / 2) / f * 1000, np.zeros_like(c)], -1).astype(np.float32)
    rect = lens.Rectifier(K, D, W, H)
    return rect(lens.to_fisheye(img, W, H, fovy)), rect.K


def test_a_floor_point_reads_the_pixel_its_ray_lands_on_at_every_level():
    """[FHA] decision [AKY]: a point projected through the straightened head camera's K reads the colour its own ray
    shows, at every pyramid level, to a few hundredths of a pixel. Corner-based reads (u - 0.5) sat half a pixel up and
    left: ~1.2-1.9 cm along the view at 2-2.5 m"""
    img, (fx, fy, cx, cy) = _ray_frame(); pyr = fs.pyramid(img)
    x = np.array([0.1, -0.2, 0.15, 0.0, 0.3]); y = np.array([0.3, 0.25, -0.1, 0.2, 0.35])
    for ppc in (1.0, 2.0, 4.0, 8.0):
        got = fs.sample(pyr, fx * x + cx, fy * y + cy, np.full(len(x), ppc)) / 1000
        assert np.abs((got[:, 0] - x) * fx).max() < 0.05 and np.abs((got[:, 1] - y) * fy).max() < 0.05, ppc


def test_the_wrist_cameras_centred_render_takes_the_same_convention():
    """[FHA] decision [AKY]: the wrist camera's K puts its centred render's centre half a pixel short of w / 2, so a point
    on its axis reads the middle of the frame"""
    import floor_mosaic2 as fm
    fx, fy, cx, cy = fm.wrist_intrinsics(64, 48)
    img = np.zeros((48, 64, 3), np.float32); img[..., 0] = np.arange(64)[None] + 0.5 - 32   # each pixel its centre's offset
    got = fs.sample(fs.pyramid(img), np.array([cx]), np.array([cy]), np.array([1.0]))
    assert abs(got[0, 0]) < 1e-6
