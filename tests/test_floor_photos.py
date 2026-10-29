"""[N4z]: photographed floors under the arm; some held out for judging only; their relief lit from the right side."""
import os, sys
from pathlib import Path
import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "arm")); sys.path.insert(0, str(HK / "src" / "houndkeeper"))
os.environ.setdefault("MUJOCO_GL", "egl")
import floors  # noqa: E402

needs_photos = pytest.mark.skipif(not floors.PHOTO_ZIPS.exists(), reason="tools/fetch_floors.sh not run")


def test_held_out_floors_never_train():
    train = {r["id"] for r in floors.photos("train")}; held = {r["id"] for r in floors.photos("heldout")}
    assert held and train and not (train & held)
    for kind in ("carpet", "wood", "tile"):                      # every kind has floors for judging
        assert floors.photos("heldout", [kind])


@needs_photos
def test_generated_floors_come_from_the_split(monkeypatch):
    held = {r["id"] for r in floors.photos("heldout")}
    for split in ("train", "heldout"):
        monkeypatch.setattr(floors, "SPLIT", split)
        ids = {floors.generate(s, rug=False)[1].split(":")[1] for s in range(12)}
        assert (ids <= held) if split == "heldout" else not (ids & held)


@needs_photos
def test_photo_floor_repeats_without_a_seam():
    a = floors.generate(3, rug=False, marks=False)[0].mean(-1)
    inner = np.abs(np.diff(a, axis=1)).mean()                    # neighbour differences inside the patch
    seam = np.abs(a[:, 0] - a[:, -1]).mean()                     # across the join to the next copy
    assert seam < 2.5 * inner


@needs_photos
@pytest.mark.parametrize("axis", [0, 1])
def test_relief_is_lit_from_the_side_it_faces(axis):
    """half the floor tilts toward +x (or +y), half toward -x: under a light from +x the first half is the brighter, and
    under a light from -x the second: the normal map's directions survive the photo's convention, the upload and the
    renderer (compared within one frame, since each frame's exposure is metered)"""
    mujoco = pytest.importorskip("mujoco"); pytest.importorskip("mujofil"); import filament_cameras as fc
    for sign in (1, -1):
        spec = mujoco.MjSpec(); w = spec.worldbody
        d_ = [0.0, 0.0, -0.5]; d_[axis] = -sign * 1.0               # low sun from the +axis (sign 1) or -axis side: even light
        l = w.add_light(); l.name = "sun"; l.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL; l.dir = d_; l.diffuse = [1, 1, 1]
        c = w.add_camera(); c.name = "top"; c.pos = [0, 0, 0.5]     # image right = +x, image up = +y
        floors.add_to_spec(spec); m = spec.compile(); d = mujoco.MjData(m)
        n = np.zeros((floors.PX, floors.PX, 3), np.float32); n[..., 2] = 0.8
        lo = (slice(None), slice(0, floors.PX // 2)) if axis == 0 else (slice(0, floors.PX // 2), slice(None))   # x < 0 (or y < 0)
        n[..., axis] = 0.6; n[lo + (axis,)] = -0.6                   # the +half tilts toward +axis, the -half toward -axis
        nrgb, orm = floors._encode(n, np.full((floors.PX, floors.PX), 0.9, np.float32), np.ones((floors.PX, floors.PX), np.float32))
        floors._upload(m, np.full((floors.PX, floors.PX, 3), 0.6, np.float32), (), None, filament=False)
        floors._upload_pbr(m, (nrgb, orm), (), None, "local_floor"); mujoco.mj_forward(m, d)
        fc._BY_SIZE.clear(); im = fc.render(m, d, "top", 160, 120).mean(-1)
        plus, minus = (im[20:100, 90:140].mean(), im[20:100, 20:70].mean()) if axis == 0 else (im[10:50, 30:130].mean(), im[70:110, 30:130].mean())
        lit, dark = (plus, minus) if sign == 1 else (minus, plus)
        assert lit > 1.15 * dark, (sign, plus, minus)
