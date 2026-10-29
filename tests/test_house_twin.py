"""[UdA] [POG] [TgM] [au3] [4TQ] [JLx]: the house twin: built from a run's own outputs with no truth in reach, its floor
made by the ground-truthing tools' registration, lit and shaken as the procedural houses are, kept in data/, its general
part DimOS's alone, and its scorer's measures."""
import json
import re
import shutil
import sys
from pathlib import Path

from houndkeeper import workspace

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))
ht = pytest.importorskip("house_twin")
FORK_MODULE = workspace.sibling("dimos-twin") / "dimos" / "simulation" / "mujoco" / "scene_from_map.py"
RUN95 = HK / "sim" / "house" / "runs" / "nights_s95"


def test_the_builder_refuses_to_open_sim_truth():
    """[UdA]: nothing of a sim house's truth is read: its truth files, true path and true floor texture are refused"""
    for name in ("nights_s70_n1.truth.json", "nights_s70_n1.traj.csv", "house_floor_s70_n1.png"):
        with pytest.raises(SystemExit): ht.opened(Path("/x") / name)
    assert ht.opened("/x/mosaic_n1.npz") == Path("/x/mosaic_n1.npz")


def _run_without_truth(tmp_path):
    """house 95's run dir as the replay path leaves it, its truth left behind"""
    ff = json.loads((RUN95 / "floor_frame.json").read_text())
    if not all(Path(d).exists() for d in ff["dbs"]): pytest.skip("house 95's recordings are not on this machine")
    run = tmp_path / "run"; (run / "shape").mkdir(parents=True)
    for f in ("floor_frame.json", "mosaic_n1.npz", "mosaic_n2.npz", "shape/corrections_1.npz", "shape/corrections_2.npz", "shape/settle.npz"):
        shutil.copy(RUN95 / f, run / f)
    assert not list(run.rglob("*truth*")) and not list(run.rglob("*.traj.csv"))
    return run


def test_a_twin_from_a_procedural_house_with_its_truth_absent_loads_in_mujoco(tmp_path):
    """[UdA]: house 95's recordings and their outputs alone make a MuJoCo house: walls and furniture as boxes, the floor
    textured with the floor picture, unmapped floor a flat marker, the robot's start in the house's frame"""
    import mujoco
    run = _run_without_truth(tmp_path)
    xml, man = ht.build(run, tmp_path / "twin", name="twin_t95")
    m = mujoco.MjModel.from_xml_path(str(xml))
    assert m.ngeom >= man["boxes"] > 50 and man["floor_seen_m2"] > 10
    assert 0 < man["floor_unmapped_share"] < 1
    assert not any("truth" in p or "traj" in p for v in man["from"].values() for p in (v if isinstance(v, list) else [v]))
    tex = (tmp_path / "twin" / "twin_t95_floor.png"); assert tex.exists()
    assert len(man["start_pos"]) == 2
    from dimos.simulation.mujoco import model as mm              # as DimOS's sim loads a room: the robot added, its assets
    assets = dict(mm.get_assets(), **{tex.name: tex.read_bytes()})
    full = mujoco.MjModel.from_xml_string(mm.get_model_xml("unitree_go1", Path(xml).read_text(), add_person=False), assets=assets)
    assert full.nbody > m.nbody and full.ngeom > m.ngeom


def _floor(shape, rng):
    """a tiled, speckled floor (edges to register on)"""
    h, w = shape
    tiles = np.repeat(np.repeat(rng.uniform(70, 190, (h // 30 + 1, w // 30 + 1, 1)), 30, 0), 30, 1)[:h, :w]
    return np.clip(tiles + rng.normal(0, 18, (h, w, 1)) * np.ones(3), 0, 255)


def test_the_floor_is_two_nights_registered_and_blended_without_a_seam(tmp_path):
    """[POG]: night 2's picture, 2 and 3 cm off night 1's and a tenth darker, is laid on night 1's patch by patch
    (floor_register.snap, as floor_truth2 scores) and blended: the texture matches the floor over both nights' floor"""
    rng = np.random.default_rng(3); T = _floor((300, 480), rng)
    s1 = np.zeros((300, 480), bool); s1[:, :300] = True
    s2 = np.zeros((300, 480), bool); s2[:, 180:] = True
    n2 = np.roll(T, (-2, -3), (0, 1)) * 0.9                 # night 2's [r, c] is the floor's [r + 2, c + 3]
    for k, (img, s) in enumerate([(T, s1), (n2, s2)], 1):
        np.savez(tmp_path / f"n{k}.npz", rgb=np.clip(img, 0, 255).astype(np.uint8), dist=np.where(s, 1.0, np.nan).astype(np.float16))
    rgb, seen, stats = ht.floor_texture([tmp_path / "n1.npz", tmp_path / "n2.npz"])
    assert stats[0]["patches"] >= 3 and stats[0]["shift_mm_median"] == pytest.approx(36.1, abs=1)
    inner = seen.copy(); inner[:, 460:] = False; inner[:10] = inner[-10:] = False   # the warp's edge
    err = np.abs(rgb.astype(float) - T)[inner].mean()
    raw = np.abs(np.where(s1[..., None], T, n2) - T)[inner].mean()
    assert err < 3 and err < raw / 3                          # within ~noise of the floor; unregistered: far off
    band = inner[:, 280:320]; assert np.abs(rgb[:, 280:320].astype(float) - T[:, 280:320])[band].mean() < 3   # no seam


def test_the_twin_is_lit_as_the_procedural_houses_are():
    """[TgM]: the twin's fixtures, bounce and headlight are the lit regime's (make_nights), not a kinder light; noise,
    flex and lag stay DimOS's sim's (the scene sets no options of its own)"""
    import make_house as mh
    import make_nights as mn
    mh.use_layout(70)
    light = mn.regime(70, 1, "lit"); proc = mn.lights_xml(light, mh.cell_to_world(*mh.START))
    want = re.search(r'name="ceiling0_0"[^>]*diffuse="([^"]+)"', proc).group(1)
    want_b = re.search(r'name="ceiling0_bounce"[^>]*diffuse="([^"]+)"', proc).group(1)
    seen = np.zeros((980, 1680), bool); seen[200:800, 200:1400] = True
    lights, (diff, amb) = ht.twin_lights(seen, 2.4)
    got = {l_.name: l_ for l_ in lights}
    assert " ".join(f"{c:.3f}" for c in got["ceiling0_0"].diffuse) == want
    assert " ".join(f"{c:.3f}" for c in got["ceiling0_bounce"].diffuse) == want_b
    assert got["ceiling0_0"].pos[2] == pytest.approx(mh.WALL_H - 0.05)
    tr, tg, tb = mn.TINT[light["tint"]]; assert amb == pytest.approx((light["ambient"] * tr, light["ambient"] * tg, light["ambient"] * tb))
    assert 2 <= len(lights) // 2 <= 6                          # ~33 m2 a fixture, as the open plan's five
    lib = ht.scene_lib(); xml = lib.build_scene([], None, name="t", lights=lights, headlight=(diff, amb)).xml
    assert "<option" not in xml and "<default" not in xml


def test_a_twin_is_written_under_data_only(tmp_path):
    """[au3]: a twin (its floor a picture of the house) is never written into the repo outside data/"""
    for bad in (HK / "sim" / "house" / "twin", HK / "docs" / "twin", HK / "twin"):
        with pytest.raises(SystemExit): ht.private_out(bad)
    assert ht.private_out(HK / "data" / "twin" / "x") == (HK / "data" / "twin" / "x").resolve()
    assert ht.private_out(tmp_path) == tmp_path.resolve()


def test_the_general_part_is_dimos_alone():
    """[4TQ]: the fork's scene_from_map holds nothing of houndkeeper"""
    if not FORK_MODULE.exists(): pytest.skip("the fork's twin branch is not checked out here")
    assert "houndkeeper" not in FORK_MODULE.read_text().lower()


def test_the_scorer_measures_footprint_and_faces(monkeypatch, tmp_path):
    """[JLx]: the twin's boxes against the plan's: the same footprint scores IoU 1 and no distance; moved 5 cm, its faces
    lie 5 cm from the plan's"""
    import twin_score as ts
    lib = ht.scene_lib(); shape = (980, 1680)
    T = np.zeros(shape, bool); T[400:500, 700:900] = True          # a 1 x 2 m block
    seen = np.zeros(shape, bool); seen[300:600, 600:1000] = True; seen &= ~T
    monkeypatch.setattr(ts.ff, "truth_structures", lambda seed, shp: T)
    monkeypatch.setattr(ts, "not_in_plan", lambda run, seed, shp: np.zeros(shp, bool))
    frame = (np.eye(2), np.zeros(2))
    def twin(dx):
        x0, x1 = -8.4 + 700 * 0.01 + dx, -8.4 + 900 * 0.01 + dx; y1, y0 = 4.9 - 400 * 0.01, 4.9 - 500 * 0.01
        return lib.build_scene([lib.Box(x0, y0, x1, y1, 0.0, 0.8)], name="t").write(tmp_path / f"t{dx}")
    g = ts.geometry(twin(0.0), seen, frame, 0, tmp_path)
    assert g["iou"] == pytest.approx(1.0, abs=0.01) and g["true_face_to_twin_cm"][1] == 0 and g["twin_face_to_true_cm"][1] == 0
    g = ts.geometry(twin(0.05), seen, frame, 0, tmp_path)
    assert g["true_face_to_twin_cm"][1] == pytest.approx(5, abs=1) and g["iou"] < 0.97
