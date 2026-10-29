"""A night can choose which mujofil build draws its cameras (waymark 0007 [3my]): a fixed renderer is installed beside
the one the running rounds use, not over it, and HK_MUJOFIL_DIR picks it."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "houndkeeper"))


def test_the_chosen_folder_goes_first_on_the_path(tmp_path, monkeypatch):
    import filament_cameras as fc
    monkeypatch.setenv("HK_MUJOFIL_DIR", str(tmp_path)); monkeypatch.setattr(sys, "path", list(sys.path))
    fc.use_chosen_mujofil(); assert sys.path[0] == str(tmp_path)
    fc.use_chosen_mujofil(); assert sys.path.count(str(tmp_path)) == 1      # once, however often it's asked


def test_without_the_switch_the_build_that_draws_every_lamp_is_used(monkeypatch):
    """since 30 Sep the fixed build is the default (waymark 0007 [3my]): opt-in let runs fall back to one that drops lamps"""
    import filament_cameras as fc
    monkeypatch.delenv("HK_MUJOFIL_DIR", raising=False)
    fc.use_chosen_mujofil(); assert sys.path[0] == str(fc.FIXED_MUJOFIL)


def test_the_render_stamp_names_the_build_and_the_default_stamps_as_the_fixed_build(monkeypatch):
    """baselines rendered by one mujofil build aren't compared with nights rendered by another; the default stamps the
    same as the rounds that chose the fixed build by name, so their baselines stand"""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from houndkeeper import sim_hook as sh
    monkeypatch.delenv("HK_MUJOFIL_DIR", raising=False); base = sh.render_settings()
    assert base["mujofil"] == "mujofil-f177-site"
    monkeypatch.setenv("HK_MUJOFIL_DIR", "/x/mujofil-f177-site"); assert sh.render_settings() == base
    monkeypatch.setenv("HK_MUJOFIL_DIR", "/x/mujofil-other"); assert sh.render_settings()["mujofil"] == "mujofil-other"


def test_a_renderer_is_never_reused_for_another_model(monkeypatch):
    """caches keyed by a model's id() hand a new model an old one's renderer when its address is reused (29 Sep: the same
    fault scrambled every toy's measured size)"""
    import filament_cameras as fc
    made = []
    class Stub:
        def __init__(self, model, data, w, h): self.m = model; made.append(model)
        def render(self, data, cid): return self.m
    monkeypatch.setattr(fc, "FilamentCameras", Stub); monkeypatch.setattr(fc, "_BY_SIZE", {})
    old, new = object(), object()
    fc._BY_SIZE[(id(new), 8, 8)] = Stub(old, None, 8, 8)                   # a stale entry at the new model's address
    assert fc.render(new, None, 0, 8, 8) is new and made[-1] is new
