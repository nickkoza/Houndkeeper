"""HK_MUJOFIL_DIR chooses the mujofil build for every test, not only the ones that go through filament_cameras
(waymark 0007 [3my]): the Filament tests import mujofil when they load, and a module is imported once, so the choice
has to be made before any of them loads (28 Sep: they ran on the installed build while a new one was under test)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "houndkeeper"))


def pytest_configure(config):
    import filament_cameras
    filament_cameras.use_chosen_mujofil()
