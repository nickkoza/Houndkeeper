"""[p2t]: a DimOS night asked to run headless (DIMOS_SIM_HEADLESS=1) starts its sim without the viewer. The stand-in viewer
once failed to construct (MjvScene needs the model on MuJoCo 3.10), so every headless night sat at "starting" (26 Sep 2026)."""
import sys
from pathlib import Path

from houndkeeper import workspace
import pytest

mujoco = pytest.importorskip("mujoco")
DIMOS = workspace.dimos()


def test_headless_viewer_stands_in():
    sys.path.insert(0, str(DIMOS))
    mp = pytest.importorskip("dimos.simulation.mujoco.mujoco_process")
    m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom size="1"/></worldbody></mujoco>')
    with mp._HeadlessViewer(m) as v:
        assert v.is_running()
        v.sync()
        with v.lock():
            assert v.user_scn.maxgeom > 0
