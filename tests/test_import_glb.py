"""tools/import_glb.py ([CaM], [qEP]): an imported model stands on z = 0, centred on its footprint, at the size asked,
however far from the origin and at whatever scale the file drew it (2 Oct 2026: an origin taken at the wrong scale left
the bananas 7.7 m in the air and the chair 30 cm off)."""
import json, subprocess, sys
from pathlib import Path

import numpy as np
import pytest

trimesh = pytest.importorskip("trimesh")
TOOL = Path(__file__).resolve().parents[1] / "tools" / "import_glb.py"


def _bounds(obj):
    V = np.array([l.split()[1:4] for l in open(obj) if l.startswith("v ")], float)
    return V.min(0), V.max(0)


@pytest.mark.parametrize("flag,value", [("--height", 0.5), ("--length", 0.3)])
def test_origin_and_size(tmp_path, flag, value):
    box = trimesh.creation.box(extents=(4.0, 2.0, 6.0))          # glTF units, Y up: 2 tall, 6 deep, 4 wide
    box.apply_translation((10.0, 7.0, -3.0))                      # far from the origin
    glb = tmp_path / "thing.glb"; box.export(glb)
    subprocess.run([sys.executable, str(TOOL), str(glb), "--name", "thing", flag, str(value), "--source", "test",
                    "--author", "test", "--licence", "CC0", "--out", str(tmp_path / "out")], check=True, capture_output=True)
    lo, hi = _bounds(tmp_path / "out" / "thing" / "thing.obj")
    assert lo[2] == pytest.approx(0.0, abs=1e-6)                              # stands on the floor
    assert (lo[:2] + hi[:2]) / 2 == pytest.approx([0.0, 0.0], abs=1e-6)       # centred on its footprint
    size = hi - lo
    if flag == "--height": assert size[2] == pytest.approx(value, rel=1e-4)
    else: assert size.max() == pytest.approx(value, rel=1e-4)
    meta = json.loads((tmp_path / "out" / "thing" / "meta.json").read_text())
    assert meta["height_m"] == pytest.approx(size[2], abs=1e-3)
