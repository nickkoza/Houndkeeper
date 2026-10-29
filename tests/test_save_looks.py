"""[JKS]: every close look is saved as taken (image, camera pose, robot pixels, aim, what the detector found), so a miss
("not seen at the flag") can be replayed offline."""
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))


def test_looks_round_trip(tmp_path):
    import pick
    img = np.random.default_rng(0).integers(0, 255, (48, 64, 3), dtype=np.uint8)
    cv = {"img": img, "c": np.array([1.0, 2.0, 0.4]), "R": np.eye(3), "fovy": 58.0, "robot": np.zeros((48, 64), bool), "aim": np.array([1.2, 2.1])}
    S = SimpleNamespace(last_looks=[(dict(cv, wide=True), None), (cv, {"xy": np.array([1.21, 2.09]), "label": "a toy animal"})])
    pick.save_looks(S, tmp_path / "item3_look1.npz", ["wide look: nothing", "a toy animal"])
    z = np.load(tmp_path / "item3_look1.npz")
    assert (z["0_img"] == img).all() and bool(z["0_wide"]) and "0_found_xy" not in z.files
    assert np.allclose(z["1_found_xy"], [1.21, 2.09]) and str(z["1_label"]) == "a toy animal"
    assert list(z["notes"]) == ["wide look: nothing", "a toy animal"]
