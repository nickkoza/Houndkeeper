"""Each grasp try's own outcome is reported, not only the last (sim-slice [bOu]): on 28 Sep a pony lifted by a wing
and dropped showed only "not seen again after a failed try", and the drop had to be guessed from the state log."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))
import pick  # noqa: E402


class _S:
    pass


def test_every_try_says_why_it_failed(monkeypatch):
    outcomes = iter([("dropped lifting", 0.004), (None, 0.003)])
    def fake(S, ref, q_ready, q_bin):
        S.jaw_used = np.array([1.0, 0.0]); S.grip_width_used_cm = 6.9; S.lean_used_deg = 12.0; S.weighed_kg = 0.2
        return next(outcomes)
    monkeypatch.setattr(pick, "grasp_to_bin", fake)
    ref = {"xy": np.zeros(2), "fits_jaws": True, "grip_width_cm": 6.9}
    S = _S(); fail, _ = pick.pick_with_tries(S, ref, None, None, relook=lambda xy: (dict(ref), ["a toy"]))
    assert fail is None and [t["fail"] for t in S.try_log] == ["dropped lifting", None]
    assert S.try_log[0]["width_cm"] == 6.9 and S.try_log[0]["lean_deg"] == 12.0 and S.try_log[0]["jaw"] == [1.0, 0.0]
