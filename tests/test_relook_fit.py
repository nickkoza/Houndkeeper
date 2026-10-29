"""[m62]'s retries: a re-look after a failed grasp that calls the item too wide, after an earlier look found it fits,
doesn't end the stop: the item didn't change size (28 Sep: a figure fitting by one leg, refused on a re-look)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))
import pick  # noqa: E402


class _S:
    jaw_used = None


def test_a_relook_does_not_reverse_the_fit(monkeypatch):
    seen = []
    def grasp(S, ref, q_ready, q_bin):
        seen.append(ref["fits_jaws"]); return ("grasp (not lifted)", None) if len(seen) == 1 else (None, 0.03)
    monkeypatch.setattr(pick, "grasp_to_bin", grasp)
    first = {"xy": np.zeros(2), "fits_jaws": True, "grip_options": [(0.04, np.array([1.0, 0.0]), np.zeros(2))], "grip_width_cm": 4.0}
    wide = {"xy": np.array([0.01, 0.0]), "fits_jaws": False, "grip_width_cm": 13.0}
    S = _S(); fail, _ = pick.pick_with_tries(S, first, None, None, relook=lambda xy: (dict(wide), ["a toy", "too wide at its height too"]))
    assert fail is None and seen == [True, True] and "kept the earlier look's fit" in S.relook_notes[0]
