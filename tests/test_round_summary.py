"""A round's nights summed the way the hold-out is judged (sim-slice [bOu], [bnO]): grippable toys by binned, too-big
toys by reported, never mixed; stops for nothing; and why each grippable toy that wasn't binned was lost."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "dimos"))


def _night(path, per_item, stops_nothing, t_s, found, pct_used=None):
    sc = {"per_item": per_item, "stops_for_nothing": stops_nothing, "t_s": t_s, "found": found,
          "items": len(per_item), "stops": stops_nothing + 2}
    if pct_used is not None: sc["energy"] = {"air_left_pct": 100 - pct_used, "air_used_Wh": pct_used * 2.5, "air_Wh": 250.0}
    path.write_text(json.dumps(sc))


def test_a_round_is_summed_by_kind(tmp_path):
    import round_summary as rs
    _night(tmp_path / "r7_s70_i1.score.json", [
        {"item": "a", "tier": "easy", "binned": True, "said_too_wide": False, "handled": True},
        {"item": "b", "tier": "hard", "binned": False, "said_too_wide": True, "handled": True},
        {"item": "c", "tier": "too_big", "binned": False, "said_too_wide": True, "handled": True}], 10, 600.0, 3, 20.0)
    _night(tmp_path / "r7_s70_i2.score.json", [
        {"item": "d", "tier": "easy", "binned": False, "said_too_wide": False, "handled": True},
        {"item": "e", "tier": "easy", "binned": False, "said_too_wide": False, "handled": False},
        {"item": "f", "tier": "too_big", "binned": False, "said_too_wide": False, "handled": False}], 20, 1200.0, 1, 40.0)
    s = rs.summarize(tmp_path, 7)
    assert s["nights"] == 2 and s["grippable"] == 4 and s["binned"] == 1
    assert s["easy"] == [1, 3] and s["hard"] == [0, 1]                               # binned of each tier ([QyZ])
    assert s["too_big"] == 2 and s["too_big_reported"] == 1 and s["too_big_binned"] == 0
    assert s["lost"] == {"refused as too wide": 1, "handled, not binned": 1, "never handled": 1}
    assert s["stops_for_nothing_per_night"] == 15.0 and s["worst_stops_for_nothing"] == 20
    assert s["pack_used_pct_per_night"] == 30.0 and s["worst_pack_used_pct"] == 40.0     # within a single charge ([bnO])


def test_the_hold_out_is_numbers_only(tmp_path, capsys, monkeypatch):
    """the hold-out's report says how it did, never why (no peeking; sim-slice [bnO]), and
    its files are named apart from the tuning rounds'"""
    import round_summary as rs
    _night(tmp_path / "h1_s80_i1.score.json", [
        {"item": "a", "tier": "easy", "binned": False, "said_too_wide": True, "handled": True}], 3, 600.0, 1, 30.0)
    monkeypatch.setattr(sys, "argv", ["round_summary.py", "1", "--prefix", "h", "--results-only", "--dir", str(tmp_path)])
    rs.main(); out = capsys.readouterr().out
    assert "0/1" in out and "refused" not in out and "lost" not in out


def test_the_gate_before_the_hold_out(tmp_path):
    """round 12 on the tuning houses is judged by the hold-out's own bar before the hold-out runs (the
    hold-out proceeds overnight only if round 12 passes): every night scored, 78% found (7 of 9), half the
    grippable toys binned, every night ending with a quarter of the pack; too-big toys are reported, not gated"""
    import round_summary as rs
    good = [{"item": "a", "tier": "easy", "binned": True, "said_too_wide": False, "handled": True},
            {"item": "b", "tier": "easy", "binned": False, "said_too_wide": False, "handled": False}]
    for i in (1, 2): _night(tmp_path / f"r12_s70_i{i}.score.json", good, 40, 2400.0, 2, 60.0)
    ok, why = rs.gate(rs.summarize(tmp_path, 12), nights=2)
    assert ok and why == []
    _night(tmp_path / "r12_s70_i2.score.json", good, 40, 2400.0, 2, 80.0)                # a night down to 20% left
    ok, why = rs.gate(rs.summarize(tmp_path, 12), nights=2)
    assert not ok and any("pack" in w for w in why)
    ok, why = rs.gate(rs.summarize(tmp_path, 12), nights=3)
    assert not ok and any("nights" in w for w in why)
