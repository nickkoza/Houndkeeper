"""detect_changes2.main's blob logic on synthetic nights (the walk-through detector, frozen for the hold-out; [nv5] and
[FHA] use its test, [d8d] its finds): something here against something left, the small-and-faint drop, and fragments
within --link-px joined into one blob. Static: two synthetic floor pictures, no sim."""
import json
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house"))

H, W = 300, 400


def _floor(seed=11, tile=20):
    rng = np.random.default_rng(seed)
    base = np.repeat(np.repeat(rng.uniform(90, 170, (H // tile + 1, W // tile + 1, 3)), tile, 0), tile, 1)[:H, :W]
    return np.clip(base + rng.normal(0, 3, (H, W, 3)), 0, 255).astype(np.uint8)


def _night(run, n, rgb):
    occ = np.zeros((H, W), bool); occ[:4, :4] = True                      # one wall post every night (static)
    np.savez(run / f"mosaic_n{n}.npz", rgb=rgb, dist=np.ones((H, W), np.float16), occ=occ)
    (run / f"nights_s1_n{n}.truth.json").write_text(json.dumps({"items": [], "stains": []}))


def _detect(run, monkeypatch, *extra):
    import detect_changes2 as dc
    monkeypatch.setattr(sys, "argv", ["detect_changes2.py", "--seed", "1", "--nights", "2", "--run-dir", str(run), *extra])
    dc.main()
    return json.loads((run / "detect_n2.json").read_text())["blobs"]


def _at(blobs, r, c):
    """the blobs whose box holds (r, c)"""
    return [b for b in blobs if b["bbox_rc"][0] <= r < b["bbox_rc"][2] and b["bbox_rc"][1] <= c < b["bbox_rc"][3]]


def test_a_toy_put_down_is_something_here_and_one_taken_away_is_something_left(tmp_path, monkeypatch):
    """[d8d] a red square on tonight's floor only is "present"; one on the baseline only is "gone"; neither is a
    floor change, and nothing else on the unchanged floor is raised"""
    T = _floor(); a, b = T.copy(), T.copy()
    b[100:108, 100:108] = (210, 30, 30)                                    # put down tonight
    a[180:188, 280:288] = (30, 30, 210)                                    # there last night, gone tonight
    _night(tmp_path, 1, a); _night(tmp_path, 2, b)
    blobs = _detect(tmp_path, monkeypatch)
    here, left = _at(blobs, 104, 104), _at(blobs, 184, 284)
    assert [x["kind"] for x in here] == ["present"] and [x["kind"] for x in left] == ["gone"], blobs
    assert len(blobs) == 2, blobs


def _speck(img, r, c, h, w, k):
    """a patch of the floor tinted by k (0..1 towards red): faint for small k"""
    p = img[r:r + h, c:c + w].astype(np.float32); img[r:r + h, c:c + w] = np.clip(p * (1 - k) + np.array([230, 20, 20]) * k, 0, 255)


def test_a_speck_tiny_and_faint_is_dropped_but_tiny_and_strong_or_larger_and_faint_is_kept(tmp_path, monkeypatch):
    """[d8d] the --small-px / --small-peak drop needs both: under 8 cm2 AND a peak under 0.45 (registration speckle);
    a strong speck of the same size, or a faint patch over 8 cm2, stays"""
    T = _floor(); b = T.copy()
    _speck(b, 80, 80, 2, 3, 0.3)                                           # tiny and faint
    _speck(b, 80, 200, 2, 3, 1.0)                                          # tiny and strong
    _speck(b, 200, 120, 4, 4, 0.3)                                         # 16 cm2, faint
    _night(tmp_path, 1, T); _night(tmp_path, 2, b)
    blobs = _detect(tmp_path, monkeypatch)
    small = {x["id"]: x for x in blobs}
    faint_tiny, strong_tiny, faint_big = _at(blobs, 81, 81), _at(blobs, 81, 201), _at(blobs, 201, 121)
    assert faint_tiny == [] and len(strong_tiny) == 1 and len(faint_big) == 1, blobs
    assert strong_tiny[0]["area_cm2"] < 8 and strong_tiny[0]["peak"] >= 0.45
    assert faint_big[0]["area_cm2"] >= 8 and faint_big[0]["peak"] < 0.45
    # the same picture without the drop keeps the faint speck: the drop, not some other rule, removed it
    blobs0 = _detect(tmp_path, monkeypatch, "--small-px", "0")
    assert len(_at(blobs0, 81, 81)) == 1 and _at(blobs0, 81, 81)[0]["peak"] < 0.45
    assert len(small) == 2


def test_fragments_of_a_string_within_link_px_are_one_blob(tmp_path, monkeypatch):
    """[d8d] a thin string seen as beads 3 cm apart is one find with --link-px 2 (default), not one find per bead"""
    T = _floor(); b = T.copy()
    for k in range(6): b[150:152, 150 + 5 * k:152 + 5 * k] = (220, 30, 30)        # 2x2 beads, 3 cm apart in a line
    _night(tmp_path, 1, T); _night(tmp_path, 2, b)
    joined = _detect(tmp_path, monkeypatch)
    apart = _detect(tmp_path, monkeypatch, "--link-px", "0")
    assert len(joined) == 1 and joined[0]["kind"] == "present", joined
    assert len(apart) > 1, apart


def test_the_detector_reads_no_truth_and_its_scorer_matches_its_blobs(tmp_path, monkeypatch):
    """[1B2], [d8d]: detect_changes2 alone finds the same blobs with the nights' truth files gone, and marks none of them
    matched; score_detect2 (the scorer) runs it and matches the toy put down with the truth's item"""
    T = _floor(); b = T.copy(); b[100:108, 100:108] = (210, 30, 30)
    _night(tmp_path, 1, T); _night(tmp_path, 2, b)
    for f in tmp_path.glob("*.truth.json"): f.unlink()
    alone = _detect(tmp_path, monkeypatch)
    assert [x["kind"] for x in alone] == ["present"] and "matched" not in alone[0]
    assert not (tmp_path / "detect_results.json").exists() and not (tmp_path / "items_n2.json").exists()
    x, y = -8.4 + 104 * 0.01, 4.9 - 104 * 0.01                               # the grid's own frame (w2rc)
    for n, items in ((1, []), (2, [{"id": "t", "kind": "toy", "x": x, "y": y, "half": [0.04, 0.04]}])):
        (tmp_path / f"nights_s1_n{n}.truth.json").write_text(json.dumps({"items": items, "stains": []}))
    import score_detect2 as sd
    sd.main(["--seed", "1", "--nights", "2", "--run-dir", str(tmp_path)])
    scored = json.loads((tmp_path / "detect_n2.json").read_text())["blobs"]
    assert [{k: v for k, v in s.items() if k != "matched"} for s in scored] == alone and scored[0]["matched"] is True
    res = json.loads((tmp_path / "detect_results.json").read_text())["2"]
    assert res["per_kind"] == {"toy": {"found": 1, "missed": 0, "unobserved": 0}} and res["false_alarms"] == 0
