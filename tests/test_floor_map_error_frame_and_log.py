"""[nv5] the floor map error beyond the map_error helper (docs/research/test-quality-2026-10-03.md, "tests still to add" 8):
the map laid on the true floor by its path's rigid fit (corrections included), a map that is already in the true frame
judged where it lies (the true-path reference), the dated log line `main --error` appends, and check_funnel_top's call
that labels each bench house's measurement. Static: synthetic floors, no sim."""
import csv
import json
import math
import re
import sys
import time
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def _floor(seed=7, h=300, w=400, tile=20):
    rng = np.random.default_rng(seed)
    base = np.repeat(np.repeat(rng.uniform(60, 200, (h // tile + 1, w // tile + 1, 3)), tile, 0), tile, 1)[:h, :w]
    return np.clip(base + rng.normal(0, 4, (h, w, 3)), 0, 255).astype(np.uint8)


def _rot(deg):
    th = math.radians(deg); return np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])


def _traj(path, odom, true):
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["wall_t", "odom_x", "odom_y", "true_x", "true_y"])
        for k, ((ox, oy), (tx, ty)) in enumerate(zip(odom, true)): w.writerow([k * 0.1, ox, oy, tx, ty])


def _walk(n=200):
    s = np.linspace(0, 2 * math.pi, n); return np.c_[1.1 * np.cos(s), 0.7 * np.sin(2 * s)]


def _no_structures(ff, fa, monkeypatch, T):
    occ = np.zeros(T.shape[:2], bool); occ[:4, :4] = True                 # one wall cell in a corner: the band is tiny
    monkeypatch.setattr(ff, "truth", lambda seed, night, shape: T.astype(np.float32))
    monkeypatch.setattr(ff, "truth_structures", lambda seed, shape: occ)
    monkeypatch.setattr(fa, "regions", lambda run, seed, nights, shape: (np.zeros(shape, bool), np.zeros(shape, bool)))


def test_the_paths_corrections_are_applied_before_the_rigid_fit(tmp_path):
    """[nv5] the map is laid by the move that lays its *corrected* path on the true path: with the night's corrections
    (a 4 deg turn and a 0.2 m shift of odometry), the frame found is the one from the corrected path, not from raw
    odometry"""
    import floor_fidelity as ff
    odom = _walk(); C = np.eye(3); C[:2, :2] = _rot(4.0); C[:2, 2] = [0.2, -0.1]
    corrected = odom @ C[:2, :2].T + C[:2, 2]
    R, t = _rot(-2.5), np.array([0.05, 0.11])
    _traj(tmp_path / "traj.csv", odom, corrected @ R.T + t)
    times = [0.0, 100.0]; np.savez(tmp_path / "corr.npz", times=np.array(times), Ts=np.stack([C, C]))
    R_, t_ = ff.frame_from_path(str(tmp_path / "corr.npz"), tmp_path / "traj.csv")
    assert np.allclose(R_, R, atol=1e-6) and np.allclose(t_, t, atol=1e-6)


def test_a_map_in_odometrys_turned_frame_is_laid_back_by_its_path_and_scores_low(tmp_path, monkeypatch):
    """[nv5] error_of lays the map by its path's rigid fit: a perfect map painted in a frame 3 deg and 12 cm off the
    sim's scores near zero once laid; the same map judged where it sits (its path saying no offset) scores high"""
    import floor_fidelity as ff, floor_agreement as fa
    T = _floor(); H, W = T.shape[:2]; _no_structures(ff, fa, monkeypatch, T)
    R, t = _rot(3.0), np.array([0.12, -0.08])
    pic = ff.to_true_frame(T.astype(np.float32), R.T, -R.T @ t)          # the floor as seen in the turned frame
    np.savez(tmp_path / "pic.npz", rgb=pic, dist=np.ones((H, W), np.float16))
    odom = _walk()
    _traj(tmp_path / "laid.csv", odom, odom @ R.T + t)
    _traj(tmp_path / "flat.csv", odom, odom)
    good, m2 = ff.error_of(tmp_path / "pic.npz", 1, 1, tmp_path / "laid.csv", None, tmp_path)
    bad, _ = ff.error_of(tmp_path / "pic.npz", 1, 1, tmp_path / "flat.csv", None, tmp_path)
    assert good < 0.03 and bad > 0.2 and m2 > 8.0, (good, bad, m2)


def test_a_bent_map_counts_against_it(tmp_path, monkeypatch):
    """[nv5] one rigid move for the whole map, so a bend is not forgiven: half the map slid 8 cm (beyond the
    confirmer's slack) while its path fits exactly is error wherever the slide shows"""
    import floor_fidelity as ff, floor_agreement as fa
    T = _floor(); H, W = T.shape[:2]; _no_structures(ff, fa, monkeypatch, T)
    pic = T.copy(); pic[:, W // 2:] = np.roll(T, 8, 1)[:, W // 2:]
    np.savez(tmp_path / "pic.npz", rgb=pic, dist=np.ones((H, W), np.float16))
    odom = _walk(); _traj(tmp_path / "traj.csv", odom, odom)
    err, _ = ff.error_of(tmp_path / "pic.npz", 1, 1, tmp_path / "traj.csv", None, tmp_path)
    assert 0.08 < err < 0.5, err                    # (on 20 cm tiles an 8 cm slide differs within ~6 cm of each tile
                                                    # edge past the slack: ~30% of the slid half, ~15% of the map)


def test_a_map_built_on_the_true_path_is_judged_where_it_lies(tmp_path, monkeypatch):
    """[nv5] the true-path reference: a map already in the sim's frame, scored with a path whose odometry is the true
    path, is laid by the identity and scores as the floor itself (near zero on an unlit synthetic floor)"""
    import floor_fidelity as ff, floor_agreement as fa
    T = _floor(); H, W = T.shape[:2]; _no_structures(ff, fa, monkeypatch, T)
    np.savez(tmp_path / "pic.npz", rgb=T, dist=np.ones((H, W), np.float16))
    odom = _walk(); _traj(tmp_path / "traj.csv", odom, odom)
    R, t = ff.frame_from_path(None, tmp_path / "traj.csv")
    assert np.allclose(R, np.eye(2), atol=1e-9) and np.allclose(t, 0, atol=1e-9)
    err, _ = ff.error_of(tmp_path / "pic.npz", 1, 1, tmp_path / "traj.csv", None, tmp_path)
    assert err < 0.005, err


def _run_main(ff, monkeypatch, capsys, argv):
    monkeypatch.setattr(sys, "argv", ["floor_fidelity.py"] + argv); ff.main(); return capsys.readouterr().out


def test_each_measurement_is_appended_to_the_log_with_its_date_and_how_the_map_was_built(tmp_path, monkeypatch, capsys):
    """[nv5] `--error` appends one JSON line per measurement (date, house, night, how built, picture, error, m2 judged)
    and keeps what the log already held"""
    import floor_fidelity as ff
    monkeypatch.setattr(ff, "error_of", lambda picture, seed, night, traj, corr, run: (0.0123456, 42.04))
    log = tmp_path / "eval" / "floor_map_error.jsonl"; log.parent.mkdir(); log.write_text('{"old": 1}\n')
    before = time.strftime("%Y-%m-%dT%H:%M")
    out = _run_main(ff, monkeypatch, capsys, ["p1.npz", "--seed", "73", "--night", "2", "--traj", "t.csv", "--error", "run",
                                              "--built", "pose graph", "--log", str(log)])
    _run_main(ff, monkeypatch, capsys, ["p2.npz", "--seed", "70", "--traj", "t.csv", "--error", "run", "--built", "true path",
                                        "--log", str(log)])
    after = time.strftime("%Y-%m-%dT%H:%M")
    lines = log.read_text().splitlines()
    assert len(lines) == 3 and json.loads(lines[0]) == {"old": 1}
    a, b = json.loads(lines[1]), json.loads(lines[2])
    assert re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}", a["date"]) and before <= a["date"] <= after
    assert (a["seed"], a["night"], a["built"], a["picture"], a["error"], a["judged_m2"]) == (73, 2, "pose graph", "p1.npz", 0.01235, 42.0)
    assert (b["seed"], b["night"], b["built"], b["picture"]) == (70, 1, "true path", "p2.npz")
    assert "floor map error 1.23% of 42.0 m2" in out and "(pose graph)" in out


def test_the_log_goes_to_data_eval_by_default():
    """[nv5] kept in data/eval/floor_map_error.jsonl unless told otherwise"""
    import floor_fidelity as ff
    import argparse
    seen = {}
    real = argparse.ArgumentParser.parse_args
    def grab(self, args=None, namespace=None):
        ns = real(self, ["p.npz", "--seed", "1"]); seen.update(vars(ns)); raise SystemExit(0)
    try:
        argparse.ArgumentParser.parse_args = grab
        with pytest.raises(SystemExit): ff.main()
    finally:
        argparse.ArgumentParser.parse_args = real
    assert Path(seen["log"]) == HK / "data" / "eval" / "floor_map_error.jsonl"


def test_the_funnel_check_measures_each_bench_house_from_its_path_and_labels_the_bench(tmp_path, monkeypatch, capsys):
    """[nv5] check_funnel_top scores every bench house that has a night-1 map and true path, with `--error` (so it is
    logged), the path, and the bench's name as how the map was built; and reports the mean"""
    import check_funnel_top as cf
    bench = tmp_path / "houses6c"
    for s in (70, 73):
        d = bench / f"nights_s{s}"; d.mkdir(parents=True); (d / "mosaic_n1.npz").write_bytes(b""); (d / f"nights_s{s}_n1.traj.csv").write_text("")
    (bench / "nights_s77").mkdir()                                          # no map: skipped
    calls = []
    def fake(args):
        calls.append(args)
        if args[0] == "floor_fidelity.py": return f"s: floor map error {1.0 if args[3] == '70' else 3.0:.2f}% of 50 m2"
        return ""
    monkeypatch.setattr(cf, "run", fake)
    monkeypatch.setattr(sys, "argv", ["check_funnel_top.py", "--map-error", str(bench)])
    with pytest.raises(SystemExit): cf.main()
    ff_calls = [c for c in calls if c[0] == "floor_fidelity.py"]
    assert len(ff_calls) == 2
    for c in ff_calls:
        seed = c[c.index("--seed") + 1]; run = Path(c[c.index("--error") + 1])
        assert run == bench / f"nights_s{seed}" and Path(c[1]) == run / "mosaic_n1.npz"
        assert Path(c[c.index("--traj") + 1]) == run / f"nights_s{seed}_n1.traj.csv"
        assert c[c.index("--built") + 1] == "bench houses6c"
    assert "2.00% mean; house 70 1.00%, house 73 3.00%" in capsys.readouterr().out
