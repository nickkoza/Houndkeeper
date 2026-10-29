"""The detection bench stops on a night it cannot replay instead of scoring it as no finds ([SdA])."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "house"))
import bench_detect as bd


def test_a_nights_own_files_are_put_in():
    """[SdA]"""
    n = {"run": "s70_t1", "traj": "a.csv", "corrections": "c.npz", "views": ["v1.npz", "v2.npz"]}
    assert bd.confirmer_args(n, ["--truth", "@traj", "--views", "@views", "--tc", "0.04"]) == \
        ["--truth", "a.csv", "--views", "v1.npz", "v2.npz", "--tc", "0.04"]


@pytest.mark.parametrize("ph", ["@traj", "@corrections", "@mount", "@views"])
def test_a_placeholder_the_night_lacks_stops_the_bench(ph):
    """[SdA]: passed on literally, the confirmer crashed on it and the night scored zero"""
    with pytest.raises(SystemExit, match=ph):
        bd.confirmer_args({"run": "s70_t1"}, ["--x", ph])


def test_a_confirmer_crash_stops_the_bench_and_keeps_its_stderr(tmp_path):
    """[SdA]: a crash is not a quiet night"""
    out, err = tmp_path / "out.json", tmp_path / "err.txt"
    with pytest.raises(SystemExit, match="exit 3"):
        bd.replay([sys.executable, "-c", "import sys; sys.stderr.write('boom\\n'); sys.exit(3)"], out, err, cwd=tmp_path)
    assert "boom" in err.read_text()


def test_a_confirmer_that_exits_0_without_a_result_stops_the_bench(tmp_path):
    """[SdA]"""
    with pytest.raises(SystemExit, match="no result"):
        bd.replay([sys.executable, "-c", "pass"], tmp_path / "out.json", tmp_path / "err.txt", cwd=tmp_path)


def test_a_good_replay_returns_its_result(tmp_path):
    """[SdA]"""
    out = tmp_path / "out.json"
    code = f"import json; json.dump({{'decisions': []}}, open({str(out)!r}, 'w'))"
    assert bd.replay([sys.executable, "-c", code], out, tmp_path / "err.txt", cwd=tmp_path) == {"decisions": []}
