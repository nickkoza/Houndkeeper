"""[5SS]: every run records its speed (sim/wall ratio and frame interval), and a run too slow to trust says so: until
[3Pv] is measured, numbers from a run under 0.9 sim/wall are marked latency-flattered wherever they are summarised."""
import json
import subprocess
import sys
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "dimos"))
import round_summary  # noqa: E402
import run_speed  # noqa: E402

PY = sys.executable
NEW = ("SIM_SPEED sim/wall = 0.30 (target max); frames every 233 ms wall, 70 ms robot; time in: physics 20%, hook 4%, "
       "viewer 1%, cameras 60%, lidar 15% of 30 s\n")
OLD = "SIM_SPEED sim/wall = 0.18 (target max); time in: physics 20%, hook 4%, viewer 1%, cameras 60%, lidar 15% of 30 s\n"


def _traj(p, ratio, host=True, n=50):
    rows = ["wall_t,sim_t,true_x,true_y,true_yaw,odom_x,odom_y,odom_yaw" + (",host_t" if host else "")]
    for k in range(n):
        sim = k * 0.1
        stamp = 1000 + sim if host else 1000 + sim / ratio                  # since [3Pv] wall_t is the robot's clock
        rows.append(f"{stamp},{sim},0,0,0,0,0,0" + (f",{1000 + sim / ratio}" if host else ""))
    p.write_text("\n".join(rows) + "\n")


def test_the_speed_is_read_from_the_sims_own_report(tmp_path):
    """[5SS]"""
    (tmp_path / "dimos.log").write_text("starting\n" + OLD + NEW)
    sp = run_speed.of_run(tmp_path)
    assert sp["sim_wall"] == pytest.approx(0.30) and sp["frame_interval_ms"] == {"wall": 233.0, "sim": 70.0, "derived": False}
    assert sp["latency_flattered"] is True


def test_an_older_report_gives_the_frame_interval_derived(tmp_path):
    """[5SS]: before the sim reported frames, the interval follows from the ratio (a frame every two 35 ms loops)"""
    (tmp_path / "dimos.log").write_text(OLD)
    sp = run_speed.of_run(tmp_path)
    assert sp["sim_wall"] == pytest.approx(0.18) and sp["frame_interval_ms"]["derived"]
    assert sp["frame_interval_ms"]["wall"] == pytest.approx(70 / 0.18, rel=0.01)


@pytest.mark.parametrize("host", [True, False])
def test_without_a_report_the_path_file_gives_the_ratio(tmp_path, host):
    """[5SS]: traj.csv's host_t (since [3Pv]) or, before, its wall_t against sim_t"""
    _traj(tmp_path / "traj.csv", 0.25, host=host)
    assert run_speed.of_run(tmp_path)["sim_wall"] == pytest.approx(0.25, rel=0.01)


def test_a_real_time_run_is_not_flagged_and_an_unknown_one_says_so(tmp_path):
    """[5SS]"""
    (tmp_path / "dimos.log").write_text(OLD.replace("0.18", "0.97"))
    assert run_speed.of_run(tmp_path)["latency_flattered"] is False and run_speed.flag(run_speed.of_run(tmp_path)) == ""
    empty = run_speed.of_run(tmp_path / "nothing")
    assert empty["sim_wall"] is None and "speed not recorded" in run_speed.flag(empty)
    assert "latency-flattered" in run_speed.flag({"sim_wall": 0.5, "latency_flattered": True})


def test_a_round_summary_marks_a_round_with_a_slow_night(tmp_path):
    """[5SS]: one slow night marks the round's numbers"""
    for i, r in enumerate((1.0, 0.4)):
        (tmp_path / f"r9_s70_i{i}.score.json").write_text(json.dumps({"found": 1, "items": 1, "speed": {"sim_wall": r, "latency_flattered": r < 0.9}}))
    s = round_summary.summarize(tmp_path, 9)
    assert s["speed"] == {"nights": 2, "flattered": 1, "unknown": 0, "sim_wall_min": 0.4}
    out = subprocess.run([PY, str(HK / "sim" / "dimos" / "round_summary.py"), "9", "--dir", str(tmp_path), "--results-only"],
                         capture_output=True, text=True, check=True).stdout
    assert "latency-flattered" in out


def test_the_vslice_summary_marks_slow_nights(tmp_path):
    """[5SS]"""
    base = {"items": 2, "binned": 1, "found": 2, "stops": 2, "stops_for_nothing": 0, "outcomes": {}, "state": "done", "error": None, "t_s": 60}
    for name, r in (("a", 0.95), ("b", 0.3)):
        (tmp_path / name).mkdir(); (tmp_path / name / "score.json").write_text(json.dumps(base | {"speed": {"sim_wall": r, "latency_flattered": r < 0.9}}))
    out = subprocess.run([PY, str(HK / "sim" / "dimos" / "vslice_summary.py"), str(tmp_path / "a"), str(tmp_path / "b")],
                         capture_output=True, text=True, check=True).stdout
    lines = out.splitlines()
    assert "latency-flattered" not in lines[0] and "latency-flattered" in lines[1]
    assert "1 of 2 nights latency-flattered" in out


def test_a_detect_runs_nights_carry_their_speed(tmp_path):
    """[5SS]: the sweep nights' logs"""
    run = tmp_path / "nights_s91"; run.mkdir()
    (run / "nights_s91_n2.sweep.log").write_text(OLD)
    (run / "nights_s91_n3.sweep.log").write_text(NEW.replace("0.30", "0.95"))
    sp = run_speed.of_nights(run, [2, 3, 4])
    assert sp == {"nights": 3, "flattered": 1, "unknown": 1, "sim_wall_min": 0.18}
    assert "latency-flattered" in run_speed.flag_many(sp)
