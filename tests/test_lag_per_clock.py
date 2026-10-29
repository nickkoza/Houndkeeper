"""[gJq], decision [55G]: the robot's frame lag is stored per clock, and every tool that places frames with it takes the
lag of the clock its recording was stamped on, read from the sim's path file (host_t since the robot's clock, [3Pv]),
or refuses: a file without that clock's lag is never read for another clock's."""
import ast
import inspect
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src")); sys.path.insert(0, str(ROOT / "sim" / "house"))
from houndkeeper import calibrate as cal  # noqa: E402

GOOD = dict(lag_s=0.14, corr=0.85, pairs=900, turning=400, edge=False)
WALL = {"lag_s": 0.135, "gate": {"lo": 0.0}, "streams": {"color_image": {"lag_s": 0.135}, "arm_color_image": {"lag_s": 0.155}}}
ROBOT = {"lag_s": 0.02, "streams": {"color_image": {"lag_s": 0.02}}}
PER_CLOCK = {"clocks": {"wall": WALL, "robot": ROBOT}}


def _write(p, d): p.write_text(json.dumps(d)); return p


def test_each_clock_has_its_own_lag(tmp_path):
    p = _write(tmp_path / "frame_lag.json", PER_CLOCK)
    assert cal.load_lag(p, "wall")["lag_s"] == 0.135 and cal.load_lag(p, "robot")["lag_s"] == 0.02
    assert cal.load_lag(p, "robot")["clock"] == "robot"
    old = _write(tmp_path / "old.json", dict(WALL, clock="wall"))            # the single-clock file, as written before
    assert cal.load_lag(old, "wall")["lag_s"] == 0.135


def test_a_clock_the_file_lacks_is_refused(tmp_path):
    only_wall = _write(tmp_path / "a.json", {"clocks": {"wall": WALL}})
    old = _write(tmp_path / "b.json", dict(WALL, clock="wall"))
    for p in (only_wall, old):
        with pytest.raises(ValueError, match="robot clock"): cal.load_lag(p, "robot")
    with pytest.raises(ValueError): cal.load_lag(_write(tmp_path / "c.json", PER_CLOCK), "sim")
    with pytest.raises(ValueError, match="robot clock"):                     # nor through the cameras' lags
        cal.camera_lags("robot", ["color_image"], {"clocks": {"wall": WALL}}, None, clock="robot")
    with pytest.raises(ValueError, match="clock"):
        cal.camera_lags("robot", ["color_image"], PER_CLOCK, None)          # no clock named: no lag


def test_storing_one_clock_keeps_the_others(tmp_path):
    p = _write(tmp_path / "frame_lag.json", dict(WALL, clock="wall"))
    cal.store_lag(p, "robot", dict(ROBOT, clock="robot"))
    d = json.loads(p.read_text())
    assert set(d["clocks"]) == {"wall", "robot"} and d["clocks"]["wall"]["lag_s"] == 0.135
    assert cal.load_lag(p, "robot")["lag_s"] == 0.02


def _traj(path, robot_clock, rows=40, rate=0.6, scramble=False):
    """a path file as the sim writes it: before [3Pv] no host_t; since, wall_t on the robot's clock (sim time from an
    epoch) and host_t the host's, the sim at `rate` x real time"""
    head = "wall_t,sim_t,true_x,true_y,true_yaw,odom_x,odom_y,odom_yaw" + (",host_t" if robot_clock is not None else "")
    lines = [head]
    for k in range(rows):
        sim = 0.035 + 0.105 * k; host = 1791081155.76 + sim / rate + (0.3 if k % 7 == 0 else 0.0)
        wall = 1791081155.75 + sim if robot_clock else host
        tx = 99.0 * k if scramble else 7.0
        lines.append(f"{wall:.4f},{sim:.4f},{tx},{-3.6},{0.1},{7.0},{-3.6},{0.1}" + (f",{host:.4f}" if robot_clock is not None else ""))
    path.write_text("\n".join(lines) + "\n"); return path


def test_a_recordings_clock_comes_from_its_path_file(tmp_path):
    assert cal.recording_clock(traj=_traj(tmp_path / "old.traj.csv", None)) == "wall"           # house 93's form
    assert cal.recording_clock(traj=_traj(tmp_path / "new.traj.csv", True)) == "robot"         # house 96's form
    assert cal.recording_clock(traj=_traj(tmp_path / "noclock.traj.csv", False)) == "wall"     # a sim without DIMOS_SIM_CLOCK
    assert cal.recording_clock(traj=_traj(tmp_path / "s.traj.csv", True, scramble=True)) == "robot"   # true_* never read


def test_a_recordings_clock_is_found_from_the_run_that_lists_it(tmp_path):
    runs = tmp_path / "runs"; run = runs / "nights_s96"; run.mkdir(parents=True)
    db1, db2 = tmp_path / "a" / "memory.db", tmp_path / "b" / "memory.db"
    for d in (db1, db2): d.parent.mkdir(); d.write_text("")
    (run / "seed_recordings.txt").write_text(f"1 {db1}\n2 {db2}\n")
    _traj(run / "nights_s96_n1.traj.csv", None); _traj(run / "nights_s96_n2.traj.csv", True)
    assert cal.recording_clock(db=db1, roots=[runs]) == "wall" and cal.recording_clock(db=db2, roots=[runs]) == "robot"
    tidy = tmp_path / "dimos_tidy" / "20261003_200750"; tidy.mkdir(parents=True); db3 = tmp_path / "c.db"; db3.write_text("")
    (tidy / "recording.txt").write_text(f"{db3}\n"); _traj(tidy / "traj.csv", True)
    assert cal.recording_clock(db=db3, roots=[runs, tidy.parent]) == "robot"
    with pytest.raises(ValueError, match="--clock robot"):                   # a recording no run lists: given, not guessed
        cal.recording_clock(db=tmp_path / "real.db", roots=[runs])


def test_the_widened_gate_keeps_a_robot_clock_estimate():
    assert cal.LAG_GATE["lo"] == pytest.approx(-0.1) and cal.LAG_GATE["hi"] == pytest.approx(0.3)
    for lag in (-0.03, -0.015, -0.055): assert cal.lag_refusal(dict(GOOD, lag_s=lag)) is None
    assert cal.lag_refusal(dict(GOOD, lag_s=-0.25)) is not None              # house 93's sign flip still refused
    assert cal.lag_refusal(dict(GOOD, lag_s=-0.03, turning=40)) is not None  # and the turning pairs' minimum kept


def test_the_calibration_refuses_a_recording_on_another_clock(tmp_path, monkeypatch):
    import calibrate_lag
    monkeypatch.setattr(cal, "recording_clock", lambda db=None, traj=None, roots=None: "wall")
    with pytest.raises(ValueError, match="wall clock, not the robot clock"):
        calibrate_lag.check_clock("s96 n1", "x.db", "robot")


# each consumer takes the robot's lag for a robot-clock recording

def test_the_replay_takes_the_robot_clocks_lag(tmp_path):
    fm = pytest.importorskip("floor_mosaic2")                                 # dimos venv
    p = _write(tmp_path / "frame_lag.json", PER_CLOCK)
    both = ["color_image", "arm_color_image"]
    got = fm.frame_lags(None, "robot", both, p, check=False, clock="robot")
    assert got["color_image"][0] == pytest.approx(0.02) and got["arm_color_image"][0] == pytest.approx(0.02)   # no wrist
    assert "robot clock" in got["color_image"][1]                                                              # entry: the head's
    assert fm.frame_lags(None, "robot", both, p, check=False, clock="wall")["arm_color_image"][0] == pytest.approx(0.155)
    with pytest.raises(ValueError): fm.frame_lags(None, "robot", both, _write(tmp_path / "w.json", {"clocks": {"wall": WALL}}), check=False, clock="robot")
    traj = _traj(tmp_path / "n.traj.csv", True)
    assert fm.clock_of("x.db", "auto", str(traj)) == "robot" and fm.clock_of("x.db", "wall") == "wall"
    with pytest.raises(SystemExit): fm.clock_of(str(tmp_path / "unlisted.db"), "auto")


def _calls(src, name):
    return [n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Call)
            and (getattr(n.func, "id", None) == name or getattr(n.func, "attr", None) == name)]


@pytest.mark.parametrize("tool", ["floor_mosaic2.py", "calibrate_mount.py", "confirm_stream.py", "baseline_views.py"])
def test_each_tool_places_frames_with_its_recordings_clock(tool):
    """static: each tool that places frames takes the stored lag through frame_lags with the recording's clock
    (clock_of: given, or from its path file), takes --clock and --traj, and estimates no lag of its own"""
    src = (ROOT / "sim" / "house" / tool).read_text()
    calls = _calls(src, "frame_lags")
    assert calls and all(any(k.arg == "clock" for k in c.keywords) for c in calls), tool
    assert _calls(src, "clock_of") and ("add_clock_args(ap)" in src), tool
    assert not _calls(src, "lag_from_turns") and "load_lag(" not in src, tool


def test_the_stitch_and_the_night_picture_pass_each_nights_clock():
    st = (ROOT / "sim" / "dimos" / "stitch_baselines.sh").read_text()
    for tool in ("floor_mosaic2.py", "baseline_views.py", "calibrate_mount.py"):
        lines = [l for l in st.split(f"{tool} ")[1:]]
        assert lines and all("clock_args" in l.split("\n")[0] + l.split("\n")[1] for l in lines), tool
    assert 'source "$SIM/lib/clock.sh"' in st and re.search(r"^STAMP=.*clock_args", st, re.M)
    np_ = (ROOT / "sim" / "house" / "night_picture.sh").read_text()
    assert "clk=$(clock_args" in np_ and np_.count("$clk") == 2
    assert "export RECORDING_CLOCK=robot" in (ROOT / "sim" / "house" / "real_nights.sh").read_text()
    clock = (ROOT / "sim" / "house" / "lib" / "clock.sh").read_text()
    assert "RECORDING_CLOCK" in clock and "_n$2.traj.csv" in clock


def test_the_live_tidy_takes_the_robot_clocks_lag(tmp_path, monkeypatch):
    tm = pytest.importorskip("houndkeeper.tidy_module")                       # dimos venv
    assert tm.live_clock({"DIMOS_SIM_CLOCK": "houndkeeper_clock_1"}) == "robot"
    assert tm.live_clock({"DIMOS_SIM_SPEED": "1"}) == "wall"                  # a sim without the robot's clock
    assert tm.live_clock({}) == "robot"                                       # the real robot: its own clock
    p = _write(tmp_path / "frame_lag.json", PER_CLOCK)
    monkeypatch.setenv("DIMOS_SIM_CLOCK", "houndkeeper_clock_1"); monkeypatch.delenv("HK_LAG", raising=False)
    lag, note = tm.robot_frame_lag([0.02], p)
    assert lag == pytest.approx(0.02) and "robot clock" in note
    with pytest.raises(ValueError): tm.robot_frame_lag([0.02], _write(tmp_path / "w.json", {"clocks": {"wall": WALL}}))


def test_the_calibration_writes_its_own_clocks_entry():
    import calibrate_lag
    src = inspect.getsource(calibrate_lag.main)
    assert "store_lag(" in src and "write_text" not in src
    assert "check_clock(" in inspect.getsource(calibrate_lag.estimate_from_runs)
