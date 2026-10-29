"""The house frame's gate ([Tu2], decisions [myl] and [UpK]): a night the stitch places far from its own odometry, at
the dock or over the night (further from the dock, further allowed), is refused, loudly, through the build helper
([SdA]); every working house passes."""
import math
import re
import subprocess
import sys
from pathlib import Path

from houndkeeper import workspace

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house"))
import frame_gate as fg  # noqa: E402

PY = workspace.dimos() / ".venv" / "bin" / "python"
RUNS = HK / "sim" / "house" / "runs"
# the tuning houses ([aj8]: 70-73, 76, 77, 82-93), validation 78 and the extra robot-clock houses 95, 96; never a
# hold-out (80, 81, 100-119)
WORKING = [70, 71, 73, 76, 77, 78, *range(82, 94), 95, 96]


def se2(x, y, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def write_shape(d, nights):
    """a shape solve's outputs from {night: [(t, believed (x, y), correction 3x3), ...]}"""
    d.mkdir(parents=True, exist_ok=True); T, N, B = [], [], []
    for k, rows in nights.items():
        np.savez(d / f"corrections_{k}.npz", times=np.array([r[0] for r in rows]), Ts=np.array([r[2] for r in rows]))
        T += [r[0] for r in rows]; N += [k] * len(rows); B += [r[1] for r in rows]
    np.savez(d / "settle.npz", passes=np.zeros((1, len(T), 3, 3)), times=np.array(T), night=np.array(N), believed=np.array(B))
    return d


def walk(n=40, start=(7.0, -3.6), drift_deg=0.0, drift_m=0.0, fold=None):
    """a night along x from the dock: its corrections drift linearly to drift_deg / drift_m by the end; fold=(i, deg):
    from keyframe i on, the frame turned by deg about the dock"""
    rows = []
    for i in range(n):
        f = i / (n - 1); p = (start[0] + 0.1 * i, start[1])
        C = se2(drift_m * f, 0.0, 0.0) @ se2(0, 0, 0)
        R = se2(0, 0, drift_deg * f); C = C @ (se2(*p, 0) @ R @ se2(-p[0], -p[1], 0))   # turned about the robot itself
        if fold and i >= fold[0]:
            C = se2(*start, 0) @ se2(0, 0, fold[1]) @ se2(-start[0], -start[1], 0) @ C
        rows.append((100.0 + i, p, C))
    return rows


def test_identity_and_drift_within_the_limits_pass(tmp_path):
    """[Tu2]: odometry's own drift (a few degrees, tens of cm) is not refused"""
    v = fg.check(write_shape(tmp_path / "s", {1: walk(), 2: walk(drift_deg=9.0, drift_m=0.6)}))
    assert not v[1]["refused"] and not v[2]["refused"]
    assert v[2]["max_turn_deg"] == pytest.approx(9.0, abs=0.01) and v[2]["max_move_m"] == pytest.approx(0.6, abs=0.01)


def test_the_move_is_measured_at_the_robot_not_the_raw_translation(tmp_path):
    """[myl]: a small turn about the robot 8 m from the origin has a raw translation of metres and moves it nothing"""
    v = fg.check(write_shape(tmp_path / "s", {1: walk(drift_deg=4.0)}))
    C = np.load(tmp_path / "s" / "corrections_1.npz")["Ts"][-1]
    assert math.hypot(C[0, 2], C[1, 2]) > 0.4 and v[1]["max_move_m"] < 1e-6 and not v[1]["refused"]


def test_a_dock_turned_or_moved_is_refused(tmp_path):
    """[Tu2]: night 2 starts on the dock: a correction there past 5 deg or 0.3 m is the frame gone wrong"""
    turned = [(t, p, se2(7.0, -3.6, 0) @ se2(0, 0, 6.0) @ se2(-7.0, 3.6, 0) @ C) for t, p, C in walk()]
    moved = [(t, p, se2(0.4, 0.0, 0) @ C) for t, p, C in walk()]
    v = fg.check(write_shape(tmp_path / "s", {1: walk(), 2: turned, 3: moved}))
    assert not v[1]["refused"]
    assert any("at the dock" in w and "deg" in w for w in v[2]["refused"])
    assert any("at the dock" in w and " m " in w for w in v[3]["refused"])


def test_a_night_that_folds_mid_way_is_refused(tmp_path):
    """[Tu2]: the dock fine, the frame turned 32 deg from keyframe 10 on (house 72's night 1: -32 then -55 deg)"""
    v = fg.check(write_shape(tmp_path / "s", {1: walk(fold=(10, -32.0))}))
    assert v[1]["dock_turn_deg"] < 1e-6 and any("over the night, keyframe 10" in w for w in v[1]["refused"])


def about_dock(rows, deg_at, start=(7.0, -3.6)):
    """rows with the frame turned about the dock by deg_at(i) degrees at keyframe i: odometry's heading drift, or a fold"""
    return [(t, p, se2(*start, 0) @ se2(0, 0, deg_at(i)) @ se2(-start[0], -start[1], 0) @ C) for i, (t, p, C) in enumerate(rows)]


def test_a_long_night_drifted_past_a_metre_far_from_the_dock_passes(tmp_path):
    """[UpK]: heading drift moves the robot in proportion to its distance from the dock: a 20 m night drifting to 5 deg
    (house 76's 1280 night 2: 4-5.7 deg, 1.02 m from odometry 10.8 m out, a right frame) passes past the flat 1 m"""
    v = fg.check(write_shape(tmp_path / "s", {1: about_dock(walk(n=200), lambda i: 5.0 * i / 199)}))[1]
    assert v["max_move_m"] > 1.5 and v["max_turn_deg"] == pytest.approx(5.0, abs=0.01) and not v["refused"], v
    assert v["tightest_limit_m"] == pytest.approx(0.3 + 0.08 * v["tightest_dist_m"]) and v["tightest_limit_m"] > 1.0
    assert fg.move_limit(0.0) == fg.NIGHT_M == 1.0 and fg.move_limit(8.75) == pytest.approx(1.0)  # never under 1 m


def test_a_frame_turned_about_the_dock_is_still_refused_by_its_move(tmp_path):
    """[UpK], [Tu2]: a turn too small for the 15 deg limit (8 deg about the dock from keyframe 30 of a 20 m night) moves
    the far end 2.8 m, past the limit there, and is refused for the move, naming the keyframe and the limit used"""
    v = fg.check(write_shape(tmp_path / "s", {1: about_dock(walk(n=200), lambda i: 8.0 if i >= 30 else 0.0)}))[1]
    assert v["max_turn_deg"] < fg.NIGHT_DEG and v["max_move_m"] > 2.5
    assert any(re.search(r"moved [\d.]+ m over the night, keyframe \d+, [\d.]+ m from the dock \(limit 1\.\d\d m there\)", w)
               for w in v["refused"]), v["refused"]


def test_house_76s_1280_night_passes_and_its_planted_turns_are_refused():
    """[UpK]: house 76's 1280 stitch, refused on 13 Oct at 1.02 m by the flat limit, passes; its night 2 turned about the
    dock from keyframe 300 on is refused at -12..-4 and +10, +12 deg, as the flat 1 m refused it (positive control)"""
    d = RUNS / "nights_s76" / "archive" / "20261013_failed1280" / "shape"
    if not (d / "settle.npz").exists(): pytest.skip("house 76's 1280 stitch is not on this machine")
    assert not [r["refused"] for r in fg.check(d).values() if r["refused"]]
    z = np.load(d / "settle.npz"); m = np.asarray(z["night"]) == 2; c = np.load(d / "corrections_2.npz")
    B = np.asarray(z["believed"])[m].reshape(-1, 2); t0 = np.asarray(z["times"])[m][300]; x, y = B[0]
    refused = []
    for deg in (-12, -10, -8, -6, -4, 4, 6, 8, 10, 12):
        F = se2(x, y, 0) @ se2(0, 0, deg) @ se2(-x, -y, 0)
        Ts = np.array([F @ T if t >= t0 else T for t, T in zip(c["times"], c["Ts"])])
        turn, move = fg.excursions(c["times"], Ts, np.asarray(z["times"])[m], B)
        if fg.judge(turn, move, np.linalg.norm(B - B[0], axis=1))["refused"]: refused.append(deg)
    assert refused == [-12, -10, -8, -6, -4, 10, 12]


def test_the_cli_refuses_with_a_nonzero_exit_and_writes_nothing(tmp_path):
    """[Tu2]: a refusal exits 1, names the night and how far, and leaves no gate.json behind"""
    d = write_shape(tmp_path / "s", {1: walk(), 2: walk(fold=(0, -90.0))})
    r = subprocess.run([str(PY), str(HK / "sim/house/frame_gate.py"), str(d), "--out", str(d / "gate.json")], capture_output=True, text=True)
    assert r.returncode == 1 and "REFUSED" in r.stdout and "night 2" in r.stdout and "90.0 deg" in r.stdout
    assert not (d / "gate.json").exists()
    d2 = write_shape(tmp_path / "ok", {1: walk(), 2: walk(drift_deg=3)})
    r = subprocess.run([str(PY), str(HK / "sim/house/frame_gate.py"), str(d2), "--out", str(d2 / "gate.json")], capture_output=True, text=True)
    assert r.returncode == 0 and (d2 / "gate.json").exists(), r.stdout + r.stderr


def test_the_stitch_runs_the_gate_through_the_build_helper_and_stops(tmp_path):
    """[Tu2], [SdA]: stitch_baselines.sh gates the shape before anything is built in it, and a reused stitch too; the
    build helper fails the refusal loudly, with the gate's own lines"""
    s = (HK / "sim/dimos/stitch_baselines.sh").read_text()
    body = s[s.index("gate() {"):]
    assert re.search(r"build [^\n]*frame_gate\.py[^\n]*\|\| \{", body), "the gate goes through build"
    assert re.search(r"exit 1; \}", body[:body.index("\n}\n")]), "a refusal stops the stitch"
    shape_at = s.index("$PY house_shape.py"); first_use = s.index("\ngate\n")
    assert shape_at < first_use < s.index("floor_mosaic2.py"), "gated after the shape solve, before any picture"
    assert re.search(r'= "\$STAMP" \].*\n\s+gate;', s), "a stitch reused by its stamp is gated too"
    d = write_shape(tmp_path / "shape", {1: walk(), 2: walk(fold=(0, -90.0))})
    lib = HK / "sim/house/lib/build.sh"
    r = subprocess.run(["bash", "-c", f"set -eu; source {lib}; build --show 'house frame gate|REFUSED' {d}/gate.json -- "
                        f"{PY} {HK}/sim/house/frame_gate.py {d} --out {d}/gate.json; echo after"], capture_output=True, text=True)
    assert r.returncode != 0 and "after" not in r.stdout and "FAILED: frame_gate.py (exit 1)" in r.stdout and "REFUSED" in r.stdout


def _shape(seed):
    d = RUNS / f"nights_s{seed}" / "shape"
    if not (d / "settle.npz").exists(): pytest.skip(f"house {seed}'s stitch is not on this machine")
    return d


@pytest.mark.parametrize("seed", WORKING)
def test_every_working_house_passes(seed):
    """[myl]: the limits clear every tuning house's own shape corrections on disk"""
    v = fg.check(_shape(seed))
    assert not [r["refused"] for r in v.values() if r["refused"]], v


def test_house_72s_broken_stitch_is_refused():
    """[Tu2]: house 72's 3 Oct stitch (turned about -90 deg), kept beside its run, is refused at night 2's dock and over
    night 1; its current stitch passes"""
    broken = RUNS / "nights_s72" / "broken_20261003" / "shape"
    if not (broken / "settle.npz").exists(): pytest.skip("house 72's broken stitch is not on this machine")
    v = fg.check(broken)
    assert v[1]["max_turn_deg"] > 80 and v[2]["dock_turn_deg"] > 80 and v[1]["refused"] and v[2]["refused"]
    assert not [r["refused"] for r in fg.check(_shape(72)).values() if r["refused"]]


def test_a_houses_own_pose_graph_settings_are_used_and_stamped():
    """[myl]: a house's own settings over house_frame.py's (FRAME_PGO, or its run's frame_pgo.args) reach the pose graph
    and its stitch's stamp, so a repaired frame is rebuilt the same way and a change of them re-stitches"""
    s = (HK / "sim/dimos/stitch_baselines.sh").read_text()
    assert 'FRAME_PGO=${FRAME_PGO:-$(cat "$RUN/frame_pgo.args"' in s
    stamp = next(l for l in s.splitlines() if l.startswith("STAMP="))
    assert "${FRAME_PGO:+ pgo=$FRAME_PGO}" in stamp
    assert re.search(r"house_frame\.py [^\n]*\$\{FRAME_PGO:\+--pgo \$FRAME_PGO\}", s)
