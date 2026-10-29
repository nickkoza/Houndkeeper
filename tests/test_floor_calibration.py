"""The head camera's floor geometry adopted as the robot's calibration ([gJq], decision [yFg]): the one writer of the
mount and the floor rows' lag (tuning houses only, backed up, recorded), the floor rows' lag where the floor is laid,
the house's relief taken by every painter by default (the plane, said, without one), products stamped with their
calibration and a stale one refused ([Q9k]), and the nightly check that only reports ([9tQ])."""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))

MOUNT = {"pitch_deg": -0.802, "yaw_deg": 0.832, "roll_deg": -0.331, "from_stitch": True}
LAG = {"clocks": {"wall": {"lag_s": 0.135, "streams": {"color_image": {"lag_s": 0.135}, "arm_color_image": {"lag_s": 0.155}}},
                  "robot": {"lag_s": -0.03, "streams": {"color_image": {"lag_s": -0.03}}}}}


def _robot(tmp_path):
    d = tmp_path / "robot"; d.mkdir()
    (d / "head_mount.json").write_text(json.dumps(MOUNT)); (d / "frame_lag.json").write_text(json.dumps(LAG))
    return d


def _selfcal(tmp_path, seed, dp=-0.18, dy=0.75, du=0.0024, dl=0.0222, regs=3000, lag_seen=True, painted_with=None):
    f = tmp_path / f"s{seed}_base.json"
    d = {"dpitch_deg": dp, "dyaw_deg": dy, "dup_m": du, "dlag_s": dl if lag_seen else 0.0,
         "rounds": [{"seen": {"dlag_s": dl if lag_seen else 0.0}, "registrations": regs}, {"seen": {}, "registrations": regs}],
         "views": [f"/scratch/jzw/s{seed}/clean/views_n1.npz", f"/scratch/jzw/s{seed}/clean/views_n2.npz"]}
    if painted_with is not None: d["painted_with"] = painted_with
    f.write_text(json.dumps(d)); return f


@pytest.mark.parametrize("bad", [100, 80, 78])
def test_the_writer_refuses_hold_outs_and_the_validation_house(tmp_path, bad):
    """[gJq], [yFg]: a hold-out's or the validation house's self-calibration is never calibration data: nothing is
    written, nothing backed up"""
    import camgeo as cg
    d = _robot(tmp_path); before = {p.name: p.read_bytes() for p in d.iterdir()}
    files = [_selfcal(tmp_path, 70), _selfcal(tmp_path, bad)]
    with pytest.raises(SystemExit, match="never calibration data"):
        cg.write_calibration(files, d, d / "head_mount.json", d / "frame_lag.json")
    assert {p.name: p.read_bytes() for p in d.iterdir()} == before


def test_the_writer_backs_up_and_records_what_it_replaced(tmp_path):
    """[yFg]: both robot files copied to dated files beside them first; the mount takes the median change (pitch, yaw,
    height), the lag file the floor rows' offset beside the frame's own lag (kept); each records the previous values,
    when, and from what; the same self-calibrations are never applied twice"""
    import camgeo as cg
    d = _robot(tmp_path); old_m, old_l = (d / "head_mount.json").read_bytes(), (d / "frame_lag.json").read_bytes()
    files = [_selfcal(tmp_path, s, dp=-0.18 + 0.01 * k) for k, s in enumerate((70, 72, 73))]
    out = cg.write_calibration(files, d, d / "head_mount.json", d / "frame_lag.json", now="2026-10-07T10:00:00-0700", log=lambda *_: None)
    assert sorted(p.name for p in d.iterdir()) == ["frame_lag.20261007_100000.json", "frame_lag.json",
                                                   "head_mount.20261007_100000.json", "head_mount.json"]
    assert (d / "head_mount.20261007_100000.json").read_bytes() == old_m and (d / "frame_lag.20261007_100000.json").read_bytes() == old_l
    m = json.loads((d / "head_mount.json").read_text())
    assert m["pitch_deg"] == round(-0.802 + 0.17, 3) and m["yaw_deg"] == round(0.832 - 0.75, 3) and m["up_m"] == 0.0024
    assert m["roll_deg"] == -0.331
    c = m["calibration"]
    assert c["previous"] == {"pitch_deg": -0.802, "yaw_deg": 0.832, "roll_deg": -0.331} and c["written"].startswith("2026-10-07T10:00")
    assert c["source"]["seeds"] == [70, 72, 73] and c["source"]["step"] == "camgeo.py calibrate" and len(c["backups"]) == 2
    w = json.loads((d / "frame_lag.json").read_text())["clocks"]
    assert w["wall"]["streams"]["color_image"]["lag_s"] == 0.135 and w["wall"]["lag_s"] == 0.135   # the frame's lag kept
    assert w["wall"]["floor_rows_offset_s"] == -0.0222 and w["wall"]["floor_rows"]["floor_lag_s"] == 0.1128
    assert w["wall"]["floor_rows"]["previous_offset_s"] is None and "floor_rows_offset_s" not in w["robot"]
    assert out["backups"]
    with pytest.raises(SystemExit, match="already written"):
        cg.write_calibration(files, d, d / "head_mount.20261007_100000.json", d / "frame_lag.20261007_100000.json", log=lambda *_: None)


def test_the_writer_refuses_views_painted_with_another_calibration(tmp_path):
    """[yFg]: a self-calibration is a change to the calibration its views were painted with: one whose views say
    another is refused, and one that does not say needs the base given"""
    import camgeo as cg
    d = _robot(tmp_path)
    with pytest.raises(SystemExit, match="painted with"):
        cg.write_calibration([_selfcal(tmp_path, 70, painted_with={"mount_sha": "aaaaaaaaaaaa", "lag_sha": "bbbbbbbbbbbb", "relief_sha": ""})], d)
    with pytest.raises(SystemExit, match="give --base-mount"):
        cg.write_calibration([_selfcal(tmp_path, 72)], d)


def test_the_floor_is_laid_with_the_floor_rows_lag(tmp_path):
    """[yFg]: the head camera's floor is placed with the stored lag plus the floor rows' offset, the wrist's and the
    other clock's unchanged; tonight's turning estimate is still checked against the frame's own lag; recalibrating
    the lag keeps the offset"""
    from houndkeeper import calibrate as cal
    d = json.loads(json.dumps(LAG)); d["clocks"]["wall"]["floor_rows_offset_s"] = -0.0222
    e = cal.lag_entry(d, "wall")
    assert cal.floor_lag(e, "color_image") == pytest.approx(0.1128) and cal.floor_lag(e, "arm_color_image") == 0.155
    assert cal.floor_lag(cal.lag_entry(d, "robot"), "color_image") == -0.03
    est = lambda s: {"lag_s": 0.135, "corr": 0.9, "pairs": 400, "turning": 300, "edge": False}
    got = cal.camera_lags("robot", ["color_image", "arm_color_image"], d, est, clock="wall")
    assert got["color_image"][0] == pytest.approx(0.1128) and "floor rows" in got["color_image"][1] and "agrees" in got["color_image"][1]
    assert got["arm_color_image"][0] == 0.155
    p = tmp_path / "lag.json"; p.write_text(json.dumps(d))
    cal.store_lag(p, "wall", {"lag_s": 0.14, "streams": {"color_image": {"lag_s": 0.14}}})
    assert json.loads(p.read_text())["clocks"]["wall"]["floor_rows_offset_s"] == -0.0222
    from houndkeeper import tidy_module as tm
    lag, note = tm.robot_frame_lag([], lag_file=p, override="", clock="wall")
    assert lag == pytest.approx(0.14 - 0.0222) and "floor rows" in note


def _run_with_relief(tmp_path, frame=True):
    from houndkeeper import floor_relief as frl
    run = tmp_path / "run"; (run / "shape").mkdir(parents=True)
    for k in (1, 2): np.savez(run / "shape" / f"corrections_{k}.npz", times=np.zeros(1), Ts=np.eye(3)[None])
    g = np.zeros((980, 1680), np.float32); g[:60, :60] = 0.012
    frl.save(run, g, [run / "shape" / f"corrections_{k}.npz" for k in (1, 2)] if frame else [])
    from houndkeeper import calibrate as cal
    (run / "stitch.args").write_text(f"{cal.MAP_VERSION} head d1 d2\n")   # a current stitch ([dmT]: an older one is refused)
    return run


def test_painters_take_the_houses_relief_in_its_frame(tmp_path):
    """[yFg]: 'auto' is the house's relief when painting in its frame (pose corrections it names), and the confirmer
    on the run's grid; the plane, said, without one, in another frame, or in odometry's own"""
    from houndkeeper import floor_relief as frl
    run = _run_with_relief(tmp_path); said = []
    g, p, _ = frl.resolve("auto", run, run / "shape" / "corrections_1.npz", log=said.append)
    assert g is not None and g[0, 0] == np.float32(0.012) and p.endswith("relief.npy") and not said
    g, p, _ = frl.resolve("auto", run, None, log=said.append); assert g is not None and not said   # the confirmer's
    other = tmp_path / "other.npz"; np.savez(other, times=np.ones(2), Ts=np.stack([np.eye(3)] * 2))
    g, p, note = frl.resolve("auto", run, other, log=said.append); assert g is None and p is None and "another frame" in said[-1]
    g, p, note = frl.resolve("auto", tmp_path / "empty", None, log=said.append)
    assert g is None and said[-1].startswith("warning: no floor relief")
    g, p, note = frl.resolve_odometry("auto", run, log=said.append); assert g is None and "odometry" in note
    assert frl.resolve("none", run)[0] is None


def test_every_painter_defaults_to_the_houses_relief():
    """[yFg]: the floor picture, the baseline views and the confirmer default to the house's relief; the live tidy
    lays tonight on it; the stitch solves it and paints every picture and view set on it"""
    src = {f: (HK / f).read_text() for f in ("sim/house/floor_mosaic2.py", "sim/house/baseline_views.py", "sim/house/confirm_stream.py")}
    for f, t in src.items():
        assert 'add_argument("--relief", default="auto"' in t, f
    tm = (HK / "src/houndkeeper/tidy_module.py").read_text()
    assert "self._cf.relief = getattr(self, \"_relief\", None)" in tm and "baseline_calibration(sw[\"run_dir\"]" in tm
    sb = (HK / "sim/dimos/stitch_baselines.sh").read_text()
    assert "floor_selfcal.py relief" in sb and "floor_selfcal.py check" in sb and 'STAMP="$MAP_VERSION ' in sb
    assert sb.index("floor_selfcal.py relief") < sb.index("floor_mosaic2.py") < sb.rindex('views "$RUN/head_mount.json" "$RUN/shape"')


def test_a_product_painted_under_another_calibration_is_refused(tmp_path):
    """[Q9k]: pictures and views carry what they were painted with; one made before the stamps, or with another
    mount, lag or relief, is named; the confirmer refuses it unless told to compare across, and the live tidy stops"""
    import argparse
    from houndkeeper import calibrate as cal, tidy_module as tm
    import confirm_stream as cs
    run = _run_with_relief(tmp_path)
    mount, lag = tmp_path / "m.json", tmp_path / "l.json"; mount.write_text(json.dumps(MOUNT)); lag.write_text(json.dumps(LAG))
    (run / "head_mount.json").write_text(json.dumps(MOUNT))
    want = cal.stamp(run / "head_mount.json", lag, run / "relief.npy")
    for k in (1, 2):
        np.savez(run / f"mosaic_n{k}.npz", rgb=np.zeros((2, 2, 3), np.uint8), **{q: np.str_(v) for q, v in want.items()})
        cs.Views([[0, 0]], [[1, 0]], [(0, 1, 0, 1)], [np.zeros((1, 1, 3), np.uint8)], [0.0], [np.eye(4)]).save(run / f"views_n{k}.npz", want)
    assert cal.stale_products(run, want) == [] and cal.stamp_of(run / "views_n1.npz") == want
    a = argparse.Namespace(relief="auto", mount=str(run / "head_mount.json"), lag_file=str(lag), dt="robot",
                           views=[str(run / "views_n1.npz"), str(run / "views_n2.npz")], mixed_calibration=False)
    relief, rpath = cs.calibration_used(a, run, [1, 2]); assert relief is not None
    assert tm.baseline_calibration(run, [1, 2], lambda *_: None, lag_file=lag) is not None
    np.savez(run / "mosaic_n2.npz", rgb=np.zeros((2, 2, 3), np.uint8), lag_sha=np.str_(want["lag_sha"]), mount_sha=np.str_(want["mount_sha"]))
    assert cal.stale_products(run, want) == ["mosaic_n2.npz carries no calibration stamp (made before [yFg]'s stamps)"]
    other = dict(want, mount_sha="0123456789ab")
    np.savez(run / "mosaic_n2.npz", rgb=np.zeros((2, 2, 3), np.uint8), **{q: np.str_(v) for q, v in other.items()})
    assert any("another mount" in w for w in cal.stale_products(run, want))
    with pytest.raises(SystemExit, match="another calibration"):
        cs.calibration_used(a, run, [1, 2])
    a.mixed_calibration = True; assert cs.calibration_used(a, run, [1, 2])[0] is not None
    with pytest.raises(RuntimeError, match="another calibration"):
        tm.baseline_calibration(run, [1, 2], lambda *_: None, lag_file=lag)


def _solve(dp=0.0, dy=0.0, du=0.0, dl=0.0, regs=5000, lag_seen=True):
    return {"dpitch_deg": dp, "dyaw_deg": dy, "dup_m": du, "dlag_s": dl,
            "rounds": [{"seen": {"dlag_s": dl if lag_seen else 0.0}, "registrations": regs}, {"registrations": regs}]}


def test_the_nightly_check_warns_past_its_tolerances_and_only_reports():
    """[9tQ], [yFg]: a night's free solve that differs from the record past 0.1 deg, 5 mm or 10 ms is warned about;
    within them it agrees; too little floor is said, not judged; an unseen lag is not judged"""
    import floor_selfcal as fs
    assert fs.check_result(_solve(0.05, -0.08, 0.004, 0.009))["warnings"] == []
    w = fs.check_result(_solve(0.15, 0.0, -0.006, 0.012))["warnings"]
    assert len(w) == 3 and "pitch" in w[0] and "+0.150 deg" in w[0] and "-6.0 mm" in w[1] and "+12.0 ms" in w[2]
    r = fs.check_result(_solve(0.5, regs=900)); assert not r["judged"] and r["warnings"] == [] and "too little floor" in r["note"]
    r = fs.check_result(_solve(dl=0.05, lag_seen=False)); assert r["change"]["lag_s"] is None and r["warnings"] == []
    src = (HK / "sim/house/floor_selfcal.py").read_text()
    assert "data/robot" not in src.split("def check_main")[1].split("def main")[0]
