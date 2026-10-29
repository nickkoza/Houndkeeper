"""[Zaf]: the light check behind the floor comparison, on the robot (houndkeeper.candidate_check), on synthetic cameras
and frames, no model loaded: each candidate's views are the frames nearest in time to its first decision that see its
point, chosen online exactly as the bench chooses them over the whole night (cascade.pick_views_by_time); its crops are
the bench's; it is kept at the chosen threshold or when no frame saw it; the recognizer falls back to the CPU when the
GPU fails; and the check reads no sim truth (tests/test_truth_boundary.py places it on the robot's side)."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))

from houndkeeper import candidate_check as cc   # noqa: E402

K = np.array([[600.0, 0, 320], [0, 600.0, 180], [0, 0, 1]]); WH = (640, 360)


def cam(x, y, yaw):
    """a camera 0.5 m up at (x, y) looking along yaw, pitched 30 deg down (world <- optical)"""
    f = np.array([math.cos(yaw), math.sin(yaw), 0.0]); left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    p = math.radians(30); z = f * math.cos(p) + np.array([0, 0, -math.sin(p)])   # optical z: forward and down
    xo = -left; yo = np.cross(z, xo)
    T = np.eye(4); T[:3, :3] = np.stack([xo, yo, z], 1); T[:3, 3] = [x, y, 0.5]
    return T


def walk(n=120, dt=0.5):
    """a walk along x and back (out at 0.1 m/s, back the same way), a frame every dt s"""
    out = []
    for k in range(n):
        t = k * dt; x = 0.1 * t if k < n // 2 else 0.1 * (n * dt - t)
        out.append((1000.0 + t, cam(x, 0.0, 0.0 if k < n // 2 else math.pi)))
    return out


class Const:
    """a recognizer scoring every crop the same"""
    device = "stub"

    def __init__(self, s): self.s = s

    def score(self, crop, uv): return self.s


def run_online(frames, decisions, rec=None):
    """the check fed frames and decisions in time order (a decision after the frame it was made on)"""
    chk = cc.CandidateCheck(K, None, "", WH, recognizer=rec or Const(0.5), keep_s=1e9)
    img = np.zeros((WH[1], WH[0], 3), np.uint8); q = sorted(decisions, key=lambda d: d["ts"]); j = 0
    for ts, T in frames:
        while j < len(q) and q[j]["ts"] < ts: chk.candidate(q[j]); j += 1
        chk.frame(ts, T, img)
    for d in q[j:]: chk.candidate(d)
    chk.finish(); return chk


def test_views_are_the_nearest_in_time_and_match_the_bench():
    """[Zaf] crop selection by time: online, as the bench picks over the whole night"""
    from baselines import cascade as ca
    frames = walk(); pts = [(4.0, 0.0, 1025.0), (3.0, 0.05, 1040.0), (5.5, -0.1, 1001.0), (2.0, 0.0, 1059.0)]
    chk = run_online(frames, [{"decision": "confirmed", "x": x, "y": y, "ts": t} for x, y, t in pts])
    poses = [(i, ts, T) for i, (ts, T) in enumerate(frames)]
    bench = ca.pick_views_by_time(pts, poses, WH[0], WH[1], K, None, "")
    for (x, y, t), b in zip(pts, bench):
        v = chk.verdicts[cc.key(x, y)]
        assert sorted(m["ts"] for m in v["views"]) == sorted(frames[i][0] for i, *_ in b)
        assert v["settled"] in ("settled", "night over")
    # the nearest in time, not the nearest in range: the point at 4.0 seen going out (t near 1025) and coming back
    v = chk.verdicts[cc.key(4.0, 0.0)]
    assert max(abs(m["ts"] - 1025.0) for m in v["views"]) <= 3.0
    ts = sorted(m["ts"] for m in v["views"]); assert all(b - a >= cc.SEP_S for a, b in zip(ts, ts[1:]))


def test_a_candidate_settles_once_no_later_frame_can_change_it():
    """[Zaf]: scored as soon as its views are fixed (live: the robot does not wait for the night's end)"""
    frames = walk(); chk = cc.CandidateCheck(K, None, "", WH, recognizer=Const(0.5), keep_s=1e9)
    img = np.zeros((WH[1], WH[0], 3), np.uint8); got = {}
    for ts, T in frames:
        got.update(chk.frame(ts, T, img))
        if ts == 1025.0: chk.candidate({"decision": "unsure", "x": 4.0, "y": 0.0, "ts": ts})
    v = chk.verdicts[cc.key(4.0, 0.0)]
    assert v["settled"] == "settled" and v["settled_ts"] <= 1025.0 + 2 * cc.SEP_S + 0.5 and cc.key(4.0, 0.0) in got


def test_the_crop_is_the_benchs():
    """[Zaf]: the crop box, the projection and the house labels are the bench's (recheck.py, place.py, prompts.py)"""
    from baselines import place as pl, prompts as pr, recheck as rk, recognizers as rc
    for args in [(100.5, 200.2, 1.3, 796.5, 1280, 720), (5, 715, 0.7, 796.5, 1280, 720), (1270, 10, 2.0, 600, 640, 360)]:
        assert cc.crop_box(*args) == rk.crop_box(*args)
    T = cam(0.0, 0.0, 0.3); D = [-0.02, 0.01, -0.003, 0.0005]
    for xy in [(1.0, 0.2), (1.8, -0.4)]:
        assert cc.project(xy, T, K, D, "equidistant") == pl.project(xy, T, K, D, "equidistant")
    assert cc.STRUCTURE == pr.STRUCTURE and (cc.CROP_PX, cc.CENTRE_GROW, cc.FLOOR_SCORE) == (rc.CROP_PX, rc.CENTRE_GROW, rc.FLOOR_SCORE)
    assert (cc.VIEWS, cc.RANGE_M, cc.SEP_S, cc.MARGIN_PX, cc.SPAN_M, cc.EVERY, cc.GRID_M) == \
        (rk.VIEWS, rk.RANGE_M, rk.SEP_S, rk.MARGIN_PX, rk.SPAN_M, rk.EVERY, rk.GRID_M)


def test_the_threshold_is_the_leads_chosen_on_tuning():
    """[Zaf] the threshold: the lead's, chosen on the tuning nights (the cascade round's results)"""
    R = json.load(open(HK / "tests/fixtures/detection_cascade_results.json"))   # the cascade round's results, 12 Oct 2026
    assert cc.THRESHOLD == R["tuning"]["yoloe_pf_l (lead: views by time)"]["threshold"]


@pytest.mark.parametrize("score,kept", [(cc.THRESHOLD, True), (cc.THRESHOLD - 1e-4, False), (0.9, True), (0.0, False)])
def test_kept_at_the_threshold(score, kept):
    """[Zaf]: kept at the threshold or above"""
    chk = run_online(walk(), [{"decision": "confirmed", "x": 4.0, "y": 0.0, "ts": 1015.0}], Const(score))
    assert chk.verdicts[cc.key(4.0, 0.0)]["kept"] is kept


def test_a_candidate_no_frame_saw_is_kept_and_floor_changes_are_not_checked():
    """[Zaf]: the robot cannot check what it never saw, so it stops for it; floor-sized changes are not candidates"""
    chk = run_online(walk(), [{"decision": "confirmed", "x": 40.0, "y": 9.0, "ts": 1015.0},
                              {"decision": "confirmed", "kind": "floor_change", "x": 4.0, "y": 0.0, "ts": 1015.0},
                              {"decision": "not_a_find", "x": 3.0, "y": 0.0, "ts": 1015.0}], Const(0.0))
    v = chk.verdicts[cc.key(40.0, 9.0)]
    assert v["score"] is None and v["kept"] and v["views"] == []
    assert set(chk.verdicts) == {cc.key(40.0, 9.0)}


def test_each_crop_keeps_its_frame_stamp():
    """[Zaf]: each view's frame stamp, index, range and box are kept with the verdict"""
    chk = run_online(walk(), [{"decision": "confirmed", "x": 4.0, "y": 0.0, "ts": 1015.0}])
    for m in chk.verdicts[cc.key(4.0, 0.0)]["views"]:
        assert {"ts", "index", "range_m", "uv", "box_full", "score"} <= set(m) and 1000.0 <= m["ts"] <= 1060.0


def test_frames_older_than_the_memory_are_not_views():
    """live: frames are kept KEEP_S; a candidate decided later than that after a frame cannot use it"""
    frames = walk(); chk = cc.CandidateCheck(K, None, "", WH, recognizer=Const(0.5), keep_s=5.0)
    img = np.zeros((WH[1], WH[0], 3), np.uint8)
    for ts, T in frames: chk.frame(ts, T, img)
    chk.candidate({"decision": "confirmed", "x": 4.0, "y": 0.0, "ts": 1059.5}); chk.finish()
    assert all(m["ts"] >= 1059.5 - 5.0 for m in chk.verdicts[cc.key(4.0, 0.0)]["views"])


def test_the_gpu_falls_back_to_the_cpu():
    """[Zaf] CPU fallback: a GPU that fails at load, or mid-night, leaves the check running on the CPU"""
    made = []

    class Det:
        def __init__(self, dev, fail): self.dev, self.fail = dev, fail

        def predict_image(self, img):
            if self.fail: raise RuntimeError("CUDA error: out of memory")
            from types import SimpleNamespace as N
            return N(detections=[N(name="toy", bbox=(100, 100, 200, 200), confidence=0.7)])

    def make_failing_load(dev):
        made.append(dev)
        if dev == "cuda": raise RuntimeError("CUDA out of memory")
        return Det(dev, False)
    y = cc.Yoloe("cuda", make_failing_load, say=lambda s: None).load()
    assert y.device == "cpu" and made == ["cuda", "cpu"]
    y2 = cc.Yoloe("cuda", lambda dev: Det(dev, dev == "cuda"), say=lambda s: None).load()
    assert y2.score(np.zeros((384, 384, 3), np.uint8), (150, 150)) == 0.7 and y2.device == "cpu"


def test_the_device_is_the_gpu_when_there_is_one(monkeypatch):
    """[Zaf]: the laptop's GPU in real use, the CPU without one, HK_CHECK_DEVICE to choose"""
    import torch
    monkeypatch.delenv("HK_CHECK_DEVICE", raising=False)
    monkeypatch.setattr(torch.cuda, "is_available", lambda: False); assert cc.device() == "cpu"
    monkeypatch.setattr(torch.cuda, "is_available", lambda: True); assert cc.device() == "cuda"
    monkeypatch.setenv("HK_CHECK_DEVICE", "cpu"); assert cc.device() == "cpu"


def test_the_check_is_off_by_default():
    """[Zaf]: off until turned on (the replay's --check, the live tidy's HK_CANDIDATE_CHECK)"""
    import confirm_stream as cs
    a = cs.parser().parse_args(["--run-dir", "r", "--seed", "1", "--night", "3", "--db", "d"])
    assert a.check is False
    src = (HK / "src/houndkeeper/tidy_module.py").read_text()
    assert 'os.environ.get("HK_CANDIDATE_CHECK") == "1"' in src


def test_the_check_reads_no_truth():
    """[Zaf] truth boundary: the check is placed on the robot's side and names no truth file"""
    import test_truth_boundary as tb
    assert "src/houndkeeper/candidate_check.py" in tb.ROBOT
    text = (HK / "src/houndkeeper/candidate_check.py").read_text()
    assert not tb.TRUTH.search(text) and "baselines" not in "".join(l for l in text.splitlines() if l.lstrip().startswith(("import", "from")))


def test_the_live_tidy_holds_a_candidate_until_the_check_scores_it():
    """[Zaf]: live, a candidate gets no stop until scored; kept, it passes on; dropped, it is logged and never stopped for"""
    from types import SimpleNamespace as N
    from houndkeeper.tidy_module import TidyArm
    frames = walk(); img = np.zeros((WH[1], WH[0], 3), np.uint8)
    for score, passes in [(0.9, True), (0.0, False)]:
        me = N(_check=cc.CandidateCheck(K, None, "", WH, recognizer=Const(score)), _held={}, _sweep_log=[], _frame_ts=None)
        got = []
        for ts, T in frames:
            me._check.frame(ts, T, img); me._frame_ts = ts
            recs = [{"decision": "confirmed", "x": 4.0, "y": 0.0, "hits": 5}] if ts == 1025.0 else []
            recs.append({"decision": "not_a_find", "x": 9.0, "y": 0.0})
            out = TidyArm._checked(me, recs)
            got += [r for r in out if r["decision"] == "confirmed"]
            if ts == 1025.0: assert not any(r["decision"] == "confirmed" for r in out)   # held, not yet scored
        assert bool(got) is passes and (got[0]["check"]["kept"] if got else True)
        assert passes or any("dropped by the check" in r.get("why", "") for r in me._sweep_log)


def test_live_a_candidate_is_scored_after_the_wait_with_the_views_it_has():
    """[Zaf]: live, a candidate seen fewer than VIEWS times is not held for the rest of the night"""
    frames = walk()
    chk = cc.CandidateCheck(K, None, "", WH, recognizer=Const(0.5), max_wait_s=5.0)
    img = np.zeros((WH[1], WH[0], 3), np.uint8)
    for ts, T in frames:
        chk.frame(ts, T, img)
        if ts == 1010.0: chk.candidate({"decision": "confirmed", "x": 40.0, "y": 9.0, "ts": ts})   # never seen
    v = chk.verdicts[cc.key(40.0, 9.0)]
    assert v["settled"] == "waited" and v["settled_ts"] <= 1016.0 and v["kept"]


def test_the_launch_setting_is_the_cascade_rounds_stage_one_and_the_check():
    """[tTD], [Zaf]: one named setting selects the launch bar's stage 1 (the r11 near choice over floor_arm's loose1)
    and the check, at the colour threshold the launch numbers were measured at (0.045, [LQs]); explicit options still
    override it; without it the defaults are unchanged"""
    import confirm_stream as cs
    from baselines import floor_arm as fa
    base = ["--run-dir", "r", "--seed", "1", "--night", "3", "--db", "d", "--near-views", "a.npz", "b.npz"]
    loose1 = [x for k, v in fa.SENSITIVITIES["loose1"][0].items() for x in (f"--{k}", str(v))]
    r11 = ("--view-noise 4 --tc 0.045 --tb 0.6 --hits 2 --span 0.3 --ratio 0.4 --slop 0 --small-px 4 --small-peak 0.4 "
           "--slack 1 --near-res 0.005 --near-rmax 1.0 --near-opt span=0.3 --near-opt min_cm2=0.5 --check").split()
    want = vars(cs.parser().parse_args(base + loose1 + r11))
    got = vars(cs.parser().parse_args(base + ["--setting", "launch"]))
    assert {k: v for k, v in got.items() if k != "setting"} == {k: v for k, v in want.items() if k != "setting"}
    assert cs.parser().parse_args(base + ["--setting=launch", "--hits", "5"]).hits == 5
    plain = cs.parser().parse_args(base)
    assert plain.check is False and plain.near_res == 0.0 and plain.hits == cs.CONFIRM["hits"]


def test_with_the_check_off_the_tidy_passes_every_decision_on_unchanged():
    """[Zaf]: off (the default), whatever path built the tidy, its decisions are untouched"""
    from types import SimpleNamespace as N
    from houndkeeper.tidy_module import TidyArm
    recs = [{"decision": "confirmed", "x": 1.0, "y": 2.0}, {"decision": "unsure", "x": 3.0, "y": 0.0}]
    assert TidyArm._checked(N(), recs) == recs and TidyArm._checked_at_end(N()) == []
    assert TidyArm.CANDIDATE_CHECK is False
