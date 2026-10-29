"""Red-team checks on the recorded data ([HqQ]) and the phone cockpit's data module ([Swt]): the ways the tools could
lose an irreplaceable real recording, let one past the people screening, send it outside the house, or let a page
write something into params/robot.yaml that the user never confirmed (the cockpit-data red team of 3 Oct 2026, in private notes)."""
import importlib
import json
import os
import sqlite3
import subprocess
import sys
from pathlib import Path

from houndkeeper import workspace

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "src")); sys.path.insert(0, str(HK / "real" / "calibration"))


def fake_recording(root: Path, run_id: str, n: int = 30, t0: float = 1_790_000_000.0, streams=("odom", "color_image")) -> Path:
    d = root / run_id; d.mkdir(parents=True, exist_ok=True); p = d / "memory.db"
    c = sqlite3.connect(p)
    c.execute("create table _streams (name text primary key, config text not null)")
    for s in streams:
        c.execute("insert into _streams values (?, '{}')", (s,))
        c.execute(f'create table "{s}" (id integer primary key autoincrement, ts real not null, value numeric)')
        c.executemany(f'insert into "{s}" (ts, value) values (?, 0)', [(t0 + i * 0.1,) for i in range(n)])
    c.commit(); c.close(); return p


@pytest.fixture
def rec(tmp_path, monkeypatch):
    monkeypatch.setenv("HK_DATA_ROOT", str(tmp_path / "data"))
    monkeypatch.setenv("HK_DIMOS", str(tmp_path / "dimos"))
    monkeypatch.delenv("HK_DATA_SERVER", raising=False)
    import houndkeeper.recordings as r
    r = importlib.reload(r)
    monkeypatch.setattr(r, "references", lambda: "")
    monkeypatch.setattr(r, "house_of", lambda rid: None)
    monkeypatch.setattr(r, "server_dest", lambda: None)
    return r, tmp_path


# ---- recordings: data loss ------------------------------------------------------------------------------------------

def test_drop_local_twice_never_removes_the_server_copy(rec, monkeypatch):
    """[Zw9]: drop-local removes the laptop's copy only; run again it must not delete the (now only) server copy"""
    r, tmp = rec
    rid = r.add(fake_recording(tmp / "dimos" / "recordings", "20261010-210000-unitree-go2"), real=True)
    server = tmp / "server"; monkeypatch.setattr(r, "server_dest", lambda: str(server))
    r.sync(); r.drop_local(rid)
    with pytest.raises(SystemExit): r.drop_local(rid)
    assert (server / rid / "memory.db").exists()


def test_drop_local_checks_the_copy_where_sync_put_it_not_a_path_relative_to_here(rec, monkeypatch):
    """[Zw9]: synced with --dest, DATA_SERVER unset, run from the store's own folder: the 'server' check must not
    find the laptop's own copy and call it verified"""
    r, tmp = rec
    rid = r.add(fake_recording(tmp / "dimos" / "recordings", "20261010-211000-unitree-go2"), real=True)
    server = tmp / "server"; r.sync(str(server))
    import shutil; shutil.rmtree(server)                                    # the mount is gone
    monkeypatch.chdir(tmp / "data" / "real")                                # <id>/memory.db exists right here: the laptop copy
    with pytest.raises(SystemExit): r.drop_local(rid)
    assert (tmp / "data" / "real" / rid / "memory.db").exists()


def test_adding_a_real_recording_moves_only_the_recording_not_its_neighbours(rec):
    """[u6u]: a memory.db not in its own DimOS run folder (Downloads, home) must not take the folder's other files"""
    r, tmp = rec
    downloads = tmp / "Downloads"; downloads.mkdir()
    (downloads / "tax-return.pdf").write_bytes(b"%PDF")
    p = fake_recording(tmp, "Downloads")                                     # a Downloads folder's memory.db
    with pytest.raises((SystemExit, ValueError)): r.add(p, real=True)
    assert (downloads / "tax-return.pdf").exists() and p.exists()


def test_scan_never_catalogs_a_battery_recording_as_disposable_sim(rec):
    """[YP8] [R1X]: a run that carries the robot's battery stream (real-only, [4hm]) recorded with plain
    `dimos --record` is real: not prunable and not screening 'n/a'"""
    r, tmp = rec
    fake_recording(tmp / "dimos" / "recordings", "20261011-090000-unitree-go2", streams=("odom", "color_image", "battery"))
    r.scan()
    row = r.get("20261011-090000")
    assert row["kind"] == "real" and row["screening"] == "unscreened"
    assert [x["id"] for x in r.prunable()] == []
    with pytest.raises(PermissionError): r.require_screened(row["id"])


def test_scan_skips_a_recording_still_being_written(rec):
    """[B2O]: a run in progress (its db touched in the last minutes) is neither cataloged nor pruned"""
    r, tmp = rec
    fake_recording(tmp / "dimos" / "recordings", "20261011-100000-unitree-go2")      # mtime: now
    assert r.scan(min_age_s=600) == []


def test_the_remote_hash_command_quotes_the_path(rec, monkeypatch):
    """[fya]: ssh joins its arguments into a shell line on the server; a recording id must not become a command"""
    r, _ = rec
    seen = {}
    def fake_run(cmd, **kw):
        seen["cmd"] = cmd
        return subprocess.CompletedProcess(cmd, 1, "", "")
    monkeypatch.setattr(r.subprocess, "run", fake_run)
    r._remote_sha("nick@10.9.8.20:/srv/rec", "x;touch pwned/memory.db")
    remote = " ".join(seen["cmd"][2:])
    assert "'/srv/rec/x;touch pwned/memory.db'" in remote


def test_sync_refuses_a_destination_that_rsync_would_read_as_an_option(rec):
    r, _ = rec
    with pytest.raises(SystemExit): r.sync("--rsh=sh -c 'curl example.com'")


# ---- recordings: leaving the house ----------------------------------------------------------------------------------

@pytest.mark.xfail(strict=True, reason="DimOS reads .env from the working directory: outside the checkout the cloud "
                                       "URL is Dimensional's (the cockpit-data red team of 3 Oct 2026, D1)")
def test_dimos_cloud_points_nowhere_from_any_working_directory(tmp_path):
    py = workspace.dimos() / ".venv" / "bin" / "python"
    if not py.exists(): pytest.skip("no DimOS checkout beside the repo")
    env = {k: v for k, v in os.environ.items() if k not in ("PYTEST_VERSION", "DIMOS_CLOUD_URL")}
    out = subprocess.run([str(py), "-c", "from dimos.core.global_config import GlobalConfig as G; print(G().dimos_cloud_url)"],
                         cwd=tmp_path, env=env, capture_output=True, text=True, timeout=120).stdout.strip().splitlines()[-1]
    assert out.startswith("http://127.0.0.1")


# ---- the cockpit's procedures ---------------------------------------------------------------------------------------

cd = pytest.importorskip("houndkeeper.cockpit_data")


def harness(tmp_path, monkeypatch):
    import measure
    params = tmp_path / "robot.yaml"
    params.write_text("go2_air:\n  walk_power_W:      {value: 170,   source: estimate, note: \"est.\"}\n"
                      "  video_stream:      {value: {drop: 0.03, stall_per_min: 2, stall_s: 0.6}, source: estimate, note: \"est.\"}\n")
    (tmp_path / "params").mkdir(); (tmp_path / "params" / "robot.yaml").symlink_to(params)
    monkeypatch.setattr(measure, "PARAMS", params); monkeypatch.setattr(cd, "HK", tmp_path)
    monkeypatch.setattr(cd, "RUNS_LOG", tmp_path / "runs.jsonl")
    m = object.__new__(cd.CockpitData)
    m._buf = cd.Buffers(); m._state = cd.ProcedureState(); m._t0 = 0.0; m._truth = {}; m._pending = {}; m._frame_n = 0
    return m, params


def stream_run(m, t0, seed=0):
    m.handle({"op": "start", "id": "stream"}, now=t0)
    t = np.arange(t0, t0 + 300, 1 / 15); keep = np.random.default_rng(seed).random(len(t)) > 0.04
    m._buf.frames.extend(t[keep].tolist())
    m.handle({"op": "stop"}, now=t0 + 300)


def test_a_write_names_the_run_the_user_confirmed(tmp_path, monkeypatch):
    """[vRJ]: a write from a stale tab (or another viewer) that confirmed an earlier run must not write a later run's
    numbers; a write without the run is refused"""
    m, params = harness(tmp_path, monkeypatch)
    stream_run(m, 1000.0); first = m._state.run
    m.handle({"op": "reset"}); stream_run(m, 2000.0, seed=1)
    assert m._state.run != first
    m.handle({"op": "write", "run": first})
    assert "measured" not in params.read_text()
    m.handle({"op": "write"})
    assert "measured" not in params.read_text()
    m.handle({"op": "write", "run": m._state.run})
    assert "measured" in params.read_text()


def test_a_result_that_is_not_a_finite_number_is_never_offered_for_writing(tmp_path, monkeypatch):
    """[vRJ]: frames all stamped alike give an infinite drop rate; it must not reach params/robot.yaml"""
    m, params = harness(tmp_path, monkeypatch)
    m.handle({"op": "start", "id": "stream"}, now=1000.0)
    m._buf.frames.extend([1000.0] * 150)
    m.handle({"op": "stop"}, now=1010.0)
    assert m._state.writes == []
    m.handle({"op": "write", "run": m._state.run})
    assert "measured" not in params.read_text()


def test_entered_truth_must_be_finite_and_plausible(tmp_path, monkeypatch):
    m, _ = harness(tmp_path, monkeypatch)
    m.handle({"op": "start", "id": "taped-square"}, now=0.0)
    m._buf.odom.extend([(float(i), 0.01 * i, 0.0, 0.0) for i in range(100)])
    m.handle({"op": "stop"}, now=100.0)
    m.handle({"op": "truth", "forward_cm": "nan", "left_cm": 1e300, "heading_deg": "inf"}, now=101.0)
    assert m._truth == {} and m._state.result == {}


def test_the_status_page_gets_a_string_and_a_stale_pass_is_not_a_pass(tmp_path, monkeypatch):
    """[F2q]: an isolation record whose summary is not a string must not break the page; an old pass shows as old"""
    m, _ = harness(tmp_path, monkeypatch)
    iso = tmp_path / "real" / "local" / "isolation.json"; iso.parent.mkdir(parents=True)
    iso.write_text(json.dumps({"summary": {"passed": True}}))
    assert isinstance(m.link(1e9).isolation, str)
    iso.write_text(json.dumps({"summary": "passed: 0 leaks"})); os.utime(iso, (1000.0, 1000.0))
    assert not m.link(1000.0 + 3 * 3600).isolation.startswith("passed")
