"""[HqQ]: one catalog of every recording, found and used by id; real ones moved into the data store, copied and
verified before any local removal; unreferenced sim ones pruned, real ones never; frames refused until screened;
nothing outside the house."""
import importlib
import time
import json
import sqlite3
import sys
from pathlib import Path

from houndkeeper import workspace

import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "src"))


def fake_recording(root: Path, run_id: str, n: int = 30, t0: float = 1_790_000_000.0) -> Path:
    """a memory.db shaped like DimOS's: _streams and a table per stream with ts"""
    d = root / run_id; d.mkdir(parents=True); p = d / "memory.db"
    c = sqlite3.connect(p)
    c.execute("create table _streams (name text primary key, config text not null)")
    for s, k in (("odom", n), ("color_image", n // 2)):
        c.execute("insert into _streams values (?, '{}')", (s,))
        c.execute(f'create table "{s}" (id integer primary key autoincrement, ts real not null, value numeric)')
        c.executemany(f'insert into "{s}" (ts, value) values (?, 0)', [(t0 + i * 0.1,) for i in range(k)])
    c.commit(); c.close(); return p


@pytest.fixture
def rec(tmp_path, monkeypatch):
    monkeypatch.setenv("HK_DATA_ROOT", str(tmp_path / "data"))
    monkeypatch.setenv("HK_DIMOS", str(tmp_path / "dimos"))
    import houndkeeper.recordings as r
    r = importlib.reload(r)
    monkeypatch.setattr(r, "references", lambda: "the night used 20261003-120000-unitree-go2 here")
    monkeypatch.setattr(r, "house_of", lambda rid: None)
    return r, tmp_path


def test_scan_catalogs_sim_recordings_with_their_streams_and_span(rec):
    r, tmp = rec
    fake_recording(tmp / "dimos" / "recordings", "20261003-120000-unitree-go2")
    fake_recording(tmp / "dimos" / "recordings", "20261003-130000-unitree-go2-agentic")
    assert len(r.scan()) == 2 and r.scan() == []                              # once only
    row = r.get("20261003-130000")
    assert row["kind"] == "sim" and row["blueprint"] == "unitree-go2-agentic" and row["screening"] == "n/a"
    assert json.loads(row["streams"]) == {"odom": 30, "color_image": 15} and abs(row["duration_s"] - 2.9) < 0.01
    assert row["params_sha"] and row["fork_patch_sha"] and row["hk_commit"]
    assert r.path_of("20261003-130000").name == "memory.db"


def test_a_real_recording_is_hashed_moved_into_the_store_and_starts_unscreened(rec):
    r, tmp = rec
    p = fake_recording(tmp / "dimos" / "recordings", "20261010-200000-unitree-go2")
    rid = r.add(p, real=True, purpose="day one: the still minute", tags=("calibration", "day-one"), firmware="1.1.15")
    row = r.get(rid); moved = Path(row["path"])
    assert row["kind"] == "real" and moved.parent == tmp / "data" / "real" / rid and moved.exists()
    assert not p.exists() and (moved.parent / "params-robot.yaml").exists()
    assert row["sha256"] == r.sha256_of(moved) and row["firmware"] == "1.1.15" and row["screening"] == "unscreened"
    assert [x["id"] for x in r.rows(real=True, tag="day-one")] == [rid] and r.rows(real=True, purpose="still")
    with pytest.raises(PermissionError): r.require_screened(rid)
    r.main(["screened", rid, "clear"]); r.require_screened(rid)


def test_sync_verifies_the_server_copy_and_only_then_allows_removing_the_laptops(rec, monkeypatch):
    r, tmp = rec
    rid = r.add(fake_recording(tmp / "dimos" / "recordings", "20261010-210000-unitree-go2"), real=True)
    with pytest.raises(SystemExit): r.drop_local(rid)                         # no server copy yet
    server = tmp / "server"; monkeypatch.setattr(r, "server_dest", lambda: str(server))
    assert r.sync() == [rid] and (server / rid / "memory.db").exists()
    assert r.verify() == []
    (server / rid / "memory.db").write_bytes(b"corrupt")
    assert any("server: changed" in p for p in r.verify())
    with pytest.raises(SystemExit): r.drop_local(rid)                         # the server's copy no longer matches
    r.sync(); r.drop_local(rid)
    assert r.get(rid)["path"].startswith(str(server)) and not (tmp / "data" / "real" / rid).exists()


def test_prune_lists_only_unreferenced_sim_recordings(rec):
    r, tmp = rec
    recs = tmp / "dimos" / "recordings"
    fake_recording(recs, "20261003-120000-unitree-go2")                      # named by a run: kept
    fake_recording(recs, "20261003-140000-unitree-go2")                      # nothing names it: prunable
    fake_recording(recs, "20261003-150000-unitree-go2")                      # tagged keep
    r.scan(); r.main(["tag", "20261003-150000", "keep"])
    rid = r.add(fake_recording(recs, "20261010-220000-unitree-go2"), real=True)
    assert [x["id"] for x in r.prunable()] == ["20261003-140000-unitree-go2"]
    n, freed = r.prune(apply=True)
    assert n == 1 and freed > 0 and not (recs / "20261003-140000-unitree-go2").exists() and r.get(rid)


def test_nothing_goes_outside_the_house(rec):
    r, _ = rec
    assert r.in_house("/mnt/server/recordings") and r.in_house("localhost:/srv/recordings")
    assert r.in_house("nick@10.9.8.20:/srv/recordings")
    assert not r.in_house("nick@8.8.8.8:/srv") and not r.in_house("nick@100.71.0.5:/srv")       # not the internet, not the tailnet
    with pytest.raises(SystemExit): r.sync("nick@8.8.8.8:/srv")


def test_dimos_cloud_upload_points_nowhere_in_the_checkout():
    env = workspace.dimos() / ".env"
    if not workspace.dimos().exists(): pytest.skip("no DimOS checkout beside the repo")
    assert env.exists() and "DIMOS_CLOUD_URL=http://127.0.0.1:9" in env.read_text()


def test_a_session_refuses_a_failed_or_stale_isolation_check_and_low_disk(rec, monkeypatch):
    r, tmp = rec
    monkeypatch.setattr(r, "HK", tmp); monkeypatch.setattr(r, "FREE_FLOOR_GB", 0.0)    # this machine's own disk is not the test
    assert any("no isolation check" in w for w in r.preflight(real=True))
    iso = tmp / "real" / "local" / "isolation.json"; iso.parent.mkdir(parents=True)
    iso.write_text(json.dumps({"summary": "FAILED: robot rules out of order"}))
    assert any("failed" in w for w in r.preflight(real=True))
    iso.write_text(json.dumps({"summary": "passed: 9 rules, 0 escapes"}))
    assert r.preflight(real=True) == [] and r.preflight(real=False) == []
    assert any("min old" in w for w in r.preflight(real=True, now=time.time() + 7200))
    monkeypatch.setattr(r, "FREE_FLOOR_GB", 1e9)
    assert any("GB free" in w for w in r.preflight(real=False))


def test_a_session_records_with_the_battery_and_catalogs_what_it_wrote(rec, monkeypatch):
    r, tmp = rec
    monkeypatch.setattr(r, "HK", tmp); monkeypatch.setattr(r, "preflight", lambda real: [])
    calls = []
    def fake_run(cmd, cwd=None):
        calls.append(cmd); fake_recording(tmp / "dimos" / "recordings", "20261011-200000-unitree-go2")
    monkeypatch.setattr(r.subprocess, "run", fake_run)
    added = r.session(True, "day one: battery walk", ("power",), "1.1.15", ["run", "unitree-go2", "--robot-ip", "x"])
    assert calls[0][1:4] == ["--record", "sqlite", "run"] and "houndkeeper.battery" in calls[0]
    assert added == ["20261011-200000-unitree-go2"] and r.get(added[0])["kind"] == "real"
    with pytest.raises(SystemExit): r.session(False, "", (), None, ["run", "unitree-go2"])


def test_ids_and_destinations_cannot_smuggle_commands_or_options(rec):
    r, tmp = rec
    with pytest.raises(ValueError): r.add(fake_recording(tmp / "dimos" / "recordings", "20261003-120000-x;touch pwned"), real=False)
    assert not r.in_house("-e sh -c x:/srv") and not r.in_house("--rsh=evil:/srv") and not r.in_house("host:/srv;rm -rf /")
    assert r.in_house("nick@localhost:/srv/recordings")


def test_the_path_of_an_unscreened_real_recording_is_refused_unless_for_numbers_only(rec, capsys):
    """[R1X]: `hk-data path` is how tools reach a recording's frames, so an unscreened real one is refused there;
    an analysis that shows and keeps no pixels (the gap report) asks for it as numbers only"""
    r, tmp = rec
    rid = r.add(fake_recording(tmp / "dimos" / "recordings", "20261010-230000-unitree-go2"), real=True)
    with pytest.raises(PermissionError): r.main(["path", rid])
    r.main(["path", "--numbers-only", rid]); assert capsys.readouterr().out.strip().endswith("memory.db")
    r.main(["screened", rid, "clear"]); r.main(["path", rid]); assert "memory.db" in capsys.readouterr().out


def test_sync_also_copies_sim_recordings_tagged_keep(rec, monkeypatch):
    """[Zw9]: a sim recording kept on purpose (a demo film's source) gets its server copy too; untagged sim ones don't"""
    r, tmp = rec
    recs = tmp / "dimos" / "recordings"
    fake_recording(recs, "20261003-160000-unitree-go2"); fake_recording(recs, "20261003-170000-unitree-go2")
    r.scan(); r.main(["tag", "20261003-160000", "keep"])
    server = tmp / "server"
    assert r.sync(str(server)) == ["20261003-160000-unitree-go2"] and (server / "20261003-160000-unitree-go2" / "memory.db").exists()


def test_a_sim_run_marked_by_its_script_stays_sim_whatever_it_streams(rec):
    """[YP8], [R1X]: a sim run with the phone cockpit carries a battery stream; the marker the sim scripts leave keeps it
    sim (it was moved into the real store, unscreened, on 3 Oct 2026); an unmarked battery-carrying run is still real"""
    import sqlite3 as sq
    r, tmp = rec
    recs = tmp / "dimos" / "recordings"
    for rid in ("20261003-214557-unitree-go2-tidy", "20261003-235900-unitree-go2"):
        p = fake_recording(recs, rid); c = sq.connect(p)
        c.execute("insert into _streams values ('battery', '{}')"); c.execute('create table "battery" (id integer primary key, ts real, value numeric)')
        c.commit(); c.close()
    (recs / "20261003-214557-unitree-go2-tidy" / r.SIM_MARKER).touch()
    r.scan()
    assert r.get("20261003-214557")["kind"] == "sim" and r.get("20261003-235900")["kind"] == "real"
