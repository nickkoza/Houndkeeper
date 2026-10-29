"""Tests written to kill mutants that survived the suite (docs/research/test-quality-2026-10-03.md): the recordings' guards
that no earlier test reached: a flagged recording's frames, a copy outside the store, a 'server' copy that is the
laptop's file, a run folder holding something else, a robot recording added as sim, a host with a public address."""
import importlib
import socket
import sqlite3
import sys
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "src"))


def fake_recording(root: Path, run_id: str, streams=("odom", "color_image"), n: int = 20) -> Path:
    d = root / run_id; d.mkdir(parents=True, exist_ok=True); p = d / "memory.db"
    c = sqlite3.connect(p); c.execute("create table _streams (name text primary key, config text not null)")
    for s in streams:
        c.execute("insert into _streams values (?, '{}')", (s,))
        c.execute(f'create table "{s}" (id integer primary key autoincrement, ts real not null, value numeric)')
        c.executemany(f'insert into "{s}" (ts, value) values (?, 0)', [(1_790_000_000.0 + i * 0.1,) for i in range(n)])
    c.commit(); c.close(); return p


@pytest.fixture
def rec(tmp_path, monkeypatch):
    monkeypatch.setenv("HK_DATA_ROOT", str(tmp_path / "data")); monkeypatch.setenv("HK_DIMOS", str(tmp_path / "dimos"))
    monkeypatch.delenv("HK_DATA_SERVER", raising=False)
    import houndkeeper.recordings as r
    r = importlib.reload(r)
    monkeypatch.setattr(r, "references", lambda: ""); monkeypatch.setattr(r, "house_of", lambda rid: None)
    monkeypatch.setattr(r, "server_dest", lambda: None)
    return r, tmp_path


def test_a_recording_flagged_for_people_is_refused_like_an_unscreened_one(rec):
    """[R1X] screened and found to show people: its frames are not used"""
    r, tmp = rec
    rid = r.add(fake_recording(tmp / "dimos" / "recordings", "20261010-200000-unitree-go2"), real=True)
    r.main(["screened", rid, "people"])
    with pytest.raises(PermissionError): r.require_screened(rid)


def test_a_robot_recording_cannot_be_added_as_sim(rec):
    """[YP8] a run carrying the battery stream is the real robot's: adding it as sim (prunable) is refused"""
    r, tmp = rec
    p = fake_recording(tmp / "dimos" / "recordings", "20261010-201000-unitree-go2", streams=("odom", "battery"))
    with pytest.raises(ValueError): r.add(p, real=False)


def test_a_run_folder_holding_anything_else_is_not_moved(rec):
    """[u6u] a run folder named like DimOS's that also holds someone's notes: nothing is moved"""
    r, tmp = rec
    p = fake_recording(tmp / "dimos" / "recordings", "20261010-202000-unitree-go2")
    (p.parent / "notes.txt").write_text("mine")
    with pytest.raises(ValueError): r.add(p, real=True)
    assert p.exists() and (p.parent / "notes.txt").exists() and not (tmp / "data" / "real").exists()


def test_prune_never_deletes_a_recording_outside_dimos_recordings(rec):
    """[B2O] a sim recording cataloged from elsewhere is listed as unreferenced but its folder is not removed"""
    r, tmp = rec
    p = fake_recording(tmp / "elsewhere", "20261003-160000-unitree-go2")
    r.add(p, real=False)
    n, _ = r.prune(apply=True)
    assert n == 1 and p.exists()


def _synced(r, tmp, monkeypatch, rid_name):
    rid = r.add(fake_recording(tmp / "dimos" / "recordings", rid_name), real=True)
    server = tmp / "server"; monkeypatch.setattr(r, "server_dest", lambda: str(server)); r.sync()
    return rid


def test_drop_local_refuses_a_laptop_copy_outside_the_store(rec, monkeypatch):
    """[Zw9] the catalog's laptop path pointed outside data/real (a hand edit, an old layout): nothing removed"""
    r, tmp = rec
    rid = _synced(r, tmp, monkeypatch, "20261010-203000-unitree-go2")
    other = tmp / "else" / rid; other.mkdir(parents=True)
    (other / "memory.db").write_bytes((tmp / "data" / "real" / rid / "memory.db").read_bytes())
    with sqlite3.connect(r.catalog_path()) as c:
        c.execute("update copies set path=? where id=? and location='laptop'", (str(other / "memory.db"), rid))
    with pytest.raises(SystemExit): r.drop_local(rid)
    assert (other / "memory.db").exists()


def test_drop_local_refuses_a_server_copy_that_is_the_laptops_own_file(rec, monkeypatch):
    """[Zw9] a 'server' copy recorded at the laptop's own path verifies against itself: refused"""
    r, tmp = rec
    rid = _synced(r, tmp, monkeypatch, "20261010-204000-unitree-go2")
    here = tmp / "data" / "real" / rid / "memory.db"
    with sqlite3.connect(r.catalog_path()) as c:
        c.execute("update copies set path=? where id=? and location='server'", (str(here), rid))
    with pytest.raises(SystemExit): r.drop_local(rid)
    assert here.exists()


def test_a_copy_that_did_not_match_is_not_marked_verified(rec, monkeypatch):
    """[Zw9] sync records a mismatched server copy unverified (so drop-local refuses before it hashes again)"""
    r, tmp = rec
    rid = r.add(fake_recording(tmp / "dimos" / "recordings", "20261010-205000-unitree-go2"), real=True)
    monkeypatch.setattr(r, "_remote_sha", lambda dest, rel: "0" * 64)
    server = tmp / "server"; r.sync(str(server))
    with sqlite3.connect(r.catalog_path()) as c:
        assert c.execute("select verified from copies where id=? and location='server'", (rid,)).fetchone()[0] is None


def test_a_host_with_any_public_address_is_outside_the_house(rec, monkeypatch):
    """[fya] a name that resolves to a private and a public address is not the house's own machine"""
    r, _ = rec
    monkeypatch.setattr(r.socket, "getaddrinfo", lambda host, port: [(socket.AF_INET, 0, 0, "", ("10.0.0.5", 0)),
                                                                     (socket.AF_INET, 0, 0, "", ("8.8.8.8", 0))])
    assert not r.in_house("nick@nas:/srv")
