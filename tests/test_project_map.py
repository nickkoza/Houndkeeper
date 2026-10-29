"""[KfL]: the project map's check before anyone sees it ([NiP]), the refusal of what git ignores ([Nlv]), and
Barrymore in the map's own words ([EZt]). Synthetic pages and values only: nothing here reads real/.env."""
import importlib.util
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("project_map", HK / "tools" / "project_map.py")
pm = importlib.util.module_from_spec(spec); sys.modules["project_map"] = pm; spec.loader.exec_module(pm)

CLEAN = '<html><title>Houndkeeper project map</title><script id="map-data" type="application/json">{"v":"Filament 1.77.1, ' \
        'a 0.17-0.19% gap, 12:30 on 3 Oct"}</script></html>'


def test_a_clean_page_carries_nothing():
    assert pm.leaks(CLEAN) == {}


def test_each_kind_is_found_and_counted():
    page = CLEAN + " 192.0.2.10 and 198.51.100.7, 00:1A:2B:3C:4D:5E, fe80:0:0:0:0:0:0:1, someone@example.org"
    assert pm.leaks(page) == {"network address": 3, "hardware address": 1, "email address": 1}


def test_values_are_counted_and_never_reported(tmp_path, capsys):
    env = tmp_path / ".env"; env.write_text("# a comment\nSHORT=yes\nNAME='not-a-real-value-123'\nEMPTY=\n")
    local = tmp_path / "local"; local.mkdir(); (local / "notes.txt").write_text("ok\nan invented line for the test\n")
    secrets = {"value from real/.env": pm.env_values(env), "line from real/local": pm.local_lines(local)}
    assert secrets["value from real/.env"] == ["not-a-real-value-123"]          # [QCx]: "yes" is too short to count
    page = CLEAN + " not-a-real-value-123 ... an invented line for the test"
    found = pm.leaks(page, secrets)
    assert found == {"value from real/.env": 1, "line from real/local": 1}
    assert "not-a-real-value-123" not in repr(found) and "invented" not in repr(found)


def test_a_value_the_repository_already_carries_does_not_count():
    page = CLEAN + " a generic how-to line, an invented private line"
    secrets = {"line from real/local": ["a generic how-to line", "an invented private line"]}
    assert pm.leaks(page, secrets, known="the how-to says: a generic how-to line, in a tracked file") == {"line from real/local": 1}


def test_a_page_that_leaks_is_removed_and_the_report_names_only_kinds(tmp_path, monkeypatch, capsys):
    out = tmp_path / "map"; out.mkdir()
    def fake_run(cmd, **kw):
        staged = Path(cmd[cmd.index("--out") + 1]); staged.mkdir(parents=True)
        (staged / "index.html").write_text(CLEAN + " 192.0.2.10 someone@example.org")
        return type("R", (), {"returncode": 0, "stdout": "", "stderr": ""})()
    method = tmp_path / "method"; (method / "bin").mkdir(parents=True); (method / "bin" / "railyard-map.ts").write_text("")
    monkeypatch.setenv("HK_RAILYARD_METHOD", str(method))
    monkeypatch.setattr(pm.subprocess, "run", lambda cmd, **kw: fake_run(cmd, **kw) if cmd[0] == "node" else
                        type("R", (), {"returncode": 1, "stdout": "", "stderr": ""})())
    monkeypatch.setattr(pm, "env_values", lambda *a: []); monkeypatch.setattr(pm, "local_lines", lambda *a: [])
    assert pm.make(out) == 1
    assert not (out / "index.html").exists() and list(out.iterdir()) == []
    err = capsys.readouterr().err
    assert "1 network address" in err and "1 email address" in err and "192.0.2.10" not in err


def test_a_named_path_git_ignores_is_refused_before_the_map_is_made(tmp_path, capsys):
    assert pm.make(tmp_path / "map", dated=("real/local/",)) == 2
    assert "git ignores" in capsys.readouterr().err
    assert not (tmp_path / "map").exists()


def test_the_maps_own_words_name_the_robot_barrymore():
    page = CLEAN.replace("project map", "project map, Edgar's register")
    assert "Edgar" in pm.own_words(page)
    assert "Edgar" not in pm.own_words(CLEAN.replace("Filament", "Edgar, in an artifact,"))


def test_a_value_escaped_in_the_pages_data_is_still_found():
    page = '<script id="map-data" type="application/json">{"t":"say \\"pass-word-xyz\\" now"}</script>'
    assert pm.leaks(page, {"value from real/.env": ['say "pass-word-xyz"']}) == {"value from real/.env": 1}
