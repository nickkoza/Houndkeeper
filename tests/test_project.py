"""[RzM] [eTc] [Dlx]: the project service (houndkeeper.project) and its launcher (houndkeeper.project_stack): the Last
night message's shape from a night's files and the findings, what failed, the bound ([WkQ]), no hold-out opened
([jnG]); the service publishing with no robot ([E2e]); bound only to loopback or the tailnet ([0Us]); and a browser smoke
at a phone's width with only a loopback relay and the project service running."""
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "src"))
pytest.importorskip("dimos")                                                   # no DimOS here: skip; a broken module: fail
import houndkeeper.project as pj  # noqa: E402
import houndkeeper.project_stack as ps  # noqa: E402

PROSE = ["Good morning. I set out at 9:46 pm and was done by 10:23 pm.", "A plastic object I put in the bin."]


def night(root, stamp, seed, score=True, morning=True, report=True, energy=True, stops=19, **score_kw):
    d = root / stamp; d.mkdir(parents=True)
    n = {"seed": seed, "mode": "confirm", "item_seed": 1, "round": None, "started": 1791089212, "ended": 1791091392}
    if seed is not ...: (d / "night.json").write_text(json.dumps(n))
    if score:
        s = {"items": 8, "binned": 1, "found": 2, "stops": stops, "stops_for_nothing": 16, "state": "done", "error": None,
             "t_s": 1041.9, "binnable": 7, "outcomes": {"review: not seen standing": 14, "binned": 1, "review: not seen at the flag": 3},
             "tiers": {"hard": [1, 2], "easy": [0, 5]}, "waypoints_not_reached": [{"waypoint": 1, "not_reached": "gave up"}],
             "speed": {"sim_wall": 0.48, "latency_flattered": True, "source": "log"}}
        s.update(score_kw); (d / "score.json").write_text(json.dumps(s))
    if report:
        (d / "report.json").write_text(json.dumps({"state": "done", "items": [{"outcome": "binned"}, {"outcome": "review: not seen standing"},
                                                                               {"outcome": "review: not seen standing"}],
                                                   "sweep": [{"waypoint": 2, "not_reached": "blocked"}]}))
    if energy: (d / "energy.json").write_text(json.dumps({"air_used_Wh": 44.3, "air_left_pct": 81.3}))
    if morning:
        (d / "morning_log.json").write_text(json.dumps({
            "prose": PROSE, "pictures_note": "", "finds": [
                {"k": 1, "what": "a plastic object", "fate": "put_away", "clock": "9:53 pm"},
                {"k": None, "what": "something", "fate": "nothing", "clock": "9:47 pm"}],
            "facts": {"minutes": {"text": "36", "source": "report.json"}, "battery_left_pct": {"text": "81", "source": "energy.json"}}}))
    return d


FINDING_ANSWER = """# A finding with an answer (4 Oct 2026)

Some lead-in.

## What ran

Things.

## Answer

**Yes**, by 12 points: see `table.md`.

| a | b |
|---|---|
| 1 | 2 |

## Caveats

Some.
"""
FINDING_LEAD = "# Lead only (3 Oct 2026)\n\nThe lead paragraph, the answer.\n\n@@RESULTS_SUMMARY@@\n\n## What ran\n\nStuff.\n"


def findings(root):
    root.mkdir(parents=True, exist_ok=True)
    (root / "old.md").write_text(FINDING_LEAD); os.utime(root / "old.md", (1e9, 1e9))
    (root / "new.md").write_text(FINDING_ANSWER)
    return root


def test_the_message_has_each_night_its_numbers_with_sources_what_failed_its_morning_log_and_the_findings(tmp_path):
    runs = tmp_path / "runs"; night(runs, "20261003_214551", 72); night(runs, "20261003_222437", 73, score=False)
    m = pj.last_night_message(runs, findings(tmp_path / "findings"))
    assert m["v"] == 1 and [n["id"] for n in m["nights"]] == ["20261003_222437", "20261003_214551"] and m["trimmed"] == []
    scored = m["nights"][1]; nums = {x["label"]: x for x in scored["numbers"]}
    assert scored["house"] == 72 and scored["scored_against_truth"] and scored["counts_from"] == "score.json"
    assert nums["Found"]["text"] == "2 of 8" and nums["Binned"]["text"] == "1 of 7" and nums["False stops"]["text"] == "16 of 19"
    assert nums["Battery left"]["text"] == "81 %" and nums["Battery left"]["source"] == "energy.json"
    assert nums["Time"]["text"] == "17 min" and nums["Sim speed"]["text"] == "0.48x real time" and nums["Sim speed"]["note"] == "latency-flattered"
    assert all(x["source"] for x in scored["numbers"])
    assert "Waypoint 1 not reached: gave up" in scored["failed"] and "14 stops: not seen standing" in scored["failed"]
    assert "easy items: 0 of 5 binned" in scored["failed"] and not any("binned" == f for f in scored["failed"])
    mo = scored["morning"]
    assert mo["prose"] == PROSE and [f["k"] for f in mo["finds"]] == [1] and {f["source"] for f in mo["figures"]} == {"report.json", "energy.json"}
    own = m["nights"][0]                                        # no score: the robot's own report, no truth
    assert own["counts_from"] == "report.json" and not own["scored_against_truth"]
    assert {x["label"] for x in own["numbers"]} >= {"Stops", "Binned"} and "False stops" not in {x["label"] for x in own["numbers"]}
    assert "2 stops: not seen standing" in own["failed"] and "Waypoint 2 not reached: blocked" in own["failed"]
    f_new, f_old = m["findings"]
    assert f_new["name"] == "new.md" and f_new["section"] == "Answer" and f_new["excerpt"].startswith("**Yes**")
    assert "| 1 | 2 |" in f_new["excerpt"] and "Caveats" not in f_new["excerpt"]
    assert not f_new["full"].startswith("# ") and "## Caveats" in f_new["full"]           # the whole text, title above it
    assert f_old["section"] == "" and f_old["excerpt"] == "The lead paragraph, the answer."   # no Answer: the lead
    assert "@@" not in f_old["full"]


def test_a_run_that_never_got_going_says_so(tmp_path):
    d = night(tmp_path, "20261003_200750", 96, stops=0, t_s=0.7, outcomes={}, waypoints_not_reached=[])
    (d / "status.json").write_text(json.dumps({"log": ["0.1s park", "0.7s sweep: no house lidar map"]}))
    fails = pj.night_summary(d, 96)["failed"]
    assert any(f.startswith("No stops at all, over after 0.7 s") and "no house lidar map" in f for f in fails)
    assert {x["label"]: x for x in pj.night_summary(d, 96)["numbers"]}["Time"]["text"] == "0.7 s"


def test_hold_out_nights_and_nights_that_name_no_house_are_passed_over_unopened(tmp_path, monkeypatch):
    runs = tmp_path / "runs"; night(runs, "20261001_000000", 72)
    for i, seed in enumerate((80, 81, 100, 119)): night(runs, f"2026100{i + 2}_000000", seed)
    night(runs, "20261009_000000", ...)                            # before night.json: no house named
    opened = []; real = pj._load
    monkeypatch.setattr(pj, "_load", lambda p: opened.append(Path(p)) or real(p))
    m = pj.last_night_message(runs, tmp_path / "none")
    assert [n["house"] for n in m["nights"]] == [72]
    held = {p for p in opened if p.parent.name != "20261001_000000"}
    assert {p.name for p in held} == {"night.json"}                 # of a hold-out, night.json alone
    assert not any("20261009" in str(p) for p in opened)


def test_the_hold_out_list_is_the_registrys():
    sys.path.insert(0, str(HK / "sim" / "house"))
    import make_house
    assert pj.HOLDOUT == frozenset(make_house.HOLDOUT)


def test_the_message_keeps_within_its_bound_dropping_whole_texts_then_older_nights(tmp_path):
    runs = tmp_path / "runs"
    for i in range(6): night(runs, f"2026100{i + 1}_000000", 70 + i)
    fd = tmp_path / "findings"; fd.mkdir()
    for i in range(4): (fd / f"f{i}.md").write_text(f"# F{i}\n\n" + ("A long line of findings text. " * 40 + "\n\n") * 30)
    full = pj.last_night_message(runs, fd)
    assert len(full["nights"]) == pj.NIGHTS and all(f["full"] for f in full["findings"])
    bound = pj._size(full) - 30_000
    m = pj.last_night_message(runs, fd, max_bytes=bound)
    assert pj._size(m) <= bound and m["trimmed"] and m["trimmed"][0].startswith("whole text of")
    assert m["nights"][0]["id"] == full["nights"][0]["id"] and m["nights"][0]["morning"]       # the newest kept whole
    tiny = pj.last_night_message(runs, fd, max_bytes=6_000)
    assert pj._size(tiny) <= 6_000 and len(tiny["nights"]) <= 1
    assert pj._size(pj.last_night_message(runs, fd, max_bytes=300)) <= 300


def test_the_write_ups_are_the_research_folders_and_its_index_is_not_one(tmp_path):
    """[RzM]: the newest write-ups come from docs/research; its README is the folder's index, not a write-up"""
    assert pj.FINDINGS == HK / "docs" / "research"
    fd = findings(tmp_path / "research"); (fd / "README.md").write_text("# Research write-ups\n\nWhat these are.\n")
    titles = [f["title"] for f in pj.newest_findings(fd)]
    assert "Research write-ups" not in titles and len(titles) == 2


def test_with_no_night_and_no_findings_it_is_still_a_message(tmp_path):
    m = pj.last_night_message(tmp_path / "nothing", tmp_path / "nothing")
    assert m["nights"] == [] and m["findings"] == [] and m["v"] == 1


class FakeOut:
    def __init__(self): self.sent = []
    def publish(self, m): self.sent.append(m)


def feed(monkeypatch, runs, fd):
    monkeypatch.setenv("HK_PROJECT_RUNS", str(runs)); monkeypatch.setenv("HK_PROJECT_FINDINGS", str(fd))
    f = object.__new__(pj.ProjectFeed); f._seen = None; f._msg = None; f.last_night = FakeOut()
    return f


def test_the_service_publishes_with_no_robot_again_each_look_and_anew_when_a_file_changes(tmp_path, monkeypatch):
    runs = tmp_path / "runs"; d = night(runs, "20261003_214551", 72); fd = findings(tmp_path / "findings")
    f = feed(monkeypatch, runs, fd)
    a = f.check(); b = f.check()
    assert f.last_night.sent == [a, a] and b is a                    # published again unchanged: a late page gets it
    (d / "score.json").write_text(json.dumps({"items": 8, "binned": 3, "found": 3, "stops": 4, "stops_for_nothing": 1}))
    os.utime(d / "score.json", (2e9, 2e9))
    c = f.check()
    assert c is not a and {x["label"]: x for x in c["nights"][0]["numbers"]}["Binned"]["text"].startswith("3 of")


def test_the_service_needs_no_robot_stream_and_sends_nothing():
    """[eTc] [Dlx] [oWO]: the blueprint is the cockpit's bridge and the feeds; the Last night feed has no input and its
    page sends nothing; the only channels a page can send on are the scoring pages' (a confirmed save of the marks, a
    judgement), which their feed writes beside the real night and logs, never to a robot"""
    bridge, fd, sf = pj.project.blueprints
    assert fd.module is pj.ProjectFeed and all(s.direction == "out" for s in fd.streams)
    assert sf.module is pj.ScoreFeed and {s.name for s in sf.streams if s.direction == "in"} == {"marks_cmd", "review_cmd"}
    man = bridge.kwargs["manifest"]
    assert sorted((c["ch"], c["dir"]) for c in man["channels"]) == [("last_night", "rx"), ("marks_cmd", "tx"), ("marks_state", "rx"),
                                                                   ("review_cmd", "tx"), ("review_state", "rx")]
    assert [p["kind"] for p in man["panels"]] == ["hk-last-night", "hk-marks", "hk-review"]


# ---- the launcher ([0Us]) ----

@pytest.mark.parametrize("host,ok", [("127.0.0.1", True), ("localhost", True), ("::1", True), ("100.101.102.103", True),
                                     ("fd7a:115c:a1e0::1", True), ("192.168.1.20", False), ("10.0.0.5", False),
                                     ("172.16.0.2", False), ("0.0.0.0", False), ("::", False), ("100.128.0.1", False),
                                     ("laptop.example.ts.net", False), ("8.8.8.8", False)])
def test_only_loopback_and_tailscale_addresses_are_allowed(host, ok):
    assert ps.allowed_host(host) is ok


def test_the_relay_is_refused_on_a_house_address_and_off_loopback_without_its_certificate_and_auth(tmp_path):
    for bad in ("192.168.1.20", "0.0.0.0", "::"):
        with pytest.raises(SystemExit, match="refused"): ps.relay_argv(bad, tls=tmp_path, auth=tmp_path / "auth.json")
    lo = ps.relay_argv("127.0.0.1", tls=tmp_path, auth=tmp_path / "auth.json")
    assert "--cert" not in lo and lo[lo.index("--host") + 1] == "127.0.0.1" and "--extensions-dir" in lo
    with pytest.raises(SystemExit, match="certificate and auth"): ps.relay_argv("100.101.102.103", tls=tmp_path, auth=tmp_path / "auth.json")
    for n in ("cockpit.crt", "cockpit.key", "auth.json"): (tmp_path / n).write_text("x")
    ts = ps.relay_argv("100.101.102.103", tls=tmp_path, auth=tmp_path / "auth.json")
    assert ts[ts.index("--auth-file") + 1] == str(tmp_path / "auth.json") and "--cert" in ts and "--key" in ts


def test_the_projects_key_goes_into_its_environment_never_its_command_line(tmp_path):
    auth = tmp_path / "auth.json"; auth.write_text(json.dumps({"robots": {"barrymore": "k1" * 16, "project": "k2" * 16}}))
    url = "https://laptop.example.ts.net:7780"
    env = ps.project_env(url, auth, env={}); argv = ps.project_argv(url, dimos="dimos")
    assert env["RELAY_KEY"] == "k2" * 16 and not any("k2" in a for a in argv)
    assert argv[argv.index("--robot-id") + 1] == "project" and argv[-1] == "houndkeeper.project"
    assert "RELAY_KEY" not in ps.project_env("http://127.0.0.1:7780", auth, env={})
    auth.write_text(json.dumps({"robots": {"barrymore": "k1" * 16}}))
    with pytest.raises(SystemExit, match="no robot key"): ps.project_env(url, auth, env={})


# ---- the browser smoke ([RzM] [eTc]) ----

def _smoke_tools():
    deno = shutil.which("deno") or str(Path.home() / ".local" / "bin" / "deno")
    web = ps.DIMOS / "web"
    need = [Path(deno), web / "cockpit" / "dist" / "index.html", HK / "real" / "tests" / "node_modules" / "playwright",
            Path(sys.executable).with_name("dimos")]
    missing = [str(p) for p in need if not p.exists()] + ([] if shutil.which("node") else ["node"])
    return deno, missing


def test_the_page_at_phone_width_with_only_a_loopback_relay_and_the_project_service(tmp_path):
    """starts the relay on loopback (a free port) and the project service with no robot or sim (on a private zenoh
    discovery bus, so a running sim stack does not collide with it), then
    tests/browser/last_night_390.mjs at 390 px. Screenshots: HK_SHOTS_DIR, else this test's tmp dir"""
    deno, missing = _smoke_tools()
    if missing: pytest.skip("the browser smoke needs " + ", ".join(missing))
    if not pj.RUNS.is_dir(): pytest.skip("no nights here")
    shots = Path(os.environ.get("HK_SHOTS_DIR") or tmp_path / "shots")
    relay_cmd = ps.relay_argv("127.0.0.1", 0, deno)
    relay = subprocess.Popen(relay_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=HK, start_new_session=True)
    proj = None
    try:
        port = None; t0 = time.time()
        while time.time() - t0 < 60 and port is None:
            line = relay.stdout.readline()
            if line.startswith("{"): port = json.loads(line).get("httpPort")
        assert port, "relay not ready"
        url = f"http://127.0.0.1:{port}"
        log = open(tmp_path / "project.log", "w")
        proj = subprocess.Popen(ps.project_argv(url), env={**ps.project_env(url), "ZENOH_SCOUT_ADDR": "224.0.0.233:7446"}, stdout=log, stderr=subprocess.STDOUT, cwd=HK,
                                start_new_session=True)
        t0 = time.time()
        while time.time() - t0 < 120 and "relay bridge up" not in (tmp_path / "project.log").read_text():
            assert proj.poll() is None, (tmp_path / "project.log").read_text()[-2000:]
            time.sleep(1)
        r = subprocess.run(["node", str(HK / "tests" / "browser" / "last_night_390.mjs"), url + "/", str(shots)],
                           capture_output=True, text=True, timeout=180)
        out = json.loads(r.stdout.strip().splitlines()[-1])
        assert out["ok"], out
        assert "Last night" in out["text"] and "Findings, newest first" in out["text"] and "What failed" in out["text"]
        assert len(out["shots"]) >= 2 and all(Path(p).exists() for p in out["shots"])
    finally:
        for p in (proj, relay):
            if p is not None and p.poll() is None:
                os.killpg(p.pid, signal.SIGTERM)
                try: p.wait(20)
                except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL)


def test_the_pages_recorded_message_is_a_last_night_message(tmp_path):
    """[RzM]: tests/fixtures/last_night_message.json, which the page's tests render from, has the shape
    last_night_message makes (each key, a night's, a number's, a morning log's and a finding's)"""
    fix = json.loads((HK / "tests" / "fixtures" / "last_night_message.json").read_text())
    runs = tmp_path / "runs"; night(runs, "20261003_214551", 72)
    m = pj.last_night_message(runs, findings(tmp_path / "findings"))
    assert set(fix) == set(m) and fix["v"] == m["v"]
    assert set(fix["nights"][0]) == set(m["nights"][0]) and set(fix["nights"][0]["numbers"][0]) == set(m["nights"][0]["numbers"][0])
    assert set(fix["nights"][0]["morning"]) == set(m["nights"][0]["morning"]) and set(fix["findings"][0]) == set(m["findings"][0])
