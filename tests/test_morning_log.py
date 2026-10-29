"""[uoW]: the butler's morning log, built from a finished night's own outputs: no sim truth in the household's part,
every figure in his prose the recorded one, a picture per find only from a screened recording and through the people
screen, the night's figures linked to their sources, and a page that fetches nothing."""
import json
import re
import shutil
import sys
import time
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "src"))
ml = pytest.importorskip("houndkeeper.morning_log")
T0 = 1_790_748_517.0
RID = "20260929-230749-unitree-go2-mcp-server-houndkeeper-tidy"


def stamp(t):
    g = time.gmtime(t); return f"{g.tm_hour:02d}:{g.tm_min:02d}:{g.tm_sec + t % 1:06.3f}"


def night(tmp_path, items=None, kind="sim", screening="n/a", monkeypatch=None):
    """a finished night shaped like a tidy run's directory: three stops (a toy binned, nothing, a cup left), a close look
    at each find from straight above, a walk, the battery, the sim's truth beside them; its recording in a fake catalog"""
    d = tmp_path / "night"; (d / "looks").mkdir(parents=True); (d / "house_map").mkdir()
    items = items if items is not None else [
        {"flag": [1.0, 0.5], "label": "a toy", "standing": "standing look: a toy (0.4) 5 cm", "tries": 1, "park_err_cm": 4.0, "outcome": "binned"},
        {"flag": [2.0, -0.5], "standing": "standing look: nothing near the flag", "outcome": "review: not seen standing"},
        {"flag": [3.0, 1.0], "label": "a cup", "standing": "standing look: a cup (0.5) 3 cm", "tries": 1, "park_err_cm": 6.0,
         "outcome": "review: no reachable grip direction"}]
    (d / "report.json").write_text(json.dumps({"state": "done", "t0": T0, "items": items, "standing_looks": [], "balance": {"max_deg": 3.2}}))
    L = [(0.4, "SWEEP: waypoint 1/4")]; t = 10.0
    for n, it in enumerate(items, 1):
        L += [(t, f"GO TO FIND: find {n} at ({it['flag'][0]}, {it['flag'][1]}), confirmed on approach at 2 m"),
              (t + 5, f"LOOK STANDING: item {n}"), (t + 40, f"state: GRASP"), (t + 60 + 30 * n, f"item {n}: {it['outcome']}"),
              (t + 61 + 30 * n, f"SWEEP: waypoint {min(n + 1, 4)}/4")]; t += 120 + 30 * n
    L.append((t + 5, "DONE: all done"))
    (d / "dimos.log").write_text("".join(f"{stamp(T0 + s)} [inf][src/houndkeeper/tidy_module.py] [tidy] {x}\n" for s, x in L)
                                 + f"{stamp(T0 + 3)} [inf][dimos] something else entirely\n")
    s = np.linspace(0, 1, 400); x, y = 4 * s, np.sin(6 * s)                   # 4 m east, wiggling
    with open(d / "traj.csv", "w") as f:
        f.write("wall_t,sim_t,true_x,true_y,true_yaw,odom_x,odom_y,odom_yaw\n")
        for k in range(len(s)): f.write(f"{T0 + k},{k},{x[k] + 0.3},{y[k] - 0.2},0,{x[k]},{y[k]},0\n")
    (d / "energy.json").write_text(json.dumps({"air_used_Wh": 47.36, "air_Wh": 236.8, "air_left_pct": 80.0}))
    for n, it in enumerate(items, 1):
        if it.get("label"):
            img = np.full((360, 640, 3), 120, np.uint8)
            np.savez_compressed(d / "looks" / f"item{n}_look1.npz", **{"0_img": img, "0_c": np.array([it["flag"][0] + 0.1, it["flag"][1], 0.4]),
                                "0_R": np.eye(3), "0_fovy": np.array(50.0), "0_found_xy": np.array(it["flag"])})
    np.save(d / "house_map" / "house_map_upto99.npy", np.array([[-1, -2, 0.3], [5, 2, 0.3], [0, 1, 0.5]], np.float32))
    (d / "recording.txt").write_text(f"/x/dimos/recordings/{RID}/memory.db\n")
    (d / "truth.json").write_text(json.dumps({"bodies": {"toy_0": [9, 9, 0]}}))
    (d / "score.json").write_text(json.dumps({"items": 3, "found": 2, "binned": 1, "binnable": 2, "too_big": 1, "too_big_reported": 0,
                                              "stops_for_nothing": 1, "per_item": [{"item": "toy_0", "tier": "easy", "binned": True}]}))
    (d / "score.txt").write_text("score: {}\n")
    if monkeypatch is not None: catalog(monkeypatch, kind, screening)
    return d


def catalog(monkeypatch, kind, screening):
    rows = {RID: {"id": RID, "kind": kind, "screening": screening}}
    def get(rid):
        if rid not in rows: raise SystemExit(f"no recording matches {rid!r}")
        return rows[rid]
    monkeypatch.setattr(ml.recordings, "get", get); return rows


def spans(html):
    return re.findall(r'<span class="fact" data-fact="(\w+)">([^<]*)</span>', html)


def test_the_household_part_is_the_same_with_the_truth_files_gone(tmp_path, monkeypatch):
    """[9X9]: the log from a sim night's directory with its truth files absent (truth.json, score.json, score.txt, and
    traj.csv's true columns scrambled) has the same facts, prose and pictures as with them; a truth file is refused"""
    d = night(tmp_path, monkeypatch=monkeypatch)
    with pytest.raises(PermissionError): ml.Night(d).path("truth.json")
    with pytest.raises(PermissionError): ml.Night(d).path("score.json")
    a = json.loads(ml.write(d, tmp_path / "a").with_suffix(".json").read_text())
    for f in ml.TRUTH_FILES: (d / f).unlink()
    t = (d / "traj.csv").read_text().splitlines()
    (d / "traj.csv").write_text("\n".join([t[0]] + [",".join(r.split(",")[:2] + ["nan", "7", "1"] + r.split(",")[5:]) for r in t[1:]]) + "\n")
    b = json.loads(ml.write(d, tmp_path / "b").with_suffix(".json").read_text())
    assert a["facts"] == b["facts"] and a["prose"] == b["prose"] and a["finds"] == b["finds"] and a["route"] == b["route"]


def test_only_a_sim_night_adds_the_section_scored_against_truth_marked_as_such(tmp_path, monkeypatch):
    """[9X9]: a sim night may add a "scored against truth" section, marked; a real night never reads its truth"""
    d = night(tmp_path, monkeypatch=monkeypatch)
    from houndkeeper import morning_score as ms
    page = ms.write(d, tmp_path / "sim").read_text()
    assert "Scored against the simulation's truth" in page and "Barrymore knew none of this" in page
    assert "Scored against" not in ml.write(d, tmp_path / "plain").read_text()
    catalog(monkeypatch, "real", "clear")
    assert ms.scored_against_truth(ml.Night(d), "real") is None
    assert "Scored against" not in ms.write(d, tmp_path / "real").read_text()


def test_every_figure_in_his_prose_is_the_recorded_one(tmp_path, monkeypatch):
    """[W0w]: each figure in the prose is a fact's text, and each fact the night's record: stops from the report, the walk
    from odometry, the battery from energy.json, the hours from the report's t0 and the tidy's log; no other digit"""
    d = night(tmp_path, monkeypatch=monkeypatch)
    p = ml.write(d, tmp_path / "out"); page = p.read_text(); F = json.loads(p.with_suffix(".json").read_text())["facts"]
    used = spans(page.split('<section class="prose">')[1].split("</section>")[0])
    assert {k for k, _ in used} >= {"started", "ended", "minutes", "walked_m", "stops", "fate_nothing", "fate_put_away_list", "battery_used_pct"}
    for k, text in used: assert text.lower() == F[k]["text"].lower(), k
    prose = re.sub(r'<span class="fact"[^>]*>[^<]*</span>', "", page.split('<section class="prose">')[1].split("</section>")[0])
    assert not re.search(r"\d", prose)                                           # every number is a fact's
    assert F["stops"]["value"] == 3 and F["fate_nothing"]["value"] == 1 and F["fate_put_away"]["value"] == 1
    assert F["battery_used_pct"]["value"] == 20 and F["battery_left_pct"]["value"] == 80
    od = np.loadtxt(d / "traj.csv", delimiter=",", skiprows=1, usecols=[5, 6])
    assert abs(F["walked_m"]["value"] - np.hypot(*np.diff(od, axis=0).T).sum()) < 1.0
    assert F["started"]["text"] == ml.clock(T0) and F["minutes"]["value"] == round((F["ended"]["value"] - T0) / 60)
    assert "a toy" in F["fate_put_away_list"]["text"] and "a cup" in F["fate_no_grip_list"]["text"]


def test_an_empty_night_says_nothing_was_found_and_a_missing_one_is_refused(tmp_path, monkeypatch):
    """[W0w]: a night with no stops says so and claims no finds; a directory with no report is not an empty night"""
    d = night(tmp_path, items=[], monkeypatch=monkeypatch)
    F = json.loads(ml.write(d, tmp_path / "out").with_suffix(".json").read_text())
    text = " ".join(F["prose"])
    assert "found nothing" in text and "bin" not in text and F["facts"]["stops"]["value"] == 0
    with pytest.raises(FileNotFoundError): ml.write(tmp_path / "no-such-night", tmp_path / "x")


def test_one_crop_per_find_inside_its_picture(tmp_path, monkeypatch):
    """[hKP]: each find (not a stop that found nothing) gets one crop, inside its picture's bounds, with a time and a place
    on the map"""
    d = night(tmp_path, monkeypatch=monkeypatch)
    p = ml.write(d, tmp_path / "out"); F = json.loads(p.with_suffix(".json").read_text()); page = p.read_text()
    got = [f for f in F["finds"] if f["picture"]]
    assert [f["n"] for f in got] == [1, 3] and all(f["fate"] != "nothing" for f in got)
    for f in got:
        x0, y0, x1, y1 = f["picture"]["box"]; W, H = f["picture"]["image_wh"]
        assert 0 <= x0 < x1 <= W and 0 <= y0 < y1 <= H and f["clock"] and f["from_start_m"] is not None
    assert page.count('<div class="card"') == 2 and page.count("data:image/jpeg;base64,") == 2   # two crops, JPEG
    assert page.count("data:image/png;base64,") == 1                                               # and the map
    assert len(re.findall(r'<g class="find ', page)) == 2


def test_a_real_nights_crops_are_refused_until_screened_and_each_passes_the_people_screen(tmp_path, monkeypatch):
    """[hKP], [R1X]: a real night's pictures come only from a recording cleared for its frames; an unknown recording gets
    none; every crop goes through the people screen, and one it flags is never kept"""
    d = night(tmp_path, kind="real", screening="unscreened", monkeypatch=monkeypatch); N = ml.Night(d)
    H = ml.household(N); seen = []
    pics, why = ml.pictures(N, H["finds"], "real", screen=lambda im: seen.append(im) or True)
    assert pics == {} and "withheld" in why.lower() and seen == []
    rows = catalog(monkeypatch, "real", "people")
    assert ml.pictures(N, H["finds"], "real", screen=lambda im: True)[0] == {}
    rows[RID]["screening"] = "clear"
    pics, _ = ml.pictures(N, H["finds"], "real", screen=lambda im: seen.append(im) or True)
    assert sorted(pics) == [1, 3] and len(seen) == 2                            # the screen ran on every crop
    pics, why = ml.pictures(N, H["finds"], "real", screen=lambda im: False)
    assert pics == {} and "people screen" in why
    assert ml.screen_for("real") is ml.owl_people_screen and ml.screen_for("unknown") is ml.owl_people_screen
    (d / "recording.txt").write_text("/x/dimos/recordings/20990101-000000-elsewhere/memory.db\n")
    assert ml.kind_of(N.recording_id()) == "unknown" and ml.pictures(N, H["finds"], "sim")[0] == {}


def test_each_figure_below_the_prose_links_to_its_source_file(tmp_path, monkeypatch):
    """[6wJ]: under the prose, the night's figures (finds by kind, false alarms, distance, battery used, the slowest
    stage, ...) each link to the file it came from, and that file is the night's"""
    d = night(tmp_path, monkeypatch=monkeypatch)
    out = tmp_path / "elsewhere"; page = ml.write(d, out).read_text()
    eng = page.split("<summary>The night's figures</summary>")[1].split("</details>")[0]
    rows = re.findall(r'<tr><td>([^<]+)</td><td class="v">[^<]*</td><td><a href="([^"]+)">', eng)
    labels = {r[0] for r in rows}
    assert {"Finds by kind (his names)", "Stops for nothing (his own judgement)", "Walked (m, odometry)", "Battery used (%)", "Slowest stage"} <= labels
    for _, href in rows:
        f = (out / href).resolve(); assert f.exists() and f.is_relative_to(d.resolve()) and f.name not in ml.TRUTH_FILES


def test_the_page_is_self_contained(tmp_path, monkeypatch):
    """[Ndt] (the file beside the night): the page fetches nothing and runs no script, so it can only be served from where
    it sits; by default it is written into the night's own directory ([t0a])"""
    d = night(tmp_path, monkeypatch=monkeypatch)
    p = ml.write(d); page = p.read_text()
    assert p.parent == d and (d / "morning_log.json").exists()
    assert "<script" not in page and not re.search(r"(src|href)=\"(https?:)?//", page) and "@import" not in page
    assert "animation" not in page and "transition" not in page                  # nothing moves on the page


# ---- night.json: the night's names ----

def test_the_tidy_writes_night_json_with_its_names_and_route_and_the_log_reads_it(tmp_path, monkeypatch):
    """[9X9] [6wJ]: night.json holds the seed, mode, item seed, round, night_tidy folder and start and end times, with
    route.json copied in beside it; the log's figures name them, each from night.json or route.json"""
    d = night(tmp_path, monkeypatch=monkeypatch); nt = tmp_path / "night_tidy"; nt.mkdir()
    (nt / "route.json").write_text(json.dumps({"waypoints": [[0, 0], [1, 0], [1, 1]], "length_m": 2.0}))
    env = {"NIGHT_SEED": "70", "NIGHT_MODE": "confirm", "NIGHT_ITEM_SEED": "2", "NIGHT_ROUND": "", "NIGHT_TIDY_DIR": str(nt),
           "NIGHT_STARTED": str(int(T0)), "NIGHT_ENDED": str(int(T0) + 900), "NIGHT_ROUTE": str(nt / "route.json"),
           "HK_SIM_ITEMS": "[[\"toy\", \"toy_0\", [1, 2, 0]]]", "DIMOS_SIM_TRUTH_FILE": str(d / "truth.json")}
    ml.write_night_json(d, env)
    rec = json.loads((d / "night.json").read_text())
    assert rec == {"seed": 70, "mode": "confirm", "item_seed": 2, "round": None, "night_tidy": str(nt),
                   "started": int(T0), "ended": int(T0) + 900, "route": "route.json"}
    assert json.loads((d / "route.json").read_text())["length_m"] == 2.0
    fx = ml.household(ml.Night(d))["facts"]
    assert fx["night_seed"]["value"] == 70 and fx["night_mode"]["source"] == "night.json" and "night_round" not in fx
    assert fx["route_waypoints"]["value"] == 3 and fx["route_planned_m"]["source"] == "route.json"
    eng = ml.write(d).read_text().split("<summary>The night's figures</summary>")[1]
    assert "House seed" in eng and "Route planned (m)" in eng


def test_night_json_holds_no_truth(tmp_path, monkeypatch):
    """[9X9]: only NIGHT_KEYS are ever written (the sim's items and truth paths in the environment stay out), and a
    night.json with any other key is refused"""
    d = night(tmp_path, monkeypatch=monkeypatch)
    ml.write_night_json(d, {"HK_SIM_ITEMS": "[1]", "DIMOS_SIM_TRUTH_FILE": "x", "NIGHT_SEED": "70"})
    text = (d / "night.json").read_text()
    assert set(json.loads(text)) == set(ml.NIGHT_KEYS) and "truth" not in text and "items" not in text
    (d / "night.json").write_text(json.dumps({"seed": 70, "items": [[1, 2]]}))
    with pytest.raises(PermissionError): ml.household(ml.Night(d))


# ---- [Ndt] the phone's message ----

def runs(tmp_path, monkeypatch, kind="sim", screening="n/a"):
    """a runs folder with two nights, the later one's log written; an older night with no log at all"""
    root = tmp_path / "runs"; root.mkdir()
    early = night(tmp_path / "a", monkeypatch=monkeypatch, kind=kind, screening=screening)
    late = night(tmp_path / "b", monkeypatch=monkeypatch, kind=kind, screening=screening,
                 items=[{"flag": [1.0, 0.5], "label": "a sock", "standing": "standing look: a sock (0.4) 5 cm", "tries": 1, "outcome": "binned"}])
    shutil.move(early, root / "20261001_230000"); shutil.move(late, root / "20261002_230000")
    ml.write(root / "20261001_230000"); ml.write(root / "20261002_230000")
    (root / "20261003_230000").mkdir()                                         # tonight, not finished: no log
    return root


def test_the_phone_gets_the_latest_nights_log_with_jpeg_crops(tmp_path, monkeypatch):
    """[Ndt] [hKP]: the latest night with a log, its prose and figures as written, each find's crop a JPEG"""
    import base64
    root = runs(tmp_path, monkeypatch)
    m = ml.morning_message(root)
    doc = json.loads((root / "20261002_230000" / "morning_log.json").read_text())
    assert m["v"] == 1 and m["prose"] == doc["prose"] and [f["what"] for f in m["finds"]] == ["a sock"]
    assert base64.b64decode(m["finds"][0]["jpg"])[:3] == b"\xff\xd8\xff"           # JPEG
    assert {f["source"] for f in m["figures"]} <= {"report.json", "dimos.log", "traj.csv", "energy.json", "night.json", "route.json"}
    assert ml.morning_message(tmp_path / "nowhere") is None


def test_the_phones_message_keeps_within_its_bound(tmp_path, monkeypatch):
    """[Ndt]: never more than PHONE_MAX_BYTES of compact JSON: crops made smaller, then left off, to fit"""
    root = runs(tmp_path, monkeypatch)
    size = lambda m: len(json.dumps(m, separators=(",", ":")))
    full = ml.morning_message(root)
    assert ml.PHONE_MAX_BYTES <= 512 * 1024 and size(full) <= ml.PHONE_MAX_BYTES
    tight = size(full) - 50
    m = ml.morning_message(root, max_bytes=tight)
    assert m is not None and size(m) <= tight
    m = ml.morning_message(root, max_bytes=size({**full, "finds": [{**f, "jpg": None} for f in full["finds"]]}) + 40)
    assert m["finds"][0]["jpg"] is None and "left off" in m["pictures_note"]


def test_the_phone_gets_no_crops_from_an_unscreened_real_night(tmp_path, monkeypatch):
    """[Ndt] [R1X]: the crops are made again for the phone through recordings.require_screened: an unscreened real
    night's are refused, and its prose still comes"""
    root = runs(tmp_path, monkeypatch, kind="real", screening="unscreened")
    asked = []; real = ml.recordings.require_screened
    monkeypatch.setattr(ml.recordings, "require_screened", lambda rid: asked.append(rid) or real(rid))
    m = ml.morning_message(root, screen=lambda im: True)
    assert asked == [RID] and all(f["jpg"] is None for f in m["finds"]) and "withheld" in m["pictures_note"].lower() and m["prose"]


def test_the_pages_recorded_message_is_a_morning_message(tmp_path, monkeypatch):
    """[Ndt]: tests/fixtures/morning_message.json, which the Morning page's tests render from, has the shape
    morning_message makes (each key, and each find's)"""
    fix = json.loads((HK / "tests" / "fixtures" / "morning_message.json").read_text())
    m = ml.morning_message(runs(tmp_path, monkeypatch))
    assert set(fix) == set(m) and set(fix["finds"][0]) == set(m["finds"][0]) and fix["v"] == m["v"]


@pytest.mark.parametrize("W, H", [(640, 360), (1280, 720)])
def test_a_finds_crop_is_the_same_share_of_its_picture_at_any_frame_size(tmp_path, W, H):
    """[hKP] [7cU]: a find's crop covers CROP_FRAC of its look's width whatever the camera delivers (300 px of a 640 x 360
    look, 600 px of a 1280 x 720 one), from the close look and from the standing look alike, and the file shows it at
    FILE_CROP_PX"""
    d = tmp_path / "n"; (d / "looks").mkdir(parents=True)
    img = np.full((H, W, 3), 120, np.uint8)
    np.savez_compressed(d / "looks" / "item1_look1.npz", **{"0_img": img, "0_c": np.array([1.1, 0.5, 0.4]), "0_R": np.eye(3),
                        "0_fovy": np.array(50.0), "0_found_xy": np.array([1.0, 0.5])})
    from PIL import Image
    Image.fromarray(img).save(d / "looks" / "standing_1.png")
    class N:
        def path(self, p): return d / p
    side = round(300 * W / 640)
    c, box, wh, *_ = ml.look_crop(N(), 1, (1.0, 0.5))
    assert c.shape[:2] == (side, side) and box[2] - box[0] == side and wh == [W, H]
    c, box, wh, *_ = ml.standing_crop(N(), 1, {"flag_px": [W / 2, H / 2]})
    assert c.shape[:2] == (side, side) and box == [W // 2 - side // 2, H // 2 - side // 2, W // 2 - side // 2 + side, H // 2 - side // 2 + side]
    import base64, io
    small = Image.open(io.BytesIO(base64.b64decode(ml.jpeg_b64(c, ml.FILE_CROP_PX))))
    assert small.size == (ml.FILE_CROP_PX, ml.FILE_CROP_PX)
