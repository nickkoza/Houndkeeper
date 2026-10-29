"""benchmark-release [WFU]: the asset manifest, versioned house specs, hashed regeneration and the dry-run export.
Static: scene files are generated (at a coarse floor resolution), nothing is stepped or rendered."""
import json
import re
import shutil
import sys
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "sim" / "arm"))
import benchmark_release as br

needs_assets = pytest.mark.skipif(not (HK / "models/items/gso").exists() or not (HK / "models/textures/floors/zips").exists(),
                                  reason="tools/fetch_items.sh and tools/fetch_floors.sh not run")
SMALL = {"gso/Dino_4", "gso/Top_Paw_Dog_Bow_Bone_Ceramic_13_fl_oz_total", "ambientcg/Wood092", "ambientcg/Carpet016",
         "ambientcg/Tiles141", "ambientcg/Tiles076", "sketchfab/bananas", "own/procedural-house",
         "own/house70-carpet-colour", "gso/PETS_ACCESSORIES"}


def _strip(m):
    return dict(m, assets=[{k: v for k, v in e.items() if k != "sha256"} for e in m["assets"]])


@pytest.fixture(scope="module")
def fresh():
    return br.build_manifest(hashes=False)


@pytest.fixture
def small():
    m = json.loads(br.MANIFEST.read_text())
    return dict(m, assets=[e for e in m["assets"] if e["id"] in SMALL])


@needs_assets
def test_every_asset_the_code_uses_has_an_entry(fresh):
    """[h1X]: every scan, surface texture, floor photo, imported model the code names has an entry with a source,
    a licence and an attribution, and no asset file on disk is left uncovered"""
    ids = {e["id"] for e in fresh["assets"]}
    items, mn = br._imports()
    import floors
    assert {f"gso/{d}" for d in items.GSO_NAMES} <= ids
    assert {f"gso/{s['dir']}" for _, s, *_ in items.FIXTURES.values()} <= ids
    assert {f"ambientcg/{t}" for ts, _ in mn.SURFACES.values() for t in ts} <= ids
    assert {f"ambientcg/{r['id']}" for r in floors.photos("train") + floors.photos("heldout")} <= ids
    assert {f"sketchfab/{n}" for n in ("wing_chair", "bananas", "snake_plant", "table_plant", "salt_pepper")} <= ids
    for e in fresh["assets"]:
        assert e["source"] and e["licence"] and e["attribution"] and e["kind"], e["id"]
    covered = {f for e in fresh["assets"] for f in e["files"] + e.get("unused_files", [])}
    assert set(br.on_disk_assets()) - covered == set()
    assert any("HDRI" in s for s in fresh["searched_not_found"])          # looked for, and said so


@needs_assets
def test_the_committed_manifest_matches_a_fresh_walk(fresh):
    """[h1X]: the manifest in the repository is the one the walk writes (re-run `benchmark_release.py manifest`)"""
    assert _strip(json.loads(br.MANIFEST.read_text())) == _strip(fresh)


def test_redistributable_follows_the_licence_alone(fresh):
    """[kWd]: one rule from the licence; unknown, non-commercial and real-house never; a compound licence only if every
    part allows it; a licence nobody recorded is written unknown, not guessed"""
    for e in fresh["assets"]:
        assert e["redistributable"] == br.redistributable(e["licence"]), e["id"]
    assert br.redistributable("CC0-1.0") and br.redistributable("CC-BY-4.0 AND MIT")
    for lic in ("unknown", "CC-BY-NC-4.0", "CC-BY-NC-SA-4.0", "real-house", "CC-BY-4.0 AND unknown", "Some-New-Licence"):
        assert not br.redistributable(lic), lic
    by = {e["id"]: e for e in fresh["assets"]}
    assert by["ambientcg/Tiles076"]["licence"] == "unknown"
    assert all(not e["redistributable"] for k, e in by.items() if k.startswith("real-house/"))
    assert all(by[k]["licence"] == "own" and by[k]["redistributable"]                  # [HQs]: the project's own values
               for k in ("own/house70-carpet-colour", "own/slate-tile-design", "own/fireside"))


def test_released_specs_are_versioned(tmp_path, monkeypatch):
    """[e5I]: each released house is a seed, a track and a split; the version names the generator's sources, and any
    change to one of them is a new version"""
    spec = br.houses()
    for h in br.released(spec):
        assert isinstance(h["seed"], int) and h["track"] in spec["tracks"] and h["split"] in ("tuning", "validation")
    v = br.generator_version()
    assert re.fullmatch(r"g-[0-9a-f]{12}", v)
    for f in br.GENERATOR_SOURCES:
        (tmp_path / f).parent.mkdir(parents=True, exist_ok=True); shutil.copy(HK / f, tmp_path / f)
    monkeypatch.setattr(br, "REPO", tmp_path)
    assert br.generator_version() == v
    with open(tmp_path / "sim/house/make_nights.py", "a") as f: f.write("\n")
    assert br.generator_version() != v


def test_hashes_ignore_the_folder_but_not_the_bytes(tmp_path):
    """[PiV], [OtY]: the same scene written to two folders hashes the same; one byte changed does not"""
    for d in ("a", "b"):
        (tmp_path / d).mkdir()
        (tmp_path / d / "scene.xml").write_text(f'<mesh file="{tmp_path / d}/m.obj"/><mesh file="{br.REPO}/models/x.obj"/>')
    assert br.scene_hashes(tmp_path / "a") == br.scene_hashes(tmp_path / "b")
    (tmp_path / "b" / "scene.xml").write_text((tmp_path / "b" / "scene.xml").read_text() + " ")
    assert br.scene_hashes(tmp_path / "a") != br.scene_hashes(tmp_path / "b")


@needs_assets
def test_a_house_and_its_nights_generated_twice_hash_the_same(tmp_path):
    """[PiV], [e5I]: a released house's tidy track (two of its nights, the floor at 4 cm a pixel: the code path, not the
    released resolution), generated twice into different folders under different hash seeds: every scene file (MJCF,
    floor, normal and relief textures, meshes, truth) is byte-identical; nothing is written outside the folder"""
    spec = br.houses(); h = next(x for x in br.released(spec) if x["seed"] == 72)
    args, gen, fl, it = br.track_args(h, spec)
    args = [("2" if a == "3" and args[i - 1] == "--nights" else a) for i, a in enumerate(args)]
    a = br.scene_hashes(br.generate(72, tmp_path / "a", args, gen, px=0.04, floors=fl, items=it, hash_seed="1"))
    b = br.scene_hashes(br.generate(72, tmp_path / "b", args, gen, px=0.04, floors=fl, items=it, hash_seed="2"))
    assert a == b
    kinds = {Path(k).suffix for k in a}
    assert {".xml", ".png", ".json", ".obj"} <= kinds
    assert sum(k.endswith(".truth.json") for k in a) == 2
    assert {p.name for p in (tmp_path / "a").iterdir()} == {"scenes", "textures", "truth"}


def test_the_procedural_house_generated_twice_hashes_the_same(tmp_path):
    """[PiV]: the procedural PoC house (make_house.py) too"""
    a = br.scene_hashes(br.generate(1, tmp_path / "a", ["--count", "30"], "make_house", hash_seed="3"))
    b = br.scene_hashes(br.generate(1, tmp_path / "b", ["--count", "30"], "make_house", hash_seed="4"))
    assert a == b and len(a) == 2


def test_held_out_houses_are_never_released_or_generated(tmp_path, small):
    """[w7W]: the hold-out houses are in no released spec, the tooling refuses to generate one, and an export names them
    only as withheld; held-out floors and items keep their label"""
    held = br.held_out_seeds()
    assert {80, 81} <= held
    assert not held & {h["seed"] for h in br.released()}
    with pytest.raises(br.Refused):
        br.generate(81, tmp_path / "g", [])
    assert not (tmp_path / "g").exists()
    br.export(tmp_path / "rel", manifest=small)
    out = json.loads((tmp_path / "rel/sim/house/benchmark/houses.json").read_text())
    assert not held & {h["seed"] for h in br.released(out)} and held <= set(out["withheld"]) == set(out["held_out"])
    assert all(set(h) == {"seed", "split", "released"} for h in out["houses"] if h["seed"] in held)   # by number only
    m = json.loads((tmp_path / "rel/sim/house/benchmark/asset_manifest.json").read_text())
    assert next(e for e in m["assets"] if e["id"] == "ambientcg/Tiles141")["split"] == "heldout"


@needs_assets
def test_export_carries_only_what_may_be_passed_on(tmp_path, small):
    """[pvP]: only redistributable, used assets are copied; every other one is listed with how to get it"""
    br.export(tmp_path / "rel", manifest=small)
    tree = tmp_path / "rel"
    files = {str(p.relative_to(tree)) for p in tree.rglob("*") if p.is_file()}
    listed = {f for e in small["assets"] for f in e["files"] + e.get("unused_files", [])}
    shipped = files & listed
    allowed = {f for e in small["assets"] if br.redistributable(e["licence"]) and e.get("used") for f in e["files"]}
    assert shipped and shipped == allowed
    assert files - listed <= set(br.code_files()) | {"README.md", "ATTRIBUTION.md", "FETCH.md", "REVIEW.md", ".benchmark-export",
                                                     "sim/house/benchmark/houses.json", "sim/house/benchmark/asset_manifest.json"}
    assert not any("PETS_ACCESSORIES" in f or "Tiles076" in f for f in shipped)                    # not used
    fetch = (tmp_path / "rel/FETCH.md").read_text()
    for k in ("ambientcg/Tiles076", "gso/PETS_ACCESSORIES"):
        assert k in fetch, k
    assert "tools/fetch_items.sh" in fetch or "fetch it by hand" in fetch


def test_export_carries_the_scoring(tmp_path, small):
    """[tYQ]: the scoring code and the metric definitions travel with the release"""
    br.export(tmp_path / "rel", manifest=small)
    for f in br.SCORING_SOURCES:
        assert (tmp_path / "rel" / f).is_file(), f
    assert json.loads((tmp_path / "rel/sim/house/criteria.json").read_text())["criteria"]


PRIVATE_NOTE = str(Path("notes") / "story.md")    # a file in the private notes folder (gitignored)


def test_nothing_of_the_real_house_is_ever_exported(tmp_path, small, monkeypatch):
    """[MXq]: data/, real/, notes/, recordings, house-twin outputs and tools, real-nights tools, run outputs, databases
    and environment files refused by path; a network address refused by content; the whole export refused before
    anything is copied; a real-house entry never copied whatever flag it carries"""
    for rel in ("data/recordings/catalog.sqlite", "data/twin/house/floor.png", "real/.env", "real/calibration/measure.py",
                PRIVATE_NOTE, "sim/house/runs/nights_s70/house_floor_s70_n1.png", "sim/house/house_twin.py",
                "sim/house/twin_score.py", "sim/house/real_nights.py", "tools/real_scans/notes.txt",
                "sim/dimos/recordings/run1/memory.db", "x/memory.db", "../elsewhere.py", "/etc/hosts"):
        with pytest.raises(br.Refused):
            br.check_path(rel)
    br.check_path("sim/house/make_nights.py"); br.check_path("models/items/gso/Dino_4/model.obj")
    for text in ('{"robot": "192.168.12.1"}', "router 10.0.0.1", "aa:bb:cc:dd:ee:ff"):
        f = tmp_path / "net.json"; f.write_text(text)
        with pytest.raises(br.Refused):
            br.check_content(f)
    monkeypatch.setattr(br, "SCORING_SOURCES", br.SCORING_SOURCES + [PRIVATE_NOTE])
    with pytest.raises(br.Refused):
        br.export(tmp_path / "rel", manifest=small)
    assert not (tmp_path / "rel").exists()
    monkeypatch.undo()
    tampered = dict(small, assets=small["assets"] + [dict(next(e for e in small["assets"] if e["id"] == "ambientcg/Wood092"),
                                                          id="real-house/x", licence="real-house", redistributable=True)])
    wood = next(e for e in small["assets"] if e["id"] == "ambientcg/Wood092")["files"]
    assert br.plan(tampered) == br.plan(small) and set(wood) <= set(br.plan(small))   # only through its own entry
    with pytest.raises(br.Refused):
        br.export(HK / "sim/house/runs/_export_test", manifest=small)           # never inside the repository


def test_export_writes_a_datasheet(tmp_path, small):
    """[9SZ]: a README saying what is in, what is left out and why, how to regenerate and verify, how it is scored;
    the credits for every CC-BY asset it carries"""
    br.export(tmp_path / "rel", manifest=small)
    readme = (tmp_path / "rel/README.md").read_text()
    for h in ("## Contents", "## Left out, and why", "## Regenerating and verifying", "## Scoring", "## Licence"):
        assert h in readme, h
    assert br.generator_version() in readme
    attr = (tmp_path / "rel/ATTRIBUTION.md").read_text()
    m = json.loads((tmp_path / "rel/sim/house/benchmark/asset_manifest.json").read_text())
    for e in m["assets"]:
        if e["in_export"] and e["credit_required"]:
            assert e["id"] in attr, e["id"]


def test_the_tooling_publishes_nothing(tmp_path, small):
    """[5JR]: no network client in the release tooling; the export writes only inside the folder it is given"""
    src = (HK / "sim/house/benchmark_release.py").read_text()
    for net in ("urllib", "requests", "http.client", "socket", "ftplib", "paramiko", "boto", "curl", "scp ", "rsync", "gh "):
        assert not re.search(rf"\b{re.escape(net.strip())}\b", src), net
    before = {p for p in tmp_path.rglob("*")}
    br.export(tmp_path / "rel", manifest=small)
    assert all(tmp_path / "rel" in p.parents or p == tmp_path / "rel" for p in set(tmp_path.rglob("*")) - before)


def test_the_release_holds_out_every_house_the_generator_does():
    """[w7W]: the release's held-out houses are make_house's registry's (sim-house-nights [ejh], [A7G]), no more, no
    fewer: a new hold-out added there and not here would be generated and released"""
    import make_house as mh
    assert br.held_out_seeds() == set(mh.HOLDOUT)
