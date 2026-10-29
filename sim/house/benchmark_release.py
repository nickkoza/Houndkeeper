#!/usr/bin/env python3
"""The benchmark, given back (benchmark-release [WFU]): an asset manifest with each asset's source, licence and
attribution, house specs that regenerate byte for byte, their scene hashes, and a dry-run export that builds the release
tree in a local folder. Nothing here uploads anything: a release leaves the machine only by hand.

Usage (the DimOS venv's python):
  benchmark_release.py manifest                    write benchmark/asset_manifest.json from what the code uses
  benchmark_release.py hashes SEED OUT [--px M]    generate one released house's nights into OUT; print its hashes
  benchmark_release.py export OUT [--copy] [--lock]   build the release tree in OUT (hard links unless --copy;
                                                   --lock generates every released house to record its hashes)
"""
import argparse, csv, hashlib, json, os, re, shutil, subprocess, sys, tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
BENCH = HERE / "benchmark"
MANIFEST = BENCH / "asset_manifest.json"
HOUSES = BENCH / "houses.json"
sys.path.insert(0, str(Path(__file__).resolve().parent)); sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "arm"))
sys.path.insert(0, str(REPO / "src")); from houndkeeper.workspace import dimos as _dimos   # noqa: E402  (decision [cy4])

# the files whose bytes decide a scene ([nIO]): the generator, its helpers, the lists it draws from and the scripts that
# pin the assets it reads (with RUNNER below, which points its output)
GENERATOR_SOURCES = ["sim/house/make_house.py", "sim/house/make_nights.py", "sim/house/cabinets.py", "sim/house/movables.py",
                     "sim/house/slate_tile.py", "sim/arm/floors.py", "sim/arm/floor_photos.csv", "sim/arm/items.py",
                     "sim/arm/jaws.py", "sim/dimos/house_items.py", "sim/dimos/house_toys.py", "tools/fetch_items.sh",
                     "tools/fetch_textures.sh", "tools/fetch_floors.sh", "tools/import_glb.py"]
# the scoring and its metric definitions ([tYQ])
SCORING_SOURCES = ["sim/house/metrics.py", "sim/house/criteria.json", "sim/house/difficulty.py", "sim/house/score_movables.py",
                   "sim/house/score_clutter.py", "sim/dimos/score_tidy.py", "sim/arm/bin_geom.py"]
TOOLS = ["sim/house/benchmark_release.py"]                     # regenerates and hashes a house from the release

# licence -> (may be passed on, needs a credit) ([Gez]); anything not here is unknown, and unknown is withheld
LICENCES = {"CC0-1.0": (True, False), "CC-BY-4.0": (True, True), "MIT": (True, True), "BSD-3-Clause": (True, True),
            "Apache-2.0": (True, True), "own": (True, False), "unknown": (False, False), "real-house": (False, False)}


class Refused(Exception):
    """a path or file the release may never carry"""


def redistributable(licence):
    """[kWd]: whether a licence lets a public release carry the asset: every part of a compound licence ("A AND B") must,
    and a non-commercial or unknown licence never does"""
    parts = [p.strip() for p in licence.split(" AND ")]
    return all("NC" not in p.split("-") and LICENCES.get(p, (False, False))[0] for p in parts)


# ---------------------------------------------------------------- what may never leave ([MXq], [zGo])
REFUSED_TOP = {"data", "real", "notes"}                        # recordings, photos, the house network, private notes
REFUSED_PREFIX = ("sim/house/runs/",)                         # run outputs (real nights among them)
NETWORK = re.compile(r"\b(?:10\.\d{1,3}\.\d{1,3}\.\d{1,3}|192\.168\.\d{1,3}\.\d{1,3}|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3}"
                     r"|(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2})\b")   # a private address or a MAC
TEXT = {".py", ".sh", ".json", ".csv", ".md", ".txt", ".xml", ".mtl", ".obj", ".urdf", ".yaml", ".yml"}


def check_path(rel):
    """[MXq]: refuse a repository path the release may never carry: anything under data/, real/ or notes/, a recording,
    a house-twin output or tool, a real-nights tool, the sim's run outputs, a database or an environment file"""
    rel = str(rel)
    p = Path(rel)
    if p.is_absolute() or ".." in p.parts: raise Refused(f"{rel}: outside the repository")
    low = [s.lower() for s in p.parts]
    why = ("under " + p.parts[0] + "/" if p.parts and p.parts[0] in REFUSED_TOP else
           "a run output or real-house folder" if rel.startswith(REFUSED_PREFIX) else
           "a recording" if any("recording" in s for s in low) else
           "a house-twin output or tool" if any("twin" in s for s in low) else
           "a real-house tool or folder" if any(s.startswith("real_") for s in low) else
           "a database or environment file" if p.suffix == ".db" or p.name.startswith(".env") else None)
    if why: raise Refused(f"{rel}: {why}")


def check_content(path):
    """[MXq]: refuse a text file holding a network address (a private IP or a MAC)"""
    path = Path(path)
    if path.suffix.lower() not in TEXT: return
    m = NETWORK.search(path.read_text(errors="replace"))
    if m: raise Refused(f"{path}: holds a network address ({m.group(0)})")


# ---------------------------------------------------------------- the manifest ([h1X], [1P3])
def _sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""): h.update(b)
    return h.hexdigest()


def _files_entry(files, hashes):
    """the files an entry covers (repository-relative, sorted), with a hash over them and their size when on disk"""
    files = sorted(str(Path(f).relative_to(REPO)) for f in files)
    if not files: return {"files": [], "present": False}
    out = {"files": files, "present": True, "bytes": sum((REPO / f).stat().st_size for f in files)}
    if hashes:
        h = hashlib.sha256()
        for f in files: h.update(f"{f}\0{_sha(REPO / f)}\n".encode())
        out["sha256"] = h.hexdigest()
    return out


def _entry(id_, kind, licence, source, attribution, evidence, used_by, files, hashes, **kw):
    e = {"id": id_, "kind": kind, "used_by": used_by, "source": source, "licence": licence, "attribution": attribution,
         "evidence": evidence, "redistributable": redistributable(licence), "credit_required": any(
             LICENCES.get(p, (False, False))[1] for p in licence.split(" AND "))}
    e.update(kw); e.update(_files_entry(files, hashes))
    return e


def _imports():
    import items, make_nights as mn
    return items, mn


def gso_entries(hashes):
    items, mn = _imports()
    gso = REPO / "models/items/gso"
    commit = re.search(r"^COMMIT=(\w+)", (REPO / "tools/fetch_items.sh").read_text(), re.M).group(1)
    used = {}
    for n in items.GSO_NAMES: used.setdefault(n, []).append("sim/arm/items.py")
    for k, s, *_ in items.FIXTURES.values(): used.setdefault(s["dir"], []).append("sim/arm/items.py (FIXTURES)")
    for n in re.findall(r'scanned\("(\w+)"', (HERE / "make_nights.py").read_text()): used.setdefault(n, []).append("sim/house/make_nights.py")
    held = set()
    for n in items.HELD_OUT:
        s = items.ITEMS[n][1]; held.add(s.get("dir") or s.get("gso_dir"))
    on_disk = {d.name for d in gso.iterdir() if d.is_dir()} if gso.exists() else set()
    fixtures = {s["dir"] for k, s, *_ in items.FIXTURES.values()}
    out = []
    for d in sorted(set(used) | on_disk):
        note = None
        if d not in used: note = "on disk, not used by the code (left out as a fused set of pieces): not released"
        elif d in fixtures: note = "tools/fetch_items.sh fetches items.GSO_NAMES only, which leaves this fixture out: fetch it by hand"
        out.append(_entry(
            f"gso/{d}", "scanned_object", "CC-BY-4.0 AND MIT",
            f"Google Scanned Objects (Google Research), as a MuJoCo model by kevinzakka/mujoco_scanned_objects at {commit}",
            f'"{d}", Google Scanned Objects by Google Research, CC BY 4.0 (https://creativecommons.org/licenses/by/4.0/); '
            "MJCF conversion from github.com/kevinzakka/mujoco_scanned_objects, MIT. Changed: texture_1k.png is the texture "
            "scaled to 1024 px",
            ["models/README.md", "tools/fetch_items.sh"], sorted(set(used.get(d, []))),
            sorted(p for p in (gso / d).rglob("*") if p.is_file()) if d in on_disk else [], hashes,
            split="heldout" if d in held else "train", used=d in used,
            source_url=f"https://github.com/kevinzakka/mujoco_scanned_objects/tree/{commit}/models/{d}",
            fetch="tools/fetch_items.sh", **({"note": note} if note else {})))
    return out


def texture_entries(hashes):
    _, mn = _imports()
    used = {t for ids, _ in mn.SURFACES.values() for t in ids}
    rows = re.findall(r"^(\w+) (\w+) ([0-9a-f]{64})$", (REPO / "tools/fetch_textures.sh").read_text(), re.M)
    out = []
    for tid, use, sha in rows:
        f = REPO / "models/textures" / f"{tid}_Color.jpg"
        out.append(_entry(
            f"ambientcg/{tid}", "surface_texture", "CC0-1.0", f"ambientCG material {tid}, 1K JPG colour map",
            f"{tid} from ambientCG (ambientcg.com), CC0", ["models/README.md", "tools/fetch_textures.sh"],
            ["sim/house/make_nights.py (SURFACES)", "sim/house/cabinets.py"] if tid in used else [],
            [f] if f.exists() else [], hashes, used=tid in used, use=use, pinned_sha256=sha,
            source_url=f"https://ambientcg.com/get?file={tid}_1K-JPG.zip", fetch="tools/fetch_textures.sh"))
    return out


def floor_entries(hashes):
    rows = list(csv.DictReader(l for l in open(REPO / "sim/arm/floor_photos.csv") if not l.startswith("#")))
    floors_dir = REPO / "models/textures/floors"
    on_disk = {p.stem for p in (floors_dir / "zips").glob("*.zip")} if floors_dir.exists() else set()
    out = []
    for r in rows:
        files = sorted(floors_dir.glob(f"zips/{r['id']}.zip")) + sorted(floors_dir.glob(f"{r['id']}_*.jpg"))
        user = ["sim/house/slate_tile.py"] if r["kind"] == "stone" else ["sim/house/make_nights.py (--photo-floors)"]
        out.append(_entry(
            f"ambientcg/{r['id']}", "floor_photo", "CC0-1.0", f"ambientCG material {r['id']}, 2K JPG set ({r['kind']})",
            f"{r['id']} from ambientCG (ambientcg.com), CC0", ["sim/arm/floor_photos.csv", "tools/fetch_floors.sh"],
            user + ["sim/arm/floors.py"], files, hashes, used=True, split=r["split"], pinned_sha256=r["zip_sha256"],
            source_url=f"https://ambientcg.com/get?file={r['id']}_2K-JPG.zip", fetch="tools/fetch_floors.sh"))
    left_out = re.search(r"Left out of the \d+ fetched: ([^(]+)\(", (REPO / "sim/arm/floor_photos.csv").read_text())
    named = set(re.findall(r"\b([A-Z][A-Za-z]+\d+[A-Z]?)\b", left_out.group(1))) if left_out else set()
    for fid in sorted(on_disk - {r["id"] for r in rows}):        # downloaded once, in no list: not used, not released
        files = sorted(floors_dir.glob(f"zips/{fid}.zip")) + sorted(floors_dir.glob(f"{fid}_*.jpg"))
        known = fid in named
        out.append(_entry(
            f"ambientcg/{fid}", "floor_photo", "CC0-1.0" if known else "unknown",
            "ambientCG (named in floor_photos.csv as left out)" if known else "unknown: a zip on disk that no list names",
            f"{fid} from ambientCG (ambientcg.com), CC0" if known else "unknown",
            ["sim/arm/floor_photos.csv"] if known else [], [], files, hashes, used=False,
            note="on disk, not used by the code: not released"))
    return out


def furniture_entries(hashes):
    src = (HERE / "make_nights.py").read_text()
    names = sorted(set(re.findall(r'model\("(\w+)"', src)))
    fdir = REPO / "models/furniture"
    out = []
    for n in names:
        meta_f = fdir / n / "meta.json"
        if not meta_f.exists():
            out.append(_entry(f"sketchfab/{n}", "imported_model", "unknown", "unknown: not imported on this machine",
                              "unknown", [], ["sim/house/make_nights.py"], [], hashes, used=True,
                              fetch="download by hand into models/furniture/_src (Sketchfab needs a login), then tools/import_glb.py"))
            continue
        m = json.loads(meta_f.read_text())
        files = [meta_f] + [fdir / n / p["obj"] for p in m["parts"]] + [fdir / n / t for p in m["parts"]
                                                                       for t in p["textures"].values() if t]
        extra = sorted(set(p for p in (fdir / n).rglob("*") if p.is_file()) - set(files))   # full-size copies: not used
        glb = fdir / "_src" / m.get("glb", "")
        out.append(_entry(
            f"sketchfab/{n}", "imported_model", m.get("licence", "unknown"), f"Sketchfab, by {m.get('author', 'unknown')}",
            f"\"{n}\" by {m.get('author', 'unknown')} ({m.get('source', 'unknown')}), {m.get('licence', 'unknown')}. Changed: "
            f"converted from glTF to OBJ and PNG, scaled to {m.get('height_m')} m tall, re-oriented (tools/import_glb.py)",
            [str(meta_f.relative_to(REPO)), "models/README.md"],
            ["sim/house/make_nights.py (demo room)"], files, hashes, used=True, source_url=m.get("source"),
            unused_files=[str(p.relative_to(REPO)) for p in extra + ([glb] if glb.is_file() else [])],
            fetch=f"download {m.get('source')} by hand into models/furniture/_src, then tools/import_glb.py"))
    return out


def other_entries(hashes):
    out = [
        _entry("menagerie/unitree_go2", "robot_model", "BSD-3-Clause",
               "DeepMind MuJoCo Menagerie unitree_go2 (via DimOS's dependency mujoco_playground)",
               "Unitree Go2 model, MuJoCo Menagerie; copyright Unitree Robotics, BSD-3-Clause (models/go2/LICENSE)",
               ["models/README.md", "models/go2/LICENSE"], ["the robot (not the house)"],
               sorted(p for p in (REPO / "models/go2").rglob("*") if p.is_file()                 # not _src/: Unitree's own CAD,
                      and "_src" not in p.relative_to(REPO / "models/go2").parts), hashes, used=True),   # licence unstated, never shipped
        _entry("makermods/maker_arm", "robot_model", "Apache-2.0", "MakerMods maker-arm-sdk @ b30d05a",
               "Maker Arm model, MakerMods maker-arm-sdk, Apache-2.0 (models/maker_arm/LICENSE)",
               ["models/README.md", "models/maker_arm/LICENSE"], ["the robot's arm (not the house)"],
               sorted(p for p in (REPO / "models/maker_arm").rglob("*") if p.is_file()), hashes, used=True),
        _entry("own/procedural-house", "procedural", "own",
               "this project: the generated house (walls, windows, cabinets, furniture shapes, fireplace, generated floors, "
               "stains, carpet wear, the slate-look tile laid from the CC0 stone photos, litter, lighting)",
               "Houndkeeper", ["sim/house/make_house.py", "sim/house/make_nights.py", "sim/house/cabinets.py",
                               "sim/house/slate_tile.py"], ["sim/house/make_house.py", "sim/house/make_nights.py"], [], hashes,
               used=True, note="generated, not stored; the project's own work is MIT (LICENSE at the repository's root)"),
    ]
    chosen = [("house70-carpet-colour", "make_nights.HOUSE70_CARPET_SRGB: house 70's carpet hue, a value chosen for the sim"),
              ("slate-tile-design", "slate_tile.py: the slate-look tile's size, palette and inset (houses 70, 95)"),
              ("fireside", "make_nights.fireplace, wing chair placement and shutters (house 95, the demo room)")]
    for k, what in chosen:                  # [HQs]: the project's own values, carried in the generator's code
        out.append(_entry(f"own/{k}", "procedural", "own", what, "Houndkeeper", ["docs/specs/benchmark-release.md"],
                          ["sim/house/make_nights.py"], [], hashes, used=True,
                          note="a value in the generator's code: carried with it"))
    return out


def build_manifest(hashes=True):
    """[h1X]: every asset the generator, its nights and the item library use, from the code's own lists"""
    entries = gso_entries(hashes) + texture_entries(hashes) + floor_entries(hashes) + furniture_entries(hashes) + other_entries(hashes)
    return {"about": "Every asset the benchmark's house generator and item library use: source, licence, attribution, whether "
                     "a public release may carry it. Written by sim/house/benchmark_release.py manifest; unknown means "
                     "nobody recorded it, not that it is free.",
            "searched_not_found": ["HDRI or environment maps: none (the sky is MuJoCo's builtin gradient; windows are "
                                   "emissive planes)"],
            "dependencies": [{"name": "DimOS", "licence": _licence_line(_dimos() / "LICENSE"),
                              "note": "the sim and the robot run on DimOS with sim/dimos/dimos_fork_changes.patch; fetched "
                                      "separately, not carried"}],
            "assets": entries}


def _licence_line(f):
    try: return "Apache-2.0" if "Apache License" in f.read_text()[:400] else "unknown"
    except OSError: return "unknown"


def on_disk_assets():
    """every asset file in the folders assets live in, repository-relative: each must belong to a manifest entry"""
    roots = [REPO / "models/items/gso", REPO / "models/textures", REPO / "models/furniture"]
    return sorted(str(p.relative_to(REPO)) for r in roots if r.exists() for p in r.rglob("*")
                  if p.is_file() and p.name != "SOURCE_COMMIT")


# ---------------------------------------------------------------- house specs, generation and hashes ([e5I], [PiV])
def generator_version():
    """[nIO]: g- and 12 hex digits of a hash over the generator's sources (path and bytes) and the runner"""
    h = hashlib.sha256(RUNNER.encode())
    for f in sorted(GENERATOR_SOURCES): h.update(f.encode() + b"\0" + (REPO / f).read_bytes() + b"\0")
    return "g-" + h.hexdigest()[:12]


def library_versions():
    import zlib, numpy, PIL, cv2, scipy, mujoco
    return {"python": sys.version.split()[0], "numpy": numpy.__version__, "Pillow": PIL.__version__, "opencv": cv2.__version__,
            "scipy": scipy.__version__, "mujoco": mujoco.__version__, "zlib": zlib.ZLIB_VERSION}


def houses():
    return json.loads(HOUSES.read_text())


def held_out_seeds():
    return {h["seed"] for h in houses()["houses"] if h["split"] == "heldout"}


def released(spec=None):
    """[w7W]: the houses a release carries: never a held-out one, never one withheld"""
    spec = spec or houses()
    return [h for h in spec["houses"] if h["split"] != "heldout" and h.get("released", True)]


RUNNER = r"""
import json, sys
from pathlib import Path
cfg = json.loads(sys.argv[1]); sys.path[:0] = cfg["paths"]
import make_house as mh
out = Path(cfg["out"]); mh.DIMOS_DATA = out / "scenes"; mh.OUT_DIR = out / "truth"
for d in ("scenes", "truth", "textures"): (out / d).mkdir(parents=True, exist_ok=True)
if cfg["generator"] == "make_house":
    sys.argv = ["make_house.py"] + cfg["args"]; mh.main()
else:
    import make_nights as mn
    mn.TEX_DIR = out / "textures"
    if cfg.get("px"): mn.PHOTO_PX = mn.PX = cfg["px"]
    sys.argv = ["make_nights.py"] + cfg["args"]; mn.main()
"""


def generate(seed, out, args, generator="make_nights", px=None, floors="train", items="train", hash_seed="0", python=None):
    """write one house's scenes into out (scenes/, textures/, truth/) with nothing written anywhere else; px coarsens
    the floor (a check of the code path, not the released resolution). A held-out house is refused ([w7W])"""
    if seed in held_out_seeds(): raise Refused(f"house {seed} is held out: the release tooling never generates it")
    out = Path(out).resolve(); out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, HK_HOUSE_SEED=str(seed), HK_FLOORS=floors, HK_ITEMS=items, PYTHONHASHSEED=str(hash_seed))
    cfg = {"paths": [str(HERE), str(REPO / "sim" / "arm")], "out": str(out), "generator": generator,
           "args": ["--seed", str(seed)] + list(args), "px": px}
    r = subprocess.run([python or sys.executable, "-c", RUNNER, json.dumps(cfg)], cwd=HERE, env=env, capture_output=True,
                       text=True)
    if r.returncode: raise RuntimeError(f"house {seed}: generation failed\n{r.stderr[-2000:]}")
    return out


def scene_hashes(out):
    """[PiV], [OtY]: SHA-256 of every file a generation wrote, by its path inside out; text files with the output folder
    and the repository written as ${OUT} and ${REPO}, so the same scene in two folders hashes the same"""
    out = Path(out).resolve(); res = {}
    for p in sorted(q for q in out.rglob("*") if q.is_file()):
        b = p.read_bytes()
        if p.suffix in (".xml", ".obj", ".json"):
            b = b.replace(str(out).encode(), b"${OUT}").replace(str(REPO).encode(), b"${REPO}")
        res[str(p.relative_to(out))] = hashlib.sha256(b).hexdigest()
    return res


def track_args(house, spec=None):
    t = (spec or houses())["tracks"][house["track"]]
    return t["args"], t["generator"], t["floors"], t["items"]


# ---------------------------------------------------------------- the export ([pvP], [tYQ], [9SZ], [ryb], [5JR])
REVIEW_WORDS = re.compile(r"\bowner\b|footage|real house|/home/\w+", re.I)   # what may describe the real house, and machine paths


def code_files():
    return GENERATOR_SOURCES + SCORING_SOURCES + TOOLS


def plan(manifest):
    """[ryb]: every repository path the export copies (to the same path in the tree, so the tree is a checkout), each
    checked first; nothing is copied until the whole plan passes"""
    paths = list(code_files())
    for e in manifest["assets"]:
        if redistributable(e["licence"]) and e.get("used") and e["present"]:   # the licence decides, not a stored flag
            paths += [f for f in e["files"] if f not in paths]
    for src in paths:
        check_path(src); check_content(REPO / src)
    return paths


def review_lines():
    """lines in the exported code that may describe the real house or name a machine's paths: to keep or
    strip by hand ([HQs])"""
    out = []
    for src in code_files():
        for i, l in enumerate((REPO / src).read_text(errors="replace").splitlines(), 1):
            if REVIEW_WORDS.search(l): out.append(f"{src}:{i}: {l.strip()[:160]}")
    return out


def export(out, manifest=None, spec=None, copy=False, lock=False):
    """[pvP]: build the release tree in out, a folder outside the repository; nothing leaves the machine ([5JR])"""
    out = Path(out).resolve()
    if out == REPO or REPO in out.parents: raise Refused(f"{out}: an export is built outside the repository")
    manifest = manifest or json.loads(MANIFEST.read_text()); spec = spec or houses()
    paths = plan(manifest)
    if out.exists():
        if any(out.iterdir()) and not (out / ".benchmark-export").exists(): raise Refused(f"{out}: not empty and not an export")
        shutil.rmtree(out)
    out.mkdir(parents=True); (out / ".benchmark-export").write_text("a dry-run release tree; nothing here was published\n")
    for src in paths:
        d = out / src; d.parent.mkdir(parents=True, exist_ok=True)
        if copy: shutil.copy2(REPO / src, d)
        else:
            try: os.link(REPO / src, d)
            except OSError: shutil.copy2(REPO / src, d)
    version = generator_version(); rel = released(spec); bench = out / "sim/house/benchmark"; bench.mkdir(parents=True)
    stubs = [{"seed": h["seed"], "split": h["split"], "released": False} for h in spec["houses"] if h not in rel]
    houses_out = {"spec_version": spec["spec_version"], "generator_version": version, "libraries": library_versions(),
                  "tracks": spec["tracks"], "houses": rel + stubs,     # the withheld by number only: the tooling refuses them
                  "withheld": sorted(h["seed"] for h in stubs),
                  "held_out": sorted(h["seed"] for h in stubs if h["split"] == "heldout")}
    (bench / "houses.json").write_text(json.dumps(houses_out, indent=1) + "\n")
    if lock:
        hashes = {}
        for h in rel:
            args, gen, fl, it = track_args(h, spec)
            with tempfile.TemporaryDirectory() as t:
                hashes[str(h["seed"])] = scene_hashes(generate(h["seed"], t, args, gen, floors=fl, items=it))
        (bench / "hashes.json").write_text(json.dumps({"generator_version": version, "libraries": library_versions(),
                                                           "houses": hashes}, indent=1) + "\n")
    shipped = set(paths)
    m = dict(manifest, assets=[dict(e, in_export=bool(e["files"]) and all(f in shipped for f in e["files"]))
                               for e in manifest["assets"]])
    (bench / "asset_manifest.json").write_text(json.dumps(m, indent=1) + "\n")
    review = review_lines()
    (out / "ATTRIBUTION.md").write_text(attribution_md(m))
    (out / "FETCH.md").write_text(fetch_md(m))
    (out / "README.md").write_text(readme_md(m, houses_out, spec, lock))
    (out / "REVIEW.md").write_text("# Lines to review before any release\n\nThe exported code's mentions of "
                                   "what may describe the real house, and the machine paths it names. Keep, reword or strip "
                                   "each; nothing is published until then.\n\n"
                                   + "".join(f"- {l}\n" for l in review))
    return {"out": str(out), "files": len(paths), "assets_in": sum(e["in_export"] for e in m["assets"]),
            "assets_out": sum(not e["in_export"] and e["kind"] != "procedural" for e in m["assets"]), "houses": len(rel),
            "withheld": houses_out["withheld"], "review_lines": len(review), "generator_version": version}


def attribution_md(m):
    lines = ["# Attribution\n", "Every third-party asset this release carries, with the credit its licence asks for.\n"]
    for e in m["assets"]:
        if e["in_export"] and e["licence"] != "own":
            lines.append(f"- **{e['id']}** ({e['licence']}): {e['attribution']}")
    lines.append("\nCC BY 4.0: https://creativecommons.org/licenses/by/4.0/ . The robot models keep their licence files "
                 "beside them. Missing from this tree: the MIT licence text of kevinzakka/mujoco_scanned_objects, which "
                 "tools/fetch_items.sh does not fetch; a release adds it.\n")
    return "\n".join(lines) + "\n"


def fetch_md(m):
    lines = ["# Assets not carried, and how to get them\n",
             "Each is fetched from its source by the script named (under tools/), or by hand where none exists.\n",
             "| Asset | Licence | Why not carried | How to get it |", "|---|---|---|---|"]
    for e in m["assets"]:
        if e["in_export"] or e["kind"] in ("procedural",): continue
        why = ("drawn from the real house: never released" if e["licence"] == "real-house" else
               "not used by the benchmark" if not e.get("used") else
               "licence unknown" if e["licence"] == "unknown" else
               "not redistributable" if not e["redistributable"] else "not on the exporting machine")
        lines.append(f"| {e['id']} | {e['licence']} | {why} | {e.get('fetch') or e.get('note') or '-'} |")
    return "\n".join(lines) + "\n"


def readme_md(m, h, spec, locked):
    a = m["assets"]; inn = [e for e in a if e["in_export"]]
    out_n = sum(1 for e in a if not e["in_export"] and e["kind"] != "procedural")
    other = [s for s in h["withheld"] if s not in h["held_out"]]
    by_lic = {}
    for e in inn: by_lic[e["licence"]] = by_lic.get(e["licence"], 0) + 1
    t = spec["tracks"]["tidy"]
    return f"""# Houndkeeper tidy-house benchmark (dry run)

A procedural benchmark for a legged robot that tidies a house at night: houses generated from a seed, each with its
nights (furniture that moves, a pet's bowls, a photographed floor with carpet relief and wear, toys and household
things on the floor), and the code that scores a run. Built for Houndkeeper, a personal learning project on DimOS.
This tree is a dry run: nothing in it has been published.

## What the sim is held to

The sim looks like a house wherever a realistic asset exists (scanned toys and household objects, photographed floors
and surfaces, real proportions), and it is never kinder than the real world: where a real robot is imperfect, the sim
models the imperfection rather than leaving it ideal. A method that only works on a perfect sim is not counted.

## Contents

The tree is laid out as a checkout: the paths are the project's own.

- `sim/house/benchmark/houses.json`: {len(released(h))} houses as specs (seed, track, split), generator version
  `{h['generator_version']}`. Track `tidy`: `make_nights.py --seed SEED {' '.join(t['args'])}`; toy sets
  {t['toy_sets']} placed by `{t['toys']}`.
- `sim/`, `tools/`: the generator and its helpers, the fetch scripts that pin every asset, and the scoring.
- `models/`: {len(inn)} assets the licences let us pass on ({', '.join(f'{v} {k}' for k, v in sorted(by_lic.items()))}).
  `sim/house/benchmark/asset_manifest.json`: every asset's source, licence, attribution and whether it is carried.
- `FETCH.md`: the {out_n} assets not carried, why, and how to get them. `ATTRIBUTION.md`: the credits.

## Left out, and why

- Held-out houses ({', '.join(str(s) for s in h['held_out'])}): kept for judging, never published. Floors and items held
  out for judging are labelled `split: heldout` in the manifest: keep them out of any tuning.
- Nothing of the real house: no photographs, recordings, floor textures made from it, network details or anything of
  the household. Houses withheld for any other reason: {', '.join(str(s) for s in other) or 'none'}.
- Assets whose licence is unknown or does not allow passing on (see FETCH.md).

## Regenerating and verifying

With the libraries `houses.json` lists (`libraries`) and the assets in FETCH.md fetched: `python
sim/house/benchmark_release.py hashes SEED OUT` writes a house's scenes into OUT and prints the SHA-256 of every file, with
absolute paths written as `${{OUT}}` and `${{REPO}}`. The same seed and generator version give the same bytes.
{'`sim/house/benchmark/hashes.json` holds the hashes to check against.' if locked else 'A release records them in `sim/house/benchmark/hashes.json` (export --lock); this dry run did not generate them.'}

## Scoring

`sim/house/criteria.json` fixes the detection criteria (written before any result); `metrics.py` gives per-night
find rates by kind; `difficulty.py` the tier of each item from its own size and contrast; `score_movables.py` false
alarms on moved furniture; `sim/dimos/score_tidy.py` a tidy run's numbers: toys binned, found, stops for nothing,
too-big items reported. A run is scored only against the sim's truth files; held-out houses are scored once, after the
method is frozen.

## Licence

Third-party assets keep their own licences (manifest, ATTRIBUTION.md). The project's own code and generated content
are MIT (LICENSE at the repository's root).
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("manifest")
    p = sub.add_parser("hashes"); p.add_argument("seed", type=int); p.add_argument("out"); p.add_argument("--px", type=float)
    p = sub.add_parser("export"); p.add_argument("out"); p.add_argument("--copy", action="store_true")
    p.add_argument("--lock", action="store_true")
    a = ap.parse_args()
    if a.cmd == "manifest":
        m = build_manifest(); MANIFEST.write_text(json.dumps(m, indent=1) + "\n")
        n = {k: sum(1 for e in m["assets"] if f(e)) for k, f in (("redistributable", lambda e: e["redistributable"]),
             ("not", lambda e: not e["redistributable"] and e["licence"] != "unknown"), ("unknown", lambda e: e["licence"] == "unknown"))}
        print(f"{len(m['assets'])} assets: {n}; written {MANIFEST.relative_to(REPO)}")
    elif a.cmd == "hashes":
        h = next((x for x in houses()["houses"] if x["seed"] == a.seed), None)
        if a.seed in held_out_seeds(): raise SystemExit(f"house {a.seed} is held out: never generated here")
        if not h or "track" not in h: raise SystemExit(f"house {a.seed} is not in {HOUSES.relative_to(REPO)}")
        args, gen, fl, it = track_args(h)
        print(json.dumps({"generator_version": generator_version(), "hashes": scene_hashes(
            generate(a.seed, a.out, args, gen, px=a.px, floors=fl, items=it))}, indent=1))
    else:
        print(json.dumps(export(a.out, copy=a.copy, lock=a.lock), indent=1))


if __name__ == "__main__":
    main()
