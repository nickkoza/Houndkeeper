"""[1B2]: nothing on the robot's side reads the generator's truth. From the robot's own code (the night's modules, the
sweep route, the floor pipeline, the confirmer, the houndkeeper modules the robot runs) every import inside the repository
is followed, and each file reached is read for an import of the house generator or a scorer, and for a name of a truth
file (a night's truth.json, a score, the sim's true path, the generator's floor texture, its seed). Static: what a file
could open, not what one run did. Only the generator, the scorers and the analysis tools may; each is named below, and
every module in sim/house, sim/dimos and src/houndkeeper is one side or the other, so a new one has to be placed.
Where the truth is needed beside robot code (a replay scored, a pose ablation, a morning log with the sim's score), the
scorer or analysis tool imports the robot's code and hands it the truth through an injected option (pose_hook, scorer,
scored) that no robot code sets."""
import ast
import functools
import re
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
SEARCH = [HK / "sim" / "house", HK / "sim" / "arm", HK / "sim" / "dimos", HK / "src" / "houndkeeper"]   # the sys.path the
                                                                                                    # robot's code sets up

ROBOT = [   # what runs as the robot: the night's modules, the route, the floor pipeline, the confirmer
    "src/houndkeeper/tidy_module.py", "src/houndkeeper/floor_overlay_module.py", "src/houndkeeper/battery_module.py",
    "src/houndkeeper/phone_cockpit.py", "src/houndkeeper/cockpit_data.py", "src/houndkeeper/mind.py",
    "src/houndkeeper/morning_log.py", "src/houndkeeper/dimos_robot.py", "src/houndkeeper/localize.py",
    "src/houndkeeper/live_localize.py", "src/houndkeeper/calibrate.py", "src/houndkeeper/floor_register.py",
    "src/houndkeeper/lens.py", "src/houndkeeper/params.py", "src/houndkeeper/recordings.py",
    "src/houndkeeper/candidate_check.py",                                             # the light check per candidate ([Zaf])
    "src/houndkeeper/real_night.py",                                                  # the dog-alone night's preflight ([jFy])
    "sim/house/sweep_route.py",                                                       # the route
    "sim/house/floor_mosaic2.py", "sim/house/tall_layer.py", "sim/house/baseline_views.py", "sim/house/stitch_floor.py",
    "sim/house/calibrate_mount.py", "sim/house/house_frame.py", "sim/house/house_shape.py", "sim/house/tonight_frame.py",
    "sim/house/house_map.py", "sim/house/inject_map.py", "sim/house/real_nights.py",
    "sim/house/floor_sample.py",                                                      # the floor pipeline
    "sim/house/floor_refine.py",                      # an experiment beside the stitch ([G9q]), held to the robot's side
    "sim/house/joint_graph.py",                       # one pose graph with the floor in it ([JzW]), the robot's side
    "sim/house/loop_verify.py",                       # loop closures verified before DimOS's graph takes them ([JzW])
    "sim/house/live_place.py",                        # tonight placed live by the house's graph ([x9X])
    "sim/house/confirm_stream.py", "sim/house/detect_changes2.py",                    # the confirmer and its detector
]
ROBOT_SHELL = ["sim/house/night_picture.sh", "sim/dimos/stitch_baselines.sh", "sim/dimos/near_views.sh", "sim/house/real_nights.sh",
               "real/night.sh"]                                                  # each python
                                                                                  # step these run is robot code too
GENERATOR = {   # the house generator: it draws the house and its nights, and knows everything about them
    "make_house", "make_nights", "movables", "cabinets", "slate_tile", "house_items", "house_toys"}
SIM = {   # the sim around the robot (its renderer, its camera model, its settings, the hook that builds its scene): not
          # followed; robot code importing one is a breach only where that module itself names a truth file
    "sim_hook", "filament_cameras", "camera_imperfect", "env", "effective_config"}
SCORERS = {   # scored against the truth, after the fact
    "metrics", "score_clutter", "score_movables", "score_tidy", "floor_fidelity", "floor_truth2",
    "grade_localization", "twin_score", "difficulty", "round_summary", "vslice_summary",
    "score_confirm", "score_detect2", "score_floor_model", "morning_score",
    "detect_changes"}   # the old detector (v1, frozen), run and scored together over old runs   # the confirmer's replay, the locked
                                    # detector, the floor model and the morning log, each scored beside the robot's code
ANALYSIS = {   # benches, diagnostics and reports over finished runs, run by the engineer, never by the robot; a pipeline
    "run_speed",   # a finished run's speed from its own logs ([5SS])
    "calibrate_lag", "calibrate_robot_mount",   # the robot's calibration, made offline over tuning houses; they read the
                                                # generator only for its hold-out registry, to refuse those ([gJq])
               # script may run one as a report step (real_nights.sh: floor_agreement), which is not followed either
    "bench_detect", "bench_mapping", "check_funnel_top", "floor_agreement", "subsystem_health", "map_model",
    "house_twin", "gpu_share", "pick_bringup", "arm_bringup", "sweep_compare", "benchmark_release",
    "true_path",   # the offline pose ablation: the robot's floor tools run on the sim's true path
    "floor_refine_bench",   # [G9q] measured: the stitch against the floor refine, scored
    "joint_bench",   # [JzW] measured: the joint graph against the chain, scored on the true path
    "camgeo",   # [4t7] the camera's floor geometry: the diagnosis reads the sim's truth; the self-calibration is the
                # robot's own (its views alone), run offline over tuning houses as the mount's calibration ([gJq])
    "area_excursion",   # [rLQ] measured: feet outside the operating area, from the sim's truth
    "project", "project_stack",   # the Last night page and its launcher ([RzM]): a night's score beside its log
    "night_scoring"}   # [vO9] [71R]: a real night scored from the user's marks and judgements, on the project service
FORBIDDEN_IMPORTS = GENERATOR | SCORERS                    # robot code importing one of these is a breach by itself
TRUTH = re.compile(r"truth\.json|\.truth\b|score\.json|score\.txt|house_floor_s|traj\.csv|true_(x|y|yaw)\b"
                   r"|DIMOS_SIM_TRUTH|HK_HOUSE_SEED")      # names that only a truth file or the generator's seed carries


def _resolve(name: str, here: Path) -> Path | None:
    """a module name as the robot's code imports it -> its file in the repository, or None (DimOS, the libraries)"""
    parts = name.split(".")
    if parts[0] == "houndkeeper":
        p = HK / "src" / Path(*parts).with_suffix(".py") if len(parts) > 1 else None
        return p if p and p.exists() else None
    if len(parts) != 1: return None
    for d in [here.parent, *SEARCH]:
        if (d / f"{name}.py").exists(): return d / f"{name}.py"
    return None


def _docstrings(tree) -> set[int]:
    out = set()
    for n in ast.walk(tree):
        if isinstance(n, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and n.body \
           and isinstance(n.body[0], ast.Expr) and isinstance(n.body[0].value, ast.Constant): out.add(id(n.body[0].value))
    return out


def _refusals(tree) -> set[int]:
    """string constants in a refusal list (an UPPER_CASE name with TRUTH in it: the names a reader refuses to open)"""
    out = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id.isupper() and "TRUTH" in t.id for t in n.targets):
            out |= {id(c) for c in ast.walk(n.value) if isinstance(c, ast.Constant)}
    return out


def scan(path: Path):
    """(imports [(module name, line)], truth names [(text, line)]) of one file, read once while it is unchanged"""
    return _scan(path, path.stat().st_mtime_ns)


@functools.lru_cache(maxsize=None)
def _scan(path: Path, _mtime: int):
    """scan's reading: help texts, docstrings and refusal lists are prose, not opens"""
    tree = ast.parse(path.read_text(), str(path)); skip = _docstrings(tree) | _refusals(tree)
    for n in ast.walk(tree):                            # an argparse help= is prose about an option, not an open
        if isinstance(n, ast.keyword) and n.arg == "help": skip |= {id(c) for c in ast.walk(n.value)}
    imports, names = [], []
    for n in ast.walk(tree):
        if isinstance(n, ast.Import): imports += [(a.name, n.lineno) for a in n.names]
        elif isinstance(n, ast.ImportFrom) and n.module and not n.level:
            imports.append((n.module, n.lineno))
            if n.module == "houndkeeper": imports += [(f"houndkeeper.{a.name}", n.lineno) for a in n.names]
        elif isinstance(n, ast.Call) and n.args and isinstance(n.args[0], ast.Constant) and isinstance(n.args[0].value, str):
            f = n.func; fn = f.id if isinstance(f, ast.Name) else f.attr if isinstance(f, ast.Attribute) else ""
            if fn in ("__import__", "import_module"): imports.append((n.args[0].value, n.lineno))
        if isinstance(n, ast.Constant) and isinstance(n.value, str) and id(n) not in skip and TRUTH.search(n.value):
            names.append((TRUTH.search(n.value).group(0), n.lineno))
    return imports, names


def robot_files() -> dict[Path, str]:
    """every file robot code reaches, each with how it was reached (the first chain found)"""
    return dict(_robot_files())


@functools.lru_cache(maxsize=1)
def _robot_files() -> tuple:
    """robot_files' walk, from the roots and each python step the robot's shell scripts run"""
    roots = [HK / r for r in ROBOT]
    for sh in ROBOT_SHELL:
        for m in sorted(set(re.findall(r"([A-Za-z_]\w*)\.py\b", (HK / sh).read_text()))):
            p = _resolve(m, HK / sh)
            if p and m not in ANALYSIS | SIM | FORBIDDEN_IMPORTS: roots.append(p)
    seen = {r: r.relative_to(HK).as_posix() for r in roots}; todo = list(roots)
    while todo:
        f = todo.pop(); imports, _ = scan(f)
        for name, _line in imports:
            p = _resolve(name, f)
            if p and p not in seen and p.stem not in FORBIDDEN_IMPORTS | SIM:
                seen[p] = f"{seen[f]} -> {p.relative_to(HK).as_posix()}"; todo.append(p)
    return tuple(seen.items())


def findings() -> dict[str, list[str]]:
    """every breach, 'file: what' -> ['line (via the chain of imports robot code reached it by)', ...]"""
    out: dict[str, list[str]] = {}
    for f, how in sorted(robot_files().items()):
        imports, names = scan(f); rel = f.relative_to(HK).as_posix()
        for name, line in imports:
            p = _resolve(name, f)
            if p and p.stem in FORBIDDEN_IMPORTS:
                what = f"imports {p.stem} ({'the generator' if p.stem in GENERATOR else 'a scorer'})"
            elif p and p.stem in SIM and scan(p)[1]: what = f"imports {p.stem} (the sim's own, which names a truth file)"
            else: continue
            out.setdefault(f"{rel}: {what}", []).append(f"{line} (via {how})")
        for text, line in names:
            out.setdefault(f"{rel}: names '{text}'", []).append(f"{line} (via {how})")
    return out


KNOWN: dict[str, str] = {}   # breaches known and not yet fixed, each with its lines: the 12 found by the guard's first run
                             # (3 Oct 2026) are fixed; a new one is fixed where it lies, not listed here


ALLOWED = {   # truth-file names robot code may carry, each with why it is no read of the truth
    "src/houndkeeper/morning_log.py: names 'traj.csv'":
        "the night's odometry: Night.odometry loads only traj.csv's ODOM columns (its true_* are never read; "
        "test_morning_log scrambles them and the log is unchanged), and the name labels the route's source",
    "src/houndkeeper/calibrate.py: names 'traj.csv'":
        "the clock a recording was stamped on ([55G]): recording_clock reads only the path file's clock columns "
        "(wall_t, sim_t, host_t), never its true_* (test_lag_per_clock scrambles them and the clock is unchanged)",
}
INJECTED = ("pose_hook", "scorer", "scored")   # how a scorer or analysis tool hands robot code the truth


def test_robot_code_reads_no_truth():
    """[1B2]: no file the robot's code reaches imports the generator or a scorer, or names a truth file, beyond the
    breaches already known (KNOWN, each listed with its lines) and the reads allowed (ALLOWED, each with why)"""
    new = [f"{k}: line {', '.join(v)}" for k, v in findings().items() if k not in KNOWN and k not in ALLOWED]
    assert not new, "robot code reaching the generator's truth:\n" + "\n".join(new)


def test_no_known_breach_is_left():
    """[1B2]: no known breach is left: a breach found is fixed, not listed"""
    left = [k for k in KNOWN if k in findings()]
    assert not left, "known breaches still there:\n" + "\n".join(f"{k} ({KNOWN[k]})" for k in left)


def test_known_breaches_are_still_there():
    """[1B2]: every KNOWN entry still names a breach the guard finds: a fixed one is dropped from the list, so the list
    never hides a new breach of the same kind"""
    gone = [k for k in KNOWN if k not in findings()]
    assert not gone, "fixed, drop from KNOWN:\n" + "\n".join(gone)


def test_allowed_reads_are_still_there():
    """[1B2]: every ALLOWED entry still names something the guard finds, so the list never outlives its reason"""
    gone = [k for k in ALLOWED if k not in findings()]
    assert not gone, "no longer found, drop from ALLOWED:\n" + "\n".join(gone)


def injections(path: Path) -> list[str]:
    """the calls in a file that set an injected truth option (INJECTED) to anything but None or a parameter of the same
    name passed on"""
    out = []
    for n in ast.walk(ast.parse(path.read_text())):
        if isinstance(n, ast.Call):
            for k in n.keywords:
                if k.arg in INJECTED and not (isinstance(k.value, ast.Constant) and k.value.value is None) \
                   and not (isinstance(k.value, ast.Name) and k.value.id == k.arg):
                    out.append(f"{path.relative_to(HK).as_posix() if path.is_relative_to(HK) else path.name}:{n.lineno} sets {k.arg}")
    return out


def test_robot_code_never_injects_the_truth():
    """[1B2]: no file robot code reaches sets pose_hook, scorer or scored (only the scorers and analysis tools do)"""
    bad = [x for f in robot_files() for x in injections(f)]
    assert not bad, "robot code injecting the truth:\n" + "\n".join(bad)


def test_the_injection_guard_catches_one(tmp_path):
    """[1B2]: a call setting an injected option is found; passing None, or one's own parameter on, is not"""
    f = tmp_path / "robot_step.py"
    f.write_text("def g(a, pose_hook=None):\n    a.main(pose_hook=pose_hook); a.main(scorer=None); a.main(scored=lambda n, k: 1)\n")
    assert injections(f) == ["robot_step.py:2 sets scored"]


def test_every_module_is_placed():
    """[1B2]: each module in sim/house, sim/dimos and src/houndkeeper is robot code (reached from the roots) or named as
    the generator, the sim, a scorer or an analysis tool, so a new one is placed when it is written"""
    reached = {p.stem for p in robot_files()}; placed = reached | GENERATOR | SIM | SCORERS | ANALYSIS
    loose = [p.relative_to(HK).as_posix() for d in (HK / "sim" / "house", HK / "sim" / "dimos", HK / "src" / "houndkeeper")
             for p in sorted(d.glob("*.py")) if p.stem not in placed and p.stem != "__init__"]
    assert not loose, "modules on neither side of the truth boundary:\n" + "\n".join(loose)


def test_the_guard_catches_a_breach(tmp_path):
    """[1B2]: a robot file that imports the generator, or opens a night's truth, is found; a refusal list, a help text
    and a docstring are not"""
    bad = tmp_path / "robot_step.py"
    bad.write_text('"""reads no truth.json"""\nimport make_house as mh\nTRUTH_NAMES = (".truth.json",)\n'
                   'import argparse\nargparse.ArgumentParser().add_argument("--x", help="not traj.csv")\n'
                   'def f(run, s, n):\n    return open(f"{run}/nights_s{s}_n{n}.truth.json")\n')
    imports, names = scan(bad)
    assert ("make_house", 2) in imports and names == [(".truth", 7)]


def test_the_generator_is_never_reached():
    """[1B2], [HuM]: the house generator is never among the files robot code reaches, the route included"""
    reached = {p.stem for p in robot_files()}
    assert "sweep_route" in reached and not reached & GENERATOR
