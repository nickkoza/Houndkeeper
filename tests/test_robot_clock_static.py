"""[3Pv]: every time constant on the robot's side that judges sim data (pose gaps, timeouts, control rates, waits) reads
the robot's clock (dimos.utils.clock: the wall clock on the robot, sim time in the sim), never the host's wall clock
directly. This lists every wall-clock read in houndkeeper's own modules; each is either the host's business (named
here with why) or a finding."""
import ast
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src" / "houndkeeper"
WALL = {"time", "monotonic", "perf_counter", "sleep", "time_ns", "monotonic_ns"}

# (module, function): why the host's own clock is right there
HOST_SIDE = {
    ("recordings.py", "add"): "when a recording was filed in the catalog: the host's bookkeeping",
    ("recordings.py", "scan"): "a recording's files' age on the host's disk",
    ("recordings.py", "sync"): "when a copy was verified: the host's bookkeeping",
    ("recordings.py", "preflight"): "an isolation check's age on the host's disk",
    ("recordings.py", "session"): "which recordings appeared since the host started a session",
    ("gpu_share.py", "share"): "waiting for another process's GPU lock on the host",
    ("sim_hook.py", "display_awake"): "the sim process asks the host's X server at most every 10 s",
    ("sim_hook.py", "viewer"): "the sim process redraws the viewer's HUD four times a wall second",
    ("operating_area.py", "save"): "when the user changed the area: a record of a person's action, on the host's clock",
}
# found, not yet moved to the robot's clock: (module, function) -> what it should read
LISTED = {
}


def wall_reads():
    """(module, enclosing function, line) of every time.time()/monotonic()/perf_counter()/sleep() call"""
    out = []
    for p in sorted(SRC.glob("*.py")):
        tree = ast.parse(p.read_text())

        def walk(node, scope):
            for ch in ast.iter_child_nodes(node):
                if isinstance(ch, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                    walk(ch, scope + [ch.name]); continue
                if (isinstance(ch, ast.Call) and isinstance(ch.func, ast.Attribute) and ch.func.attr in WALL
                        and isinstance(ch.func.value, ast.Name) and ch.func.value.id == "time"):
                    out.append((p.name, ".".join(scope), ch.lineno))
                if isinstance(ch, ast.ImportFrom) and ch.module == "time" and any(a.name in WALL for a in ch.names):
                    out.append((p.name, ".".join(scope) or "<module>", ch.lineno))
                walk(ch, scope)
        walk(tree, [])
    return out


def test_the_robot_side_reads_time_through_the_robot_clock():
    """[3Pv]"""
    found = [(m, f, ln) for m, f, ln in wall_reads() if (m, f) not in HOST_SIDE and (m, f) not in LISTED]
    assert not found, "read the robot's clock (dimos.utils.clock now/sleep/wait), or say why the host's is right: " + \
        ", ".join(f"{m}:{ln} in {f}" for m, f, ln in found)


def test_the_lists_name_only_reads_that_exist():
    """[3Pv]: an entry outlives its read only by mistake"""
    have = {(m, f) for m, f, _ in wall_reads()}
    assert not (set(HOST_SIDE) | set(LISTED)) - have, (set(HOST_SIDE) | set(LISTED)) - have
