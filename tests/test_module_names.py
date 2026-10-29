"""[gB6]: no houndkeeper DimOS module defines a name DimOS's Module sets on its instances (waymark [ja6]): such a method
is hidden by the instance attribute without a word (a feed's thread given the event loop died at start). The names are
read from DimOS's own source, so a DimOS update that adds one is caught too; our modules are read as source (ast), so
nothing heavy is imported."""
import ast
import inspect
import re
import sys
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
pytest.importorskip("dimos")


def dimos_instance_names() -> set[str]:
    from dimos.core.module import Module
    names = set()
    for cls in Module.__mro__:
        if cls.__module__.split(".")[0] != "dimos": continue
        try: src = inspect.getsource(cls)
        except (OSError, TypeError): continue
        for line in src.splitlines():
            if "=" in line and "==" not in line.split("=")[0]:
                lhs = line.split("=")[0]
                names.update(re.findall(r"\bself\.(\w+)", lhs))
    return names


def our_module_classes():
    for p in sorted((HK / "src" / "houndkeeper").glob("*.py")):
        for node in ast.walk(ast.parse(p.read_text())):
            if isinstance(node, ast.ClassDef) and any(getattr(b, "id", getattr(b, "attr", None)) == "Module" for b in node.bases):
                defined = {n.name for n in node.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
                defined |= {t.id for n in node.body if isinstance(n, ast.Assign) for t in n.targets if isinstance(t, ast.Name)}
                defined |= {n.target.id for n in node.body if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name)}
                yield f"{p.name}:{node.name}", defined


def test_dimos_names_are_read():
    assert {"_loop", "_loop_thread"} <= dimos_instance_names()      # the pair that hid the feeds' threads (a7492431f)


def test_no_module_of_ours_defines_a_name_dimos_sets_on_its_instances():
    taken = dimos_instance_names() - {"__init__"}
    classes = dict(our_module_classes())
    assert {"cockpit_data.py:MorningLogFeed", "project.py:ProjectFeed"} <= set(classes)
    clashes = {cls: sorted(defined & taken) for cls, defined in classes.items() if defined & taken}
    assert clashes == {}
