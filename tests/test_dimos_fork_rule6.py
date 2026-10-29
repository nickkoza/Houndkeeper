"""[UvE] rule 6 (docs/dimos-fork.md): the DimOS fork's changes are extension points or general features. Their diff
(sim/dimos/dimos_fork_changes.patch, regenerated from the fork) names no houndkeeper, reads no HK_ variable and cites
none of our spec IDs: our values and our reasons stay on our side."""
import re
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
PATCH = HK / "sim" / "dimos" / "dimos_fork_changes.patch"


def _added_lines():
    file = None
    for line in PATCH.read_text().splitlines():
        if line.startswith("+++ "):
            file = line[6:] if line.startswith("+++ b/") else line[4:]
        elif line.startswith("+") and file:
            yield file, line[1:]


def _our_ids() -> set:
    ids = set()
    for d in ("specs", "waymarks", "principles"):
        for f in (HK / "docs" / d).glob("*.md"):
            t = f.read_text()
            ids |= set(re.findall(r"\*\*ID:\*\* \[([A-Za-z0-9]{3})\]", t))
            ids |= set(re.findall(r"^\s*\d+\. \[([A-Za-z0-9]{3})\]", t, re.M))
    return ids


def test_the_fork_names_no_houndkeeper():
    bad = [(f, ln.strip()[:100]) for f, ln in _added_lines() if re.search(r"houndkeeper|grant sim|maker.?arm", ln, re.I)]
    assert not bad, bad


def test_the_fork_reads_no_hk_variable():
    bad = [(f, ln.strip()[:100]) for f, ln in _added_lines() if re.search(r"\bHK_[A-Z]", ln)]
    assert not bad, bad


def test_the_fork_cites_none_of_our_spec_ids():
    ids = _our_ids()
    assert len(ids) > 100                       # the registry was read
    bad = [(f, i) for f, ln in _added_lines() for i in re.findall(r"\[([A-Za-z0-9]{3})\]", ln) if i in ids]
    assert not bad, bad
