"""public-repository [6kF]: what git carries is what a stranger may read. Every carried text file is searched for the
household ([4i5]), the real house ([TOg]), development machines ([8kb]) and links that go nowhere or into private notes
([qdZ]), and for the person behind a request rather than its reason (principle [JqF]); the licence is checked at the
root ([5NA]). Each search is first run on a planted line, so a pattern that
matches nothing can't pass for a clean repository."""
import re
import subprocess
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
SELF = {"tests/test_public_repository.py"}

HOUSEHOLD = re.compile(r"household (member|has)|household'?s (own )?(children|kids|photosensitiv|skeleton)"
                       r"|(someone|anyone|a person) (in|at) (the|this|our) house is|in this house\)|owner'?s (son|daughter|wife|husband|partner|kids?|child|children|family)"
                       r"|\b(his|her|their) (son|daughter|wife|husband|kids|children)\b", re.I)
HOUSE = re.compile(r"owner'?s (own )?(house|home|floor|carpet|tiles?|fireside|living room|office|kitchen|hallway|porch"
                   r"|phone footage|footage)|drawn (after|on) the owner|real-house/owner", re.I)
MACHINE = re.compile(r"/home/[a-z_][a-z0-9_-]*|/Users/[A-Za-z]"
                     r"|(\$\{?HOME\}?|(?<![\w.~/])~)/(?!\.)[\w-]"                 # $HOME/x, ~/x (a dot-folder like ~/.cache is fine)
                     r"|Path\.home\(\)\s*/\s*f?[\"'](?!\.)"                       # Path.home() / "a project"
                     r"|expanduser\(\s*f?[\"']~/?(?!\.)[\"\w]")                    # expanduser("~/a project")
NOTES = re.compile(r"(?<![\w./-])notes/[\w./-]+")
VOICE = re.compile(r'(?<!")\bthe owner\b|\bowner\'s\b|\((owner|user)[,;:]|\b(owner|user), [0-9]{1,2} [A-Z][a-z]{2}', re.I)
VOICE_EXEMPT = {".claude/skills/",                                   # the Railyard method, vendored as published
                "models/maker_arm/LICENSE",                          # third-party licence
                "sim/dimos/dimos_fork_changes.patch"}                # DimOS's code as the fork carries it
MD_LINK = re.compile(r"\]\(([^)#\s]+)(?:#[^)]*)?\)")

PLANTED = {HOUSEHOLD: "nothing flashes (a household member is photosensitive)",
           HOUSE: "the carpet colour measured from the owner's phone footage",
           MACHINE: ["run it from /home/someone/project", 'PY=$HOME/houndkeeper/dimos/.venv/bin/python',
                     "OUT=${OUT:-~/houndkeeper/build}", 'DIMOS = Path.home() / "houndkeeper" / "dimos"',
                     'SIM = os.path.expanduser("~/houndkeeper/sim")', "BUILD=${HOME}/build"],
           NOTES: "see notes/youtube-story.md",
           VOICE: ['round the room once (owner, 3 Oct 2026: "do a lap first")', "until the owner confirms it",
                   "on the owner's phone", "(user, 24 Sep 2026)"]}
NOT_MACHINE = ['LOCK = Path.home() / ".cache" / "houndkeeper"', "a key in ~/.ssh/id_ed25519.pub", "$HOME/.local/bin/deno",
               "the workspace, HK_WORKSPACE, or the repository's parent"]


def carried():
    r = subprocess.run(["git", "-C", str(HK), "ls-files", "-z", "--cached", "--others", "--exclude-standard"],
                       capture_output=True, check=True)
    for p in r.stdout.decode().split("\0"):
        if not p or p in SELF or not (HK / p).is_file(): continue
        try: yield p, (HK / p).read_text()
        except UnicodeDecodeError: continue           # binary: meshes, images, arrays


def hits(pattern):
    return [f"{p}:{n}: {line.strip()[:120]}" for p, text in carried() for n, line in enumerate(text.splitlines(), 1)
            if pattern.search(line)]


@pytest.mark.parametrize("pattern", list(PLANTED), ids=["household", "house", "machine", "notes", "voice"])
def test_each_search_finds_its_planted_line(pattern):
    lines = PLANTED[pattern] if isinstance(PLANTED[pattern], list) else [PLANTED[pattern]]
    for line in lines: assert pattern.search(line), line


def test_the_machine_search_passes_dot_folders_and_the_workspace():   # [8kb]: a cache or key folder names no project
    for line in NOT_MACHINE: assert not MACHINE.search(line), line


def test_the_workspace_is_the_repositorys_parent_unless_named(monkeypatch, tmp_path):   # [8kb], decision [cy4]
    from houndkeeper import workspace
    monkeypatch.delenv("HK_WORKSPACE", raising=False); monkeypatch.delenv("HK_DIMOS", raising=False)
    assert workspace.workspace() == HK.parent and workspace.dimos() == HK.parent / "dimos"
    monkeypatch.setenv("HK_WORKSPACE", str(tmp_path))
    assert workspace.sibling("maker-arm-sdk") == tmp_path / "maker-arm-sdk" and workspace.dimos() == tmp_path / "dimos"
    monkeypatch.setenv("HK_DIMOS", str(tmp_path / "fork"))
    assert workspace.dimos() == tmp_path / "fork"


def test_nothing_describes_a_household_member():   # [4i5]
    assert not hits(HOUSEHOLD)


def test_nothing_describes_the_real_house():       # [TOg]
    assert not hits(HOUSE)


def test_nothing_names_a_development_machine():      # [8kb]
    assert not hits(MACHINE)


def test_nothing_points_into_private_notes():      # [qdZ]
    assert not hits(NOTES)


def test_one_impersonal_voice():                   # principle [JqF]
    assert not VOICE.search('no "the owner", no quoting the person who asked')   # the rule may name what it forbids
    assert not [h for h in hits(VOICE) if not h.split(":", 1)[0].startswith(tuple(VOICE_EXEMPT))]


def test_every_markdown_link_into_the_repository_resolves():   # [qdZ]
    bad = []
    for p, text in carried():
        if not p.endswith(".md"): continue
        for target in MD_LINK.findall(text):
            if re.match(r"[a-z]+:", target) or target.startswith("/"): continue   # URLs, mail, site-absolute
            if not (HK / p).parent.joinpath(target).resolve().exists(): bad.append(f"{p}: {target}")
    assert not bad


def test_the_project_is_mit_licensed():            # [5NA]
    text = (HK / "LICENSE").read_text()
    assert text.startswith("MIT License") and "Permission is hereby granted, free of charge" in text


def _uncredited(assets, credits):
    return [a["id"] for a in assets if a["credit_required"] and a["used"] and a["id"].split("/", 1)[1] not in credits]


def test_every_asset_that_asks_for_credit_is_credited():   # [5NA]
    import json
    manifest = json.loads((HK / "sim" / "house" / "benchmark" / "asset_manifest.json").read_text())
    credits = (HK / "ATTRIBUTIONS.md").read_text()
    planted = {"id": "sketchfab/planted_lamp", "credit_required": True, "used": True}
    assert _uncredited([planted], credits) == ["sketchfab/planted_lamp"]          # the search can fail
    assert not _uncredited(manifest["assets"], credits)
    for author in [a["source"].split("by ", 1)[1] for a in manifest["assets"] if a["kind"] == "imported_model" and a["used"]]:
        assert author in credits, author
