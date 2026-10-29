# Public repository

**Type:** Capability spec · **Status:** Draft · **Source:** the repository opened for a conference talk, with no details
about the house or the people in it, and no secrets; principles [zGo], [JHu] · **ID:** [6kF]

## Purpose

The repository is public. Everything git carries can be read by anyone, so what it carries is held to what a stranger
may read: the robot, the method and the sim, and nothing of the people or the house it was built in.

## Acceptance criteria

1. [4i5] **Nothing identifies or describes anyone who lives in the house.** No name, health condition, age or presence. A rule that came from one (no visible flashing, no children in visual data) is stated as the
   project's rule, with no person behind it.
2. [TOg] **Nothing describes the real house.** No layout, room, colour, floor, finish or furnishing measured from it or
   drawn after it; the sim's houses are made from seeds and public assets alone.
3. [8kb] **Nothing names the machines it was built on.** No home-directory path, account name, host name or network detail;
   paths are relative to the repository or to a stated workspace.
4. [qdZ] **Every link the repository makes into itself resolves, and none points into private notes.** Run outputs
   the code writes and fetched assets may be named; private notes may not.
5. [5NA] **Licence and credit travel with it.** The project's own work is under the MIT licence; every third-party
   asset, model or tool it carries keeps its own licence and attribution.

## Decisions

1. [qK3] **History starts at the public snapshot**. The private history is archived outside the
   repository and work continues on the public one; nothing before the snapshot is published.
2. [uHt] **The sim houses drawn after the real one are reworded, not regenerated**: their colours
   and tiles stay as values with no account of where they came from, so every baseline and result stays comparable.
3. [cy4] **The workspace is the directory holding this repository, or `HK_WORKSPACE`** (conventional, 28 Oct 2026):
   the sibling checkouts and outputs (`dimos`, `dimos-ext`, `dimos-twin`, `maker-arm-sdk`, `build`) are found under
   it, and DimOS is `HK_DIMOS` when that is set. Python asks `houndkeeper.workspace`; a script works it out from its
   own path. Nothing names a home directory, and nothing outside the workspace has a default path: a private
   repository the code may read (the films) is an environment variable with no default (`HK_DEMOS_DATA`).

## Tests

| Criterion | Test | Status |
|---|---|---|
| [4i5] (no one who lives there) | `tests/test_public_repository.py`: every carried file searched for household and health wording; a planted line fails | Planned |
| [TOg] (no description of the house) | the same: wording that ties a value to the real house, its floors or footage of it | Planned |
| [8kb] (no machine names) | the same: home-directory paths, `$HOME/...`, `~/...`, `Path.home()` or `expanduser` joined with a project (dot-folders pass), each with planted lines; the workspace helper ([cy4]) honours `HK_WORKSPACE` and `HK_DIMOS`; `tests/test_no_network_specifics.py` for the network ([Fhn]) | Failing: one line in `docs/backlog.md` names a Desktop folder (28 Oct 2026) |
| [qdZ] (links resolve) | the same: every repository path a Markdown file links to exists in git; no path into `notes/` | Planned |
| [5NA] (licence and credit) | the same: an MIT `LICENSE` at the root; every used asset the manifest marks as needing credit named in `ATTRIBUTIONS.md` (with a planted miss); `tests/test_benchmark_release.py` for the assets ([h1X]) | Planned |
