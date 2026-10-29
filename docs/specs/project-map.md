# Project map

**Type:** Capability spec · **Status:** Draft · **Source:** the engineering made visible, with the gaps against the
Railyard Method found and what can move into Railyard and the Railyard Method brought out; principles [zGo], [JHu] · **ID:** [KfL]

## Purpose

The engineering behind Barrymore made visible on one page: the principles, the specs under them, each criterion and
where it stands, the code and tests that hold it, the source it came from, the findings and audits that cite it,
and what changed this fortnight. The page is made by the Railyard Method's own map command; this spec is what
Houndkeeper adds to it, which is mostly what must never be on it. A page made to be shown is the easiest way for the
house to leave the house ([zGo]).

## Acceptance criteria

1. [r5B] **The map is made from the repository by the method's map command, notes included.** The working notes under
   `notes/` (audits, critiques, overnight logs) are named as dated documents alongside the marker's own, so a criterion
   shows the notes that cite it.
2. [Nlv] **Nothing kept out of git is on the map.** No file git ignores is read to make it: not `real/.env`, not
   `real/local/`, not `data/`.
3. [NiP] **The map is checked for what must not leave before anyone sees it.** After it is made, the page is searched
   for network addresses, hardware addresses, email addresses, and every value kept in `real/.env` and every line of
   `real/local/`; a hit stops the run, naming the kind and how many, and never the value. A failed check leaves no page
   behind.
4. [6nG] **No dated document's text is on the map,** only its title, its date and the IDs it cites: notes quote the
   house and its network in passing, and the map is not where they are read.
5. [EZt] **The robot is Barrymore in anything the map says of its own.** The map's own words name no earlier name for
   the robot; the artifacts it shows say what they say.

## Decisions

1. [ZMl] **The method is found where the marker says, unless told otherwise.** `tools/project_map.py` runs the map
   command from `HK_RAILYARD_METHOD`, else from the `method` entry of `.railyard/method.json`'s `repositories`. Until
   the map command is in a method release, that is the method's own checkout on the branch that carries it.
2. [QCx] **A value from `real/.env` counts when it is at least six characters long, a line of `real/local/` at twelve,
   and either only where the repository's own files do not already carry it.** Shorter ones (a port, a yes or no)
   would match ordinary words and numbers on any page. The map is made from what git keeps, so what this check is for
   is anything that reached the page from elsewhere; a phrase the specs already write is the repository's own, and
   whether the repository should carry it is a question for the repository, not the map (found 3 Oct 2026: five
   ordinary phrases, all in the specs, stopped the first map).

## Tests

| Criterion | Test | Status |
|---|---|---|
| [r5B] (made by the method's command, notes as dated) | `tools/project_map.py` run on the repository; the method's own `method/map.test.ts` holds the page | Partial: made 3 Oct 2026 from the method's branch `feat/from-houndkeeper`, not yet a release |
| [Nlv] (nothing git ignores is read) | the method's `method/map.test.ts` (an ignored path refused, an ignored file unread); `tests/test_project_map.py` (a named ignored path refused before the map is made) | Written |
| [NiP] (checked before anyone sees it; kind and count, never the value) | `tests/test_project_map.py` (each kind found in a synthetic page, values never in the report, the page removed on a hit) | Written |
| [6nG] (no dated document's text) | the method's `method/map.test.ts` (a dated document's body absent from the page) | Written |
| [EZt] (Barrymore in the map's own words) | `tests/test_project_map.py` (the page's own text, outside the artifacts' data, names no earlier name) | Written |
