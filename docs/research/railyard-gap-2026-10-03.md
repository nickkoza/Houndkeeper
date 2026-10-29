# Houndkeeper against the Railyard Method: gaps, and what to take back (3 Oct 2026)

> **Why this is here:** it measured this repository against the Railyard Method on 3 Oct 2026 (0 of 21 commit subjects then named a criterion), and its list of what to take back shaped how the method is used here since.

The question: what gaps houndkeeper has against the Railyard Method, and what of it could go back into Railyard and
the method. Static review only: git, the artifacts, `railyard.mjs check`. Nothing was changed in
the existing specs or code as part of this review.

## Where houndkeeper stands

- **The skill copy is current.** `.claude/skills/spec-driven-change/` (installed 28 Sep in 4f9026e) is byte-identical
  to the method's newest release, `release/1.0.3` (`ca48def`, 27 Sep): `SKILL.md` and `tools/railyard.mjs` both.
  `.railyard/method.json` says 1.0.3, and both scan hooks (edits and shell commands) are installed.
- **Upstream is the one that is out of step.** The method repository's `main` is 13 commits *behind* `release/1.0.3`
  (a strict ancestor of it): it still declares method 0.2.0 and lacks the method's LeX rule (IDs that read as objectionable words are
  never minted), while its own eval results of 27 Sep are in no release. Re-installing the skill from `main` would
  downgrade houndkeeper. Merging `release/1.0.3` into `main` is a fast-forward. (Reported in the method branch's
  proposals README; not changed.)
- **`check` is clean**: no findings, 5 notices. Four are the check's lLm rule: `sim/house/bench_mapping.py`,
  `sim/showcase/dog_video.py`, `src/houndkeeper/filament_cameras.py` and `src/houndkeeper/floor_overlay_module.py`
  add to `sys.path` in a way the check cannot follow, so their imports are not held to the model. One is its G0i rule: 6
  source files have no links.
- **By the numbers**: 28 specs, 229 criteria, 232 Tests rows (46 Passing, 9 Written, 1 Measured, 73 Partial,
  5 Failing, 19 Planned, 78 Not yet written, 1 Withdrawn); 12 principles; 12 waymarks; a CALM model of 37 nodes and
  63 relationships, 13 owning code by `source-path`; 232 links files; 4 findings under `docs/findings/`, 11 documents
  under `notes/` before this one.

## Gaps against the method, ranked by value

| # | Gap | Size | Fix |
|---|---|---|---|
| 1 | **The findings left the method.** The last finding under `docs/findings/` is 25 Sep. Everything since (the 3 Oct red teams, the network audit, the sim-kindness audit, the pipeline critique, the static sweep, the tile-floor and trimmed-mean studies) is in `notes/`: untracked, outside `docs/`, not in the marker's `dated` list, so `check` and `trace` never read it and nothing in the specs points to it. Three of the eleven cite no ID at all. | 11 documents, about 190 KB of the project's freshest evidence | Move the dated findings to `docs/findings/` (already `dated`), or add `notes/` to `dated`; each audit's recommended actions become criteria or decisions before they are fixed, and each finding is closed by naming that ID. |
| 2 | **A third of the Tests rows have no test.** 78 rows are "Not yet written" and 19 Planned. The method: "A criterion with no test is a wish." Some are honestly hardware-bound (the Pi, the dog); many are sim behaviour that is built. | 97 of 232 rows | Triage: rows whose behaviour is built get their test (or are marked Partial with what holds them); hardware-bound ones become Planned with the first-boot milestone named. The map below shows them spec by spec. |
| 3 | **Measured figures sit in status notes.** 36 rows carry a date in their status note and 8 a figure ("Written: 43/62 pickable binned (69%) on the core items, 25 Sep"). The method says a measured number is "never in ... a status". Houndkeeper's practice is the more useful one *when the note is dated*; the undated ones are the risk. | 36 dated, 8 with figures | Keep dated notes, name where each figure was measured, date the undated. Proposed upstream as "a status note says as of when" (method proposal 5). |
| 4 | **Quoted requests sit inside criteria.** 28 of 229 criteria carry a dated quotation of who asked, "(name, date: "...")". The method (the LeX commit's source note, and its spec shape): a criterion states the requirement only; the words of the request are the source of the spec or of a decision. The project's own memory rule already says dated quotes belong in Decisions. Railyard has the same habit (34 of 582). | 28 criteria | Move each quotation to its spec's `Source` line or to a decision, verbatim. Proposed upstream as a readable form for quotations (method proposal 6). |
| 5 | **A superseded waymark.** `docs/waymarks/0003-servo-gravity-compensation.md` is "Superseded by [z7P]". The method: a reversed choice is rewritten where it stands or deleted, never superseded; git keeps the history. Waymarks also carry `Status`, `Builds on` and `Serves` fields and an `[ID] —` title the method's shape doesn't have (harmless extensions). | 1 waymark | Delete 0003 (its content is in [z7P] and in git), re-point anything citing [Oav]. |
| 6 | **Boundaries the check cannot see.** The four `sys.path` inserts above leave those files' imports unchecked against the CALM model; the 24 of 37 nodes with no `source-path` are hardware, networks, external services or DimOS's own code (legitimately code-less here). Only the system spec has a view drawn from the model; the specs about boundaries (arm host, recordings, dog first boot, robot segment) have none. | 4 files; 1 view | Replace the inserts with a package import or a path from the repository's root that the check reads; add a view to the arm-host and dog-first-boot specs. |
| 7 | **Commits cite no IDs.** 0 of 21 commit subjects name an ID; the method's and Railyard's own commits end with them. Not a written rule, but it is the cheapest trail from a change to its requirement, and the map has to rebuild it from links instead. | every commit | End commit subjects with the IDs they serve. |
| 8 | **Six source files unlinked** (the check's G0i rule): `sim/arm/profile_multi.py`, `sim/dimos/score_tidy.py`, `sim/house/floor_truth.py`, `sim/house/overlay.py`, `sim/house/render_nights.py` and one more. | 6 files | Link as they are next touched (the method's own advice). |
| 9 | **Cross-repository citations.** Houndkeeper cites DimOS and the method in prose; Railyard cites the method as `[method@1c98e:ID]`, which survives a rename. `.railyard/method.json` names only the method repository. | low | Add `dimos` to `repositories` once DimOS keeps IDs; until then, commit-pinned prose is right. |

Not gaps: decisions are all numbered with IDs (`decisions-by-id` would change nothing), every Tests row names its
criteria by ID, every spec has a `## Purpose`, the principles file is in the method's shape, and the skill's spec
metadata extension (`Type · Status · Source · ID`) is the same one Railyard uses.

## What houndkeeper invented, ranked by how well it travels

| # | Practice | Where it lives here | Travels? | Taken back as |
|---|---|---|---|---|
| 1 | **Engineering made visible**: one page from principles to evidence | (new) the project map | Yes: every method repository has the same artifacts | Method proposal 1, a `railyard map` command; Railyard L1, L2, L4 |
| 2 | **Provenance on every measurement**: `hk_commit` (+dirty), `fork_patch_sha`, `params_sha`, `sha256` on every recording ([xOp]) | `src/houndkeeper/recordings.py` | Yes: any benchmark or eval; the method's own eval results record no commit or skill hash | Method proposal 2, with `tools/provenance.ts` |
| 3 | **Hold-out discipline**: fixed before it is run, margin first, results only, spent once, failures become tuning work (sim-slice [bnO]) | sim-slice; the agent's memory | Yes: any measured claim, including the method's evals, which are re-run after fixes on the only set they have | Method proposal 3 |
| 4 | **"The sim is never kinder" ([x3f])** and the imperfection register ([nAH]), plus the audit against it | principles; waymark 0008; `docs/findings/sim-kindness-audit-2026-10-03.md` | Yes, as "a stand-in is never kinder than what it stands for": fakes, mocks, recorded responses | Method proposal 4 |
| 5 | **Dated status notes with figures** | 36 Tests rows | Yes, once reconciled with the method's rule | Method proposal 5; Railyard L1 |
| 6 | **Requests quoted verbatim and dated** in specs' `Source`, decisions and principles | throughout | Yes; the method already leans this way | Method proposal 6; Railyard L3 |
| 7 | **Adversarial passes** written as dated findings (two red teams, a network audit, a principle audit, a pipeline critique, a static sweep on 3 Oct) | `notes/` | Yes, and Railyard invented the same ("Adversarial passes", 9 reviews) | Method proposal 7 |
| 8 | **Contact sheets** before a render is sent | `src/contact_sheet.py` in the separate demos repository (`houndkeeper-demos`) | Yes, as "look at what you made before you hand it over" | Method proposal 8; Railyard L5 |
| 9 | Story log for the retrospective, demo clips raw, never boring | private video notes; the agent's memory | Partly: "never boring, tell stories" shaped the map's design; the rest is this project's | Not proposed |
| 10 | Project safety rules (no visible flashing, no children in visual data, nothing seen leaves the house) | principles [0Py], [JHu], [zGo] | No: they are this project's, and stay here | Not proposed |

## Where the drafts are

Nothing is committed; both are branches in new worktrees, cut so the main checkouts and their uncommitted work are
untouched.

- **Railyard Method**: a worktree of the method repository, branch `feat/from-houndkeeper` from `release/1.0.3` (not `main`, which
  is behind it). `proposals/from-houndkeeper/README.md` indexes eight proposals with their draft text;
  `tools/provenance.ts` (+ test) is proposal 2's code; `docs/specs/map.md` and `method/map*.ts`,
  `bin/railyard-map.ts` are proposal 1 (method spec k6t, 17 tests passing). `node_modules` there is a symlink to another checkout's.
- **Railyard**: a worktree of the Railyard repository, branch `feat/from-houndkeeper` from `main`.
  `docs/reviews/2026-10-03-from-a-method-project.md`: five Library and process proposals (declared beside verified
  status, a Findings chapter, quoted requests, the book printed, looking at the desk before handing back).

## Suggested order for houndkeeper

1. Bring `notes/` findings under the method (gap 1): it is the cheapest change with the most evidence behind it, and
   the map already reads `notes/` as dated so nothing is lost meanwhile.
2. Triage the 78 "Not yet written" rows (gap 2) with the map open: it shows them spec by spec.
3. Move the 28 quotations out of criteria and delete waymark 0003 (gaps 4, 5) in one artifact-only change.
4. Fix the four `sys.path` inserts (gap 6) the next time those files are touched.

## The project map (built)

- Generator: the method's proposed `map` command, `method/map*.ts` in that method worktree (spec k6t there); houndkeeper
  glue `tools/project_map.py` (spec [KfL], the project-map spec), which stages the page, leak-checks it
  (addresses, MACs, emails, values from the gitignored network files, read in memory and never printed) and only then
  writes `index.html`.
- Output (not published): `~/.claude/jobs/a53e8952/tmp/project-map/index.html`, with screenshots beside it.
- Before it goes anywhere: two quotes in the house-twin spec, [lQ1], carry a swear word and appear on the page;
  fix the spec and regenerate.
