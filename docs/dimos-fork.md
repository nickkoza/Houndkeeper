# Keeping our DimOS fork current

Why this exists: houndkeeper runs on a fork of DimOS, and these are the rules that keep the fork's changes small,
project-agnostic and easy to rebase or offer upstream. `tests/test_dimos_fork_rule6.py` holds rule 6.

The fork is published as a GitHub fork of dimensionalOS/dimos: one branch per change under `nick/feat/*`, and
`nick/feat/integration`, the branches merged, which is what houndkeeper runs on. How far it is behind upstream is
whatever `git fetch upstream` says on the day; this file does not track it.

1. **Remotes**: `upstream` = dimensionalOS/dimos, `origin` = our GitHub fork. Local `main` mirrors `upstream/main`; never commit to it.
2. **Branches** (upstream enforces `<who>/<type>/<topic>`, #4256), each merged into `nick/feat/integration`:
   - `nick/feat/maker-arm-adapter` - Maker Arm manipulator adapter (to PR)
   - `nick/feat/legged-manipulator-sim` - Go2 + arm in DimOS's MuJoCo sim (to PR)
   - `nick/feat/tf-interpolation` - interpolated tf lookups, a buffer clock, odometry in recording lookups ([x5h]; worktree `../dimos-tf`)
   - `nick/feat/keepout-zones` - keep-out zones and virtual walls as a costmap layer, the feet in the sim's truth
     trajectory ([u4W]; written 5 Oct 2026, not yet committed to its branch: in sim/dimos/dimos_fork_changes.patch)
   - `nick/feat/pgo-loop-verifier` - an extension point on the pose graph: `PGO(loop_verifier=...)` is handed each
     loop closure ICP proposes (`LoopCandidate`: the two keyframes' times and current poses, the proposed offset, the
     fitness and the two submaps) and may refuse it before it enters the graph; with none, nothing changes. DimOS's
     own tests in `test_pgo.py` (`TestLoopVerifier`). Our verifier (`sim/house/loop_verify.py`) stays on our side
     (written 8 Oct 2026, not yet committed to its branch: in sim/dimos/dimos_fork_changes.patch)
   - No local-only patches remain: the pass-under rule is set through DimOS's own costmap config (`sim/dimos/env.py`),
     and the sim speed option is a fork change (`sim/dimos/dimos_fork_changes.patch`).
3. **Cadence**: fetch upstream at the start of each work session (at least weekly) and rebase our branches onto it.
4. **After each sync**: `uv sync`; DimOS tests for the areas we touch; our suite smoke run (walk, grasp --trials 3, one short
   night with the viewer). The suite history records the DimOS commit for every result.
5. **Upstream after the real robot.** A branch is proved on real hardware before it is offered, since running there
   may show it needs more iteration. Branches are kept upstream-shaped (DimOS's
   style, their own tests, nothing of houndkeeper) and committed on their own branches; none is offered as a PR
   until it has run on the real robot for a while.
6. **Every change to DimOS is project-agnostic.** Either an **extension point** (a hook, a plugin interface, a
   config option, a loader: the cockpit's extensions, [Eir]) that houndkeeper's own code plugs into, or a **general feature** useful to any
   DimOS user (an interpolating tf lookup, a sim clock, a scene from a map, a Maker Arm driver). Nothing in DimOS
   names houndkeeper, reads its files or environment variables, or carries its constants; what is ours lives in
   houndkeeper and reaches DimOS through those points. A local patch that is neither (a houndkeeper default
   hard-wired in DimOS) is debt: listed here, and turned into a config option or an extension point.
7. **A sync script** (planned, not written): fetch, report how far behind, rebase, run the smoke suite, print a report.
   Never pushes.

Relevant upstream changes to read before building on them: #4271 (generalize perception stack), #3524 (benchmark replay).
