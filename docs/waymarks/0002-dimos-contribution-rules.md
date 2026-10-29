# [2SB] — What DimOS asks of a contribution

**Status:** Proposed · **Date:** 2026-09-24 · **Source:** DimOS `CONTRIBUTING.md` and `AGENTS.md` on upstream main 6043c7371 · **Builds on:** [1kb] · **ID:** [2SB]

## Context

Two deliverables are upstream contributions: the Maker Arm adapter ([oIS]) and the legged manipulator sim ([RJi]).
Whether they are accepted is decided by Dimensional's rules, not ours. Observed 2026-09-24 on upstream main 6043c7371
(`CONTRIBUTING.md`; branch naming enforced since #4256):

- Robot or platform support, and other core architecture changes, start as a GitHub discussion before any code.
  Non-trivial work maps to an accepted issue first.
- A signed Contributor License Agreement is required.
- Branches are named `<who>/<type>/<topic>`, with `type` one of feat, fix, refactor, docs, test, chore, perf, build,
  ci, style, revert. PRs target `main`.
- `uv run pytest` and pre-commit run locally before pushing; no force-pushes (each push costs about an hour of CI).
- The PR description template is filled in completely, including how to test in sim, replay or hardware.
- Motion, control and driver changes say which robot and firmware they were tested on, or that they were not.
- The contributor must understand and be able to explain everything submitted; AI use is disclosed per `AI_POLICY.md`.

## Decisions

1. [Msc] **Both contributions open with a discussion,** describing the adapter or the sim extension, before code is
   offered as a PR.
2. [MMl] **Branches follow `nick/<type>/<topic>`** in the fork from the start, so nothing has to be renamed to go
   upstream.
3. [hN0] **Re-read these rules before each PR,** since they changed between the fork point and upstream main.
