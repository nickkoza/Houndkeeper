# Recordings and the data they hold

**Type:** Capability spec · **Status:** Draft · **Source:** all the infrastructure needed for recording, built on what
DimOS has in this area, with the data easy and safe to manage;
principle [zGo] (nothing recorded in the house leaves it) · **ID:** [HqQ]

## Purpose

From first boot the real robot records: the calibration procedures, nights in the house, the first tidy. Those
recordings are the project's most valuable data and can't be made again; the sim's can. This keeps every recording,
real and sim, findable with what it is, when, why and under which code and parameters, safely stored (on the laptop
now, the home server once it is up), never outside the house, and cheap to use: a name to replay, measure or build a
floor map from.

DimOS records already (`dimos --record`: a SQLite `memory.db` per run under its checkout's `recordings/`, `dimos mem
summary` and `--replay-db`); this builds on that rather than beside it. DimOS also uploads recordings to Dimensional's
cloud (`dimos data upload`): that is closed for this project.

## Acceptance criteria

1. [xOp] **One catalog of every recording.** Each has an entry: an id, real or sim, the blueprint, start, length,
   streams (from the recording's own `_streams`), size, a SHA-256 of the file, the robot's firmware (real), the
   houndkeeper commit and the DimOS fork patch's hash, a hash of `params/robot.yaml` (and the file itself for a real
   recording), its purpose, tags and notes, and where its copies are.
2. [5zg] **Easy to find and use.** One command lists and filters recordings (real/sim, tag, purpose, date, house),
   shows one, adds tags and notes, and prints the path of its `memory.db` by id, so replay, `measure.py` and the
   night pipeline take a recording by id.
3. [fya] **Nothing leaves the house.** DimOS's cloud upload cannot run in this project: its endpoint is pointed at
   nothing in the DimOS checkout's configuration, and the catalog's tools refuse any destination outside the house's
   own machines.
4. [6xc] **A session records what a procedure needs, and checks first.** One command starts a recorded DimOS run
   with the streams the real-robot procedures need (the battery among them), refuses to start under a free-space floor
   or when the house's isolation check last failed, and catalogs the recording when the run ends.
5. [Zw9] **Two copies of every real recording.** Real recordings are copied to the home server, checksum-verified
   there; until the server is up they are listed as single-copy. A laptop copy is removed only after the server's copy
   is verified, and never automatically.
6. [B2O] **Room for more.** A retention command frees space by the sim recordings nothing refers to (no run directory,
   seed list or notes name them), dry run by default; real recordings are never pruned by it.
7. [R1X] **People screened before footage is used.** Real recordings start unscreened; the screening tool marks them
   (clear stretches, or people found); anything that exports frames from a recording refuses an unscreened or flagged
   one (the standing rule: no children in any visual data).
8. [T9e] **Integrity.** A verify command re-hashes every copy against the catalog and reports any difference or
   missing copy.

## Decisions

1. [u6u] **A SQLite catalog beside the data, real recordings moved into the project's data store** (3 Oct 2026).
   DimOS writes each run to its checkout's `recordings/<run-id>/memory.db` (its location is fixed by the checkout); a
   real recording is moved (same filesystem: a rename) to `data/recordings/real/<id>/` when cataloged, so the
   checkout's folder holds only sim runs and a DimOS reset can't touch real data. Sim recordings stay where the night
   pipeline finds them. The catalog is `data/recordings/catalog.sqlite` (gitignored with the rest of `data/`):
   SQLite, as DimOS's own recordings are, one file, queryable. Considered: a JSON file per recording (no queries), a
   directory naming scheme alone (no tags or copies).
2. [X7i] **The home server as the second copy, by rsync over the house network**, the laptop holding
   the data until the server is online. The R720 (256 GB RAM, no GPU) takes a
   verified copy of every real recording; DimOS's own content hashing and lz4 are the model (SHA-256 per file,
   compression only for transfer, never for the copy at rest, so a recording opens directly from either copy). The
   destination comes from `real/.env`; until it's set, sync reports what is waiting.
3. [YP8] **Sim recordings are disposable, real ones are not** (3 Oct 2026). 449 sim recordings fill 257 GB of a disk
   77% full; they can be re-recorded, so they are pruned when nothing refers to them. Real recordings are pruned only
   by hand, after the server holds them.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [xOp] (one catalog) | `tests/test_recordings.py`: cataloging a recording fills every field from the file and the repo | Not yet written |
| [5zg] (find and use) | the same: list/filter/show/tag/note/path by id | Not yet written |
| [fya] (never leaves) | `dimos data upload` fails in the checkout; sync refuses a destination outside the house | Not yet written |
| [6xc] (a session) | the session command refuses under the free-space floor and on a failed isolation check; a sim run is cataloged at its end | Not yet written |
| [Zw9] (two copies) | sync to a local stand-in for the server verifies the copy; removal refused without a verified copy | Not yet written |
| [B2O] (room) | prune lists only unreferenced sim recordings; a referenced or real one is never listed | Not yet written |
| [R1X] (screened) | `tests/test_recordings.py`: an unscreened or flagged real recording refused by `require_screened` and by `hk-data path` (the way tools reach frames), unless asked for as numbers only; the morning log's crops ([hKP]) and any training export must call it too | Partial: 2 passing (3 Oct 2026); no export tool yet calls it |
| [T9e] (integrity) | verify reports a changed byte and a missing copy | Not yet written |
