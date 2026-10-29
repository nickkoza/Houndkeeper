# What he thinks, live

**Type:** Capability spec · **Status:** Draft · **Source:** a robot's reasons made visible, for demos and debugging · **ID:** [RNC]

## Purpose

Watch Barrymore think while he works: on the phone, over the camera's view and on the map, what he believes is on the
floor and how sure he is, where he is going next and why, what he has ruled out, and what he is doing with the arm. It
turns a robot wandering a room into one whose reasons are visible, which is the most persuasive demo there is and the
fastest way to see why a night went wrong.

## Acceptance criteria

1. [ULM] **A "mind" stream.** The night's modules publish, on one channel, a small summary of their state a few times a
   second: the current stage (sweep, look closer, pick, put away, dock), the goal and the reason for it, the candidate
   finds with their confidence and status (new, looked at, ruled out, picked), and the arm's state. Published from the
   modules' own state, no extra computation in the night's loop beyond the summary.
2. [Ttd] **On the map.** A Cockpit page (an extension, [Eir]) draws the floor map with the route walked, the planned
   route, the candidates coloured by status and confidence, and the goal with its reason in words.
3. [D1U] **Over the camera.** The candidates the camera can see are outlined on the video with their confidence, and
   the stage and reason are a caption.
4. [B4u] **Replays too.** The same page plays a recorded night's mind stream (DimOS's replay), so a night can be
   watched afterwards at speed, and a demo clip can be cut from it.
5. [ls3] **Read only, private.** The page sends nothing to the robot; it is served like the rest of the cockpit
   ([rhP]); the camera never leaves the house.

## Non-functional requirements

- The mind message is at most 8 KiB and comes at most three times a second (24 KiB/s at worst over the tailnet),
  however long the night and however many candidates it holds.

## Decisions

1. [BxF] **One summary channel, not every internal stream** (proposed, 3 Oct 2026). The phone over the tailnet is the
   tight link; a few kilobytes a few times a second is cheap, and one schema is one thing to version (json.v1, like
   the procedures channel). The full streams stay in the recording for the engineering.
2. [1XG] **Reasons as words chosen by the module that decided** (proposed, 3 Oct 2026). The navigator says why it chose
   a goal ("the far corner: not yet seen"); the page only shows it. Keeps the page dumb and the reason honest.
3. [Fho] **The summary travels as JSON text in a DimOS String.** Forced: DimOS's `--record` skips any stream whose type
   is not a DimOS message (no `lcm_encode`), so a dict would vanish from the recording and [B4u] would have nothing to
   replay (observed 3 Oct 2026, the fork's `dimos/memory/tap.py`, branch feat/cockpit-extensions). The tidy module
   publishes the text; a small module on the cockpit side turns it into the page's json.v1 channel, the same path live
   and replayed.
4. [Izb] **Outlines are projected on the robot, with the camera pose the floor judgement used** (3 Oct 2026). The
   summary carries each visible candidate's outline in image fractions, projected through the lens's own fisheye
   model from the pose the confirmer placed that frame with; the page only draws them ([1XG]: the page stays dumb).
   Outlines show while frames are being judged (the sweep); during a stop the summary carries none.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [ULM] (a mind stream) | `tests/test_mind_view.py`: the summary from synthetic night state (its shape, each find once with its stop's status, ruled out as the tidy module clears it, every reason a string the state held), the tidy module's hook and reasons, a huge night within 8 KiB, a usual one under 4 KiB built in under 20 ms, the channel at 3 Hz, the feed passing only mind messages | Partial: 16 passing (3 Oct 2026); not yet published from a sim night (`HK_MIND=1`) |
| [Ttd] (on the map) | `tests/js/hk_mind.test.mjs` (node, the cockpit's own React, run by `tests/test_mind_view.py`): from the recorded message `tests/fixtures/mind_message.json` (checked against the summary), the map scene puts each candidate where it lies in its status colour with its confidence as opacity, the walked and planned routes, the planner's path, the goal and its reason in words, and the page renders it all. Not in the fork's `extensions.test.tsx` harness: a houndkeeper page in the fork's tests would break [CSu] | Partial: 6 passing in node and the fixture check (3 Oct 2026); the canvases' painting and the costmap's inflate need a browser: not yet seen on the phone or the sim |
| [D1U] (over the camera) | `tests/test_mind_view.py`: floor points projected through the head camera's fisheye agree with OpenCV's within 0.5 px; a projected centre taken back through the lens lands within 1 cm of its floor point; out of view, ruled out or stale, no outline. `tests/js/hk_mind.test.mjs`: the outlines and confidence drawn where the message puts them | Partial: passing (3 Oct 2026); outlines only while the sweep judges frames ([Izb]); not yet seen over a video |
| [B4u] (replays) | sim: a night run with `HK_MIND=1` and `--record` keeps `mind_json` ([Fho]); `dimos --replay-db <run>/memory.db --viewer none run replay houndkeeper.phone-cockpit --local-relay --extensions-dir <houndkeeper>/cockpit/extensions` (with `--speed 4` for a quick watch) plays it through `MindFeed` to the Mind page; the page must show the night's stages, goals and finds in order (`src/houndkeeper/phone_cockpit.py`) | Planned |
| [ls3] (read only) | `tests/test_mind_view.py`: every channel the page binds is rx with no publish; `tests/js/hk_mind.test.mjs`: the page given a session and a teleop never touches them, and its code has no publish | Passing (3 Oct 2026) |
