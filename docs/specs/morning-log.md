# The butler's morning log

**Type:** Capability spec · **Status:** Draft · **Source:** a night's work reported as a story · **ID:** [uoW]

## Purpose

Each morning Barrymore reports on his night the way a butler would: what he found and where, what he picked up and
put away, what he couldn't manage and why, how far he walked and what it cost the battery, and anything about the house
that changed. Written from the night's own outputs, with a picture of each find, so the household sees in a minute
what a night of work came to, and the engineering sees the same night's figures one tap down. It tells the night as a
story (the rule for anything shown: never boring), and every sentence in it traces to a number the night recorded.

## Acceptance criteria

1. [9X9] **From the night's outputs only.** A command turns a finished night (the night pipeline's run directory: finds,
   route, picks, battery, failures) into the log; no sim truth is read for the household's part, so the same command
   works on a real night. A sim night may add a "scored against truth" section, marked as such.
2. [W0w] **In his voice, from facts.** The prose is Barrymore's (a Victorian butler: courteous, dry, brief), built from
   templates over the night's facts; each sentence's figure is the recorded one. No sentence is invented that the
   night's data doesn't support; an empty night says so.
3. [hKP] **A picture for each find.** Each find shows a crop of the floor picture around it, the place on a small map of
   the house, and the time. For a real night, crops come only from screened recordings ([R1X]); a crop that shows a
   person is never shown (the people screen runs on every crop; [JHu], [zGo]).
4. [Ndt] **Read on the phone.** The log is a page in the phone cockpit ([Swt], an extension) and a file beside the night;
   it is served only inside the house or over the tailnet ([rhP]), never uploaded.
5. [6wJ] **The night's engineering one tap down.** Under the prose: the night's figures (coverage, finds by kind,
   false alarms, distance, battery used, the slowest stage), each linked to where it came from.

## Decisions

1. [ZNG] **Templates, not a language model, for the prose** (proposed, 3 Oct 2026). Every sentence must trace to a
   figure ([W0w]) and nothing about the house may leave it; templates make both automatic. A local model may later
   vary the wording, checked against the same facts.
2. [t0a] **One log per night, kept with the night** (proposed, 3 Oct 2026). A night's log is a file in its run
   directory (sim) or beside its recording in `data/` (real), so the catalog finds it with the night ([HqQ]).

## Tests

| Criterion | Test | Status |
|---|---|---|
| [9X9] (from outputs only) | `tests/test_morning_log.py`: the log built from a sim night's run directory with its truth files absent (and traj.csv's true columns scrambled); the same facts, prose and pictures as with them; a truth file refused; the scored section (morning_score.py, beside the log) only on a sim night; the tidy's night.json (seed, mode, item seed, round, night_tidy folder, start and end, route.json copied in) written and read, holding only its own keys, and one with any other key refused | Passing: 4 tests (3 Oct 2026); built on two tuning-house nights (houses 70 and 71, round 16); night.json from a test night only: the tidy (tidy_run.sh) has not yet written one in a run |
| [W0w] (in his voice, from facts) | `tests/test_morning_log.py`: each figure in the prose matches the night's record, and no digit stands outside a fact; an empty night's log says nothing was found; a directory with no report is refused | Passing: 2 tests (3 Oct 2026) |
| [hKP] (a picture for each find) | `tests/test_morning_log.py`: one crop per find (JPEG), inside its picture's bounds, the same share of its look at 640 x 360 and 1280 x 720 ([7cU]); a real night's crops refused while unscreened, flagged or uncatalogued; the people screen called on every crop and a flagged crop dropped | Partial: 2 passing (3 Oct 2026); the crop is from his close look (else his standing look), because a confirm-mode night keeps no floor picture of itself; the real people screen (`screen_people.people_in`) not yet run on a crop |
| [Ndt] (on the phone) | `tests/js/hk_morning_log.test.mjs`: the Morning page renders the prose, a card with its JPEG crop for each find, the map and the figures with their sources from a recorded message, sends nothing, and nothing on it moves; `tests/test_morning_log.py`: the file beside the night fetches nothing and runs no script; the phone's message is the latest night's, within PHONE_MAX_BYTES (384 KiB), with no crops from an unscreened real night (recordings.require_screened), and the fixture is its shape; `tests/test_cockpit_data.py`: the feed publishes the latest log and again only for a newer one; the page registered, its channel received only | Partial: 5 node and 7 Python passing (3 Oct 2026); not yet seen on a phone (needs a sim night), and served only as the cockpit is ([rhP] not tested here) |
| [6wJ] (engineering below) | `tests/test_morning_log.py`: each figure links to its source file, the night's own | Passing: 1 test (3 Oct 2026) |
