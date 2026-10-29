# The project on a phone

**Type:** Capability spec · **Status:** Draft · **Source:** principle [Yj2] · **ID:** [C3p]

## Purpose

The project is run from a phone ([Yj2]): reviewing the previous night's report, adjusting things, scripting new
illustrations and videos, which calls for an app to manage and display some of it. One private app, at phone size, for
what the user reads, decides and directs; conversation with Claude stays the channel for anything free-form.

## Acceptance criteria

1. [RzM] **Last night, at a glance.** The morning's page shows each night that ran: the morning log ([uoW]), its
   numbers (found, binned, false stops, battery, time, the speed it ran at), what failed and why, and the newest research
   write-ups, each opening with its answer; every figure links to its source. Readable without zooming.
2. [eCN] **What is running and what is next.** The queue of runs (sims, experiments, agents' long jobs) with state,
   progress and the machine's load; the user can start a queued run, hold one, or stop one, each a confirmed action.
3. [08U] **Decisions in one place.** Everything waiting on a decision (a new dependency, a hold-out go, a frontier spend,
   a change of direction: build-or-depend [eK3]; principles [eNx], [zGo]) listed with the evidence and a yes / no / note,
   answered from the phone; each answer recorded with the date, and the work it unblocks resumes.
4. [k7c] **Tuning from the phone.** The parameters that matter (params/robot.yaml, the stored calibration, the detector's
   and confirmer's thresholds) shown with their source and history; a change is proposed, shown as a diff with what
   it would affect (the runs to repeat, [7Xn]), and applied only on confirm, then recorded like a calibration write
   ([vRJ]'s rule).
5. [QXu] **Video and illustrations: not in this app** (withdrawn: video production does not belong in the cockpit and
   becomes its own app). See decision [kuU].
6. [Dlx] **Private, and every action recorded.** Reached over the tailnet with the cockpit's login ([rhP]), never a public
   page; anything that moves the robot, spends quota or changes what runs asks for confirmation, and every action is
   logged (when, what, from where).
7. [eTc] **Works with nothing running.** The project pages work when no robot or sim is connected: the laptop keeps a
   small project service up, so the morning's review never depends on last night's run still being live.

## Decisions

1. [ehV] **One app: the phone cockpit grown into it, with a project participant** (4 Oct 2026; verified the same day).
   The phone cockpit ([Swt]) already has the private connection, the login, the phone layout and the extension
   mechanism ([Eir]); project pages are more extensions, so the user opens one bookmark. Build-or-depend ([eM8]): reuse
   before a second app. Considered: a separate web app (a second login, layout and private connection to keep); Claude's
   own surfaces only (free-form, not glanceable).
   - Forced (DimOS fork at a7492431f with our local changes, observed 4 Oct 2026): the relay alone serves the Cockpit
     and the extensions, but the Cockpit shows only "Waiting for a robot to register...": a page's layout comes from a
     registered participant's manifest, never from the relay.
   - So the project service ([eTc], `houndkeeper.project`) registers with the relay like a robot, under the robot id
     `project`, with a manifest of only the project's pages and channels; the robot registers beside it as itself.
     With both up, the Cockpit's own robot picker switches between them. No DimOS change: a participant is any
     blueprint with a cockpit, and nothing in DimOS requires it to be a robot (fork rule 6 needs no extension point
     here).
2. [J2U] **Conversation stays the free-form channel** (proposed, 4 Oct 2026). Free-form asks, explanations and new
   directions go through Claude (as now, from the phone); the app is for what is better seen than said: reports, queues,
   decisions waiting, parameters, shots and renders.
3. [kuU] **Video production is its own app, hosted and shared with collaborators**: video production does not belong in
   the cockpit, and clips, scripts, content and ideas are shared with a video producer who may do the edit. Scripting
   illustrations and videos, the story's acts and beats, shot sheets and camera blocking ([tDW]), clips and renders for
   review and edit: with the films (the separate `houndkeeper-demos` repository), not the robot's cockpit. Phone-first
   ([Yj2]). Unlike the cockpit it is hosted, reachable by the user and the people they invite (a video producer who may
   edit), and nothing else: signed in, never public. What goes in is only what was made to be shared:
   - sim renders, illustrations and the storybook (no house data): freely;
   - scripts, beats, shot lists and ideas written for the app, never the raw story notes (they hold household details);
   - real robot footage only when deliberately chosen ([zGo]), never a child ([JHu]), people-screened first, never raw
     recordings or training data.
   Every share and every collaborator's access recorded. The host is a new service (build-or-depend [eM8]), leaning to
   Cloudflare: the pages on Cloudflare Pages, clips and renders in R2, sign-in by Cloudflare Access for the user and
   invited addresses; its
   tunnel is already on the laptop. Considered: a small site on a personal domain, a shared-folder service the
   collaborator already uses. Specced on its own when started; the account set up then.
4. [0Us] **Bound as the relay is, and no further** (4 Oct 2026). The project service opens no port of its own: it reaches
   the phone only through the relay, as a client of it, and DimOS's own transport between its processes listens on
   loopback (zenoh `tcp/127.0.0.1`, observed 4 Oct 2026). The relay binds loopback for tests and the laptop's Tailscale
   address in production, with the certificate and auth file ([rhP]); the launcher refuses any other address before
   anything starts, a house address and every-address included. The service's relay key is its own (`project` in the
   auth file), read into its environment at start.
5. [jnG] **A night is listed only when its house is known and not held out** (4 Oct 2026). Hold-out houses are never
   opened ([ejh]; their registry is `sim/house/make_house.py` HOLDOUT). A night names its house in its night.json, the
   first file read; a night that names none (runs from before night.json) or names a hold-out is passed over with
   nothing else of it opened. Hazard: a run directory's other files (report, score, morning log) carry no house, so
   listing by them would open hold-out nights unseen.
6. [WkQ] **The page's message is bounded** (4 Oct 2026). One compact JSON message of at most 256 KiB, the morning log
   without its pictures (the Morning page has them, [Ndt]). Over the bound, in this order: findings' whole texts from the
   oldest, older nights' morning logs, older nights, then excerpts shortened; what was left off is named on the page.
   Measured on 4 Oct 2026: three nights and four findings, 50 KB.
7. [E2e] **The message is published again every 10 s, unchanged unless a file changed** (4 Oct 2026). Hard-won: DimOS's
   bridge replays a channel's last message only when the channel goes from no viewer to one; a page opened while
   another viewer (another tab, or a closed page whose session has not yet idled out, about 30 s) is subscribed got
   nothing until the next publish. Observed on the fork at a7492431f, 4 Oct 2026. The robot's Morning page shares the
   gap. A general fix (the relay keeping a latest channel's last frame for a late viewer) would be a DimOS feature
   under rule 6; not made in this slice.

## Open questions

1. Notifications: when a night finishes or a decision is waiting, how the phone is told (Claude's push, a self-hosted
   push service, the tailnet), privately.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [RzM] (last night) | `tests/test_project.py` (the message's shape from a night and findings, what failed, the findings' answers, the bound [WkQ], no hold-out opened [jnG]); `tests/js/hk_last_night.test.mjs` (the page from `tests/fixtures/last_night_message.json`: nights, numbers with sources, failures, morning log, findings and their whole text, nothing sent, nothing moving); the browser smoke in `tests/test_project.py` (`tests/browser/last_night_390.mjs`: 390 px, nothing wider than the screen, no text under 13 px, nothing animated, screenshots) | Passing (4 Oct 2026) |
| [eCN] (running and next) | the queue shown; start, hold and stop each confirmed and logged | Not yet written |
| [08U] (decisions) | a waiting decision answered from the page is recorded and unblocks its work | Not yet written |
| [k7c] (tuning) | a parameter change shown as a diff, applied only on confirm, recorded with its source | Not yet written |
| [QXu] (video) | moved to its own app ([kuU]) | Withdrawn |
| [Dlx] (private, logged) | `tests/test_project.py`: the launcher refuses every address but loopback and Tailscale's, and off loopback runs the relay only with its certificate and auth file [0Us]; the Last night page's channel is received only, and the only channels a page sends on are the scoring pages' (a confirmed save of the marks, a judgement, sim-to-real [oWO]); `tests/test_real_night_scoring.py`: each save and judgement logged with its time | Partial: the scoring pages' actions are logged; the marks' save asks for confirmation, a judgement (which moves nothing and runs nothing) does not; actions for [eCN], [08U], [k7c] not yet built |
| [eTc] (nothing running) | `tests/test_project.py`: the service needs no robot stream and publishes its message on its own [E2e]; the browser smoke starts only a loopback relay and the project service, and the page fills | Passing (4 Oct 2026) |
