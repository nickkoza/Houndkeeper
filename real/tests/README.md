# Real-robot and sim checks

## Cockpit deadman: silence stops the robot (phone-cockpit [Mj2])

Drives the sim Go2 from the Cockpit page in a headless Chromium, makes the page fall silent, and reads odometry to
check the robot stopped within 0.6 s.

| File | What |
|---|---|
| `deadman_sim.sh` | runs it all: sim + Cockpit on a local relay (headless), odometry logger, each case, the check, cleanup |
| `cockpit_silence.mjs` | the browser: arms the teleop panel, holds D (turn on the spot) for 3 s, then one silence case |
| `deadman.py` | `log` odometry from Zenoh to CSV; `check` the stop time of each case |

Silence cases (`CASES`, all four by default):

- `killed`: SIGKILL the browser (no key-up, no goodbye)
- `hidden`: the page reports hidden and fires `visibilitychange` (phone locked, app switched): the panel disarms
- `frozen`: CDP `Debugger.pause`: the page's JS halts with its connection still open (phone asleep, page hung,
  Wi-Fi gone quiet), so only the bridge's 300 ms deadman can stop the robot. (`Page.setWebLifecycleState frozen`
  does nothing to a visible headless page: its timers keep firing and the robot keeps driving.)
- `closed`: the tab is closed

A case passes when the robot was turning faster than 20 deg/s over the 1.5 s before the silence, and from a moment
no more than 0.6 s after it (`stop_s`), every 0.2 s of odometry up to 2 s after the silence turns under 3 deg/s the
way it was driven, and any settling stays under 10 deg/s back the other way and 0.05 m/s. (Halted mid-turn, the
gait settles back 4-7 deg/s for up to a second; `still_s` reports when it is fully still: under 3 deg/s either way
and 0.03 m/s.) Odometry is stamped when it arrives, so its own delay counts against the stop.

### Install (once)

```sh
cd real/tests
npm install                      # Playwright (node_modules/ is gitignored)
npx playwright install chromium  # its browser, into ~/.cache/ms-playwright
```

Also needed: the DimOS checkout beside this repo with its `.venv` (or `DIMOS=/path/to/dimos`), and `deno` on PATH
(`~/.local/bin` is added) for the relay.

### Run

```sh
real/tests/deadman_sim.sh [OUT_DIR]            # default OUT_DIR /tmp/hk_deadman_<time>
CASES="killed frozen" real/tests/deadman_sim.sh
```

About 2 minutes. One MuJoCo sim at a time: it refuses to start while another is running. Long or unattended:
`setsid nohup real/tests/deadman_sim.sh /tmp/dm1 >/tmp/dm1.out 2>&1 &`, then read `/tmp/dm1/result.txt`.
Exit 0 only if every case passed; 1 on a failed case; 3 (INVALID) when odometry ran under 11 Hz around a case:
the sim publishes 15 Hz in real time, and a loaded machine (load above the core count) runs it slower than real time,
so the robot's stop, timed in sim, would look slow against the relay's wall-clock deadman. Rerun on a quieter machine.
`OUT_DIR` keeps `sim.log`, `odom.csv`, `events.jsonl`, `browser.log` and
`result.txt` (one JSON line per case: `stop_s`, the turn rate before, the largest turn and speed after).
