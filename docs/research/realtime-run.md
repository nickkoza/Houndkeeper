# Real-time sim run: results

> **Why this is here:** the sim ran 3.3x faster than real time while DimOS ran on the wall clock, so the camera saw
> about 3x fewer frames a metre and every number looked better than it would on the robot. A plain account of a
> clock mismatch that flatters a simulation, and what it cost to fix (sim-house-nights [drP] holds the result).

Run overnight 22–23 Sep 2026. Raw output is in `sim/runs/nights_s8/final.txt`, with the log in `sim/runs/nights_s8/run_notes.txt`.
All numbers are from the simulator, not the real house. Numbers marked (est.) are estimates.

## Why we ran it
The sim had been running about 3.3× faster than real time. Each physics loop advanced 35 ms of sim time but waited only for 5 ms of real time, and DimOS runs on the real clock. So the robot moved about 3× faster than DimOS thought, and the camera saw about 3× fewer frames per metre.
A speed setting (`DIMOS_SIM_SPEED`) was added to DimOS's sim loop. This run used real time.

## What changed compared with the earlier run
Three things changed at once, so no single difference can be pinned on sim speed alone:
1. **Sim speed:** real time (measured 0.91–0.93×), against about 3.3×.
2. **New timeline (seed 8):** a different floor, and 12 nights instead of 8.
3. **Moving furniture:** six dining chairs, a stool and a kid's chair, each tucked in, nudged a bit, or moved a lot each night. None are pick-up targets.

Unchanged: detector v2 (frozen), the navigation fixes, the sweep route and the scoring. Tuning nights 2–4, held-out nights 5–12, scored once.

## Results (held-out nights)

| | Earlier: ~3.3×, seed 7, nights 5–8 | **Real time: seed 8 with chairs, nights 5–12** |
|---|---|---|
| Items scored | 87 | 156 |
| Found | 72% | **58%** (91/156) |
| Easy / medium / hard | 86 / 74 / 54% | 66 / 54 / 42% |
| False alarms per night | ~43 | **~18** |
| Look-closer: real finds kept | 95% | 84% |
| Look-closer: false alarms removed | 29% | 53% |
| False alarms left after look-closer | ~30 a night | **~8 a night** |
| Items never properly seen | 8 | 24 |
| Flags on chairs, or on the spot a chair left | n/a | 0 of 175 false alarms |

Old detector (v1) on the same real-time nights, for reference: 47% found, ~30 false alarms a night.

## Navigation
- 49/49 waypoints every night; 0–2 failed waypoints a night, with the robot still reaching all 49.
- Stalled 0–1% of the time it was told to move. Night 12 was 6% (39 s), cause not investigated.
- A night took about 24 minutes of real time, against about 8–10 minutes at 3.3×. The robot was told to move for about 500–680 s a night (about 112–170 s at 3.3×).

## Floor model
It got steadily sharper at real time and ended much better than the 3.3× run:

| | After night 1 | After night 12 | 3.3× run, final |
|---|---|---|---|
| Fidelity to true floor | 0.24 | 0.30 | 0.12 |
| Confidently mapped patches | 33% | 49% | 10% |
| Waviness | 1.4 cm | 2.0 cm | 4.0 cm (peaked at 4.3) |
| Coverage | 72% | 76% | 80% |

This fits the idea that real time gives the camera about 3× more frames per metre.

## Reading the results
- **Fewer false alarms is the big win.** About 8 closer looks a night after look-closer, against about 30. That's the battery-budget problem shrinking a lot.
- **The drop in finds is mostly items seen only as tiny specks.** Misses where the item was detected and then dropped went from 8 to 34, mostly specks under 8 px.
  - Likely cause, not verified: some thresholds count lidar hits per cell. They were tuned at 3.3×, and real time gives about 3× more scans.
  - The lidar "stands up" layer tripled in size (~23 → ~80 m²) and probably masks floor next to items.
- **Chairs weren't flagged, but big chair moves were barely tested.** A generator bug meant only 2 big moves in 12 nights: when a big move couldn't find space, the generator re-rolled the move type instead of retrying the position.
- **Decision (user):** we are not re-tuning for sim speed. At this point it would be tuning for the simulator rather than the real robot.
  - The design fix still worth keeping is to normalise hit counts by how many scans saw each cell. The real robot's scan rate won't match either sim speed.
  - Threshold numbers get set on real data.
- **The one-pager is unchanged** (user decision). It still shows the 3.3× numbers.

## State after the run
- The sim is back in fast mode by default. `sweep_night.sh` uses fast unless `DIMOS_SIM_SPEED` is set (1 = real time) and logs the speed used each night.
- The speed setting and its measured-speed logging remain in DimOS as a fork change (now in `sim/dimos/dimos_fork_changes.patch`); unset means the old fast behaviour.
- Follow-ups were listed in the backlog of the day (now `docs/backlog.md`).
