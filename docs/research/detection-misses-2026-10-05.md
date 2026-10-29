# Why OWLv2 missed what it missed: detector or prompt? (sim, 5 Oct 2026)

> **Why this is here:** a short, clean diagnosis of a question every open-vocabulary detector raises: when it misses,
> is it the model or the words it was given? Here it was mostly the words (detection-baselines [XEi], [En1]).

The question: when the detector fails to detect a toy, is that a failure of the detector or of the prompt? This
diagnoses every toy that OWLv2 base and OWLv2 large missed as detectors in
the recognizers round of 4 Oct (detection-baselines [80w]'s Tests row), on its 270 frames and 48 toys, with the 12 prompts of [XEi] ([En1]'s
recognizers). Sim only; everything ran on the laptop; nothing was sent to any hosted service. This is a **diagnosis,
not a result**. It reads the truth, and the generic-prompt thresholds were matched using all six nights' truth.

## Answer

**Mostly the prompt, and a little the threshold. Not the detector.** No miss is an in-view toy of ordinary size that
OWLv2 cannot see:

- **The bowl and the dinosaurs** sit outside the 12 prompts' words. They score 0.06 to 0.19 with them, and 0.35 to
  0.60 when asked about with "small object" or "object on the floor". This is a prompt failure.
- **The lion, the other dinosaur and the lacing sheep** are seen, but just under the line: 0.09 to 0.19, against
  thresholds of 0.15 and 0.2.
- **The 1×1 Lego brick** is 2 to 6 pixels across. It is too small for base at this resolution. Large, with its finer
  patches, does pick it up in one house.
- **House 96's card game** never comes within the 2.5 m range in which the arm places detections. Large's miss of it
  is a matter of where the camera went, not of recognition. Base's "find" of it was a coincidence: its box was on the
  lacing sheep, which lies within reach of both toys.

Adding one generic prompt, "small object", to the 12 finds 3 more toys for each model on these nights. It costs no
extra false positives and loses no toy that was found before. That figure is fitted to these nights (below).

## Counts

| Class | OWLv2 base (thr 0.2): 7 misses | OWLv2 large (thr 0.15): 6 misses |
|---|---|---|
| threshold (best score on the toy at least half the threshold) | **2**: s76 lion (0.149), s76 dinosaur dino_5 (0.191) | **3**: s71 dinosaur dino_5 (0.126), s76 bowl (0.090), s96 lacing sheep (0.088) |
| prompt, generic (12 + a generic prompt finds it at the original false-positive level) | **3**: s71 bowl, s76 bowl, s71 dinosaur | **1**: s71 bowl |
| prompt, specific (only its own name finds it) | 0 | 0 |
| resolution/size (projected long side under 20 px) | **2**: s71 Lego 1×1 (6 px), s76 Lego 1×1 (3 px) | **1**: s76 Lego 1×1 (3 px) |
| detector (nothing finds it, and it is large enough) | 0 | 0 |
| not in view (no frame sees it within the arm's 2.5 m placement range; a class added below) | 0 | **1**: s96 card game |

The classes overlap. The first that holds is given, in the order of the table. Of the threshold class:

- For base, the lion is also found by its own name ("lion", 0.277). The s76 dinosaur is also found by "small object"
  at the matched threshold.
- For large, the s76 bowl and the s96 sheep are also found by "small object". The s71 dinosaur is also found by
  "object on the floor".

So the "threshold" toys are prompt-fixable too, and for both models every miss but the Lego bricks and the
out-of-range card game is prompt-fixable or within a hair of the line.

## What ran

| | |
|---|---|
| Harness | houndkeeper `sim/house/baselines/misses.py` (new; `run`, `diagnose`), on base commit ae84241; the placement and scorer of `place.py`, `score.py`, `table.py` unchanged ([o0E], [oR6]) |
| DimOS | fork base a7492431f with its local working-tree changes; the wrapper `dimos/perception/detection/detectors/owlv2.py` (sha256 `d618af38…bc21c`), `Owlv2Detector.query_score_rows` |
| Models | `google/owlv2-base-patch16-ensemble`, HF snapshot `cfd3195ba4ea9592eec887ded089f4c08eff231d`; `google/owlv2-large-patch14-ensemble`, HF snapshot `95e26936e865f87db1742128404b3c035d47d89d`; both float32, from the local Hugging Face cache, `HF_HUB_OFFLINE=1` |
| Software | torch 2.7.1+cu128, transformers 4.53.3; one NVIDIA GeForce RTX 2070 (8 GB), one GPU job at a time; Python 3.12 in the DimOS venv |
| Frames | the first run's 270 frames (45 a night) of `detection-baselines-2026-10-04/frames_manifest.json` (identical to the scratch `frames/manifest.json`, sha256 `4b4a989c…e3d1`); nights s70_t1, s71_t2, s73_t1, s76_t2, s77_t1, s96_t1 ([8VL]); no hold-out (78, 80, 81, 100-119) opened |
| Time | base 495 ms a frame, large 1864 ms a frame (median, 30 queries); load average under 1 |

**One pass, every query.** `run` asks each model all 30 queries at once:

- the 12 prompts;
- the 5 generic probes: "object on the floor", "small object", "clutter", "item", "thing on the floor";
- 13 oracle names: "toy airplane", "cart", "action figure", "lacing sheep", "sheep", "box of cards", "toy brick",
  "mug", "ballet flat", "lego brick", "dinosaur", "lion", "bowl", plus "card game" and "shoe", which are already
  among the 12.

`run` keeps every box any query scores at 0.01 or more, with its whole score row. In OWLv2, one query's score does
not depend on the other queries asked with it. Post-processing takes the max over the asked queries per box (no
NMS). So any prompt set can be replayed exactly from these rows.

**Reproduction check.** Replaying the 12 prompts gives the published arms exactly: base 41/48 at 3 false positives
over six nights, and large 42/48 at 4, the same toys found (`misses.json`, `reproduction`).

**Where each toy is (truth, analysis only).** Each toy's surface is the sim's own mesh at its recorded resting pose
(`truth.json`), placed at its drop point. A toy the walk moved is also placed where it ended. It is carried into
each frame's odometry and projected through the frame's fisheye calibration and pose. A frame "sees" a toy when its
projected centre is in the image within 2.5 m. Line of sight is not checked; the contact sheets were looked at
instead (below).

### Rules, fixed before any score was seen, and one revision

**The measures, per miss, over the frames that see it:**

1. **Below threshold:** the 12 prompts' best score on the toy.
2. **Generic prompts:** the 12 plus each probe, and plus all five. Each set runs at the lowest threshold (0.05 to
   0.60 in 0.01 steps) whose false positives over all six nights do not exceed the original arm's (base 3, large 4).
   The set's whole result is recorded: the toys it gains and loses.
3. **Own name:** the 12 plus each oracle name, the same way.
4. **Size:** the long side, in pixels, of the toy's largest projected box over its views. This is recorded for all
   48 toys.

**The classes, in order:**

1. placement: a box on the toy at or above threshold, but placed out of reach (none occurred);
2. threshold: best score on the toy at least half the threshold;
3. prompt, generic;
4. prompt, specific;
5. resolution/size: under 20 px, about two base patches once OWLv2 enlarges the frame;
6. detector.

**The revision, made after the first pass.** "Near the toy" first meant any box whose floor point fell within the
toy's reach. That picked up boxes beside the toy: large gave the 3 px Lego brick 0.101 from a box that was not on it.
"On the toy" now means placed within reach **and** overlapping its projected box by IoU 0.10 or more. This moved the
s76 Lego for large from threshold to resolution, and the s71 Lego for base stayed under 0.05. The tables give both
scores.

**The "not in view" class** was added when house 96's card game turned out to have no frame within range.

## Per miss

Scores are the best on the toy over its views (in brackets, the best of any box placed in reach). "Matched" means the
false-positive-matched threshold of rule 2. The full rows, with every set's gains and losses, are in `misses.md` and
`misses.json`.

### OWLv2 base (threshold 0.2; 3 false positives over six nights)

| Miss | Class | Views; best view | 12 prompts | Generic, on the toy | Own name | Lowering the 12 to its score |
|---|---|---|---|---|---|---|
| s71 Lego 1×1 | resolution | 10; 6 px at 0.79 m | 0.026 | object on the floor 0.057, others ≤ 0.031 | lego brick 0.022 | - |
| s71 dinosaur (dino_5) | prompt, generic | 7; see note | 0.062 | **object on the floor 0.429**, thing on the floor 0.239, small object 0.183 | dinosaur 0.018 | 0.061: 26 FP |
| s71 bowl | prompt, generic | 10; 52 px at 1.23 m | 0.067 | **object on the floor 0.605, small object 0.354** | bowl 0.615 | 0.067: 21 FP |
| s76 Lego 1×1 | resolution | 3; 3 px at 1.65 m | 0.000 (0.083 off the toy) | 0.000 | 0.000 | - |
| s76 lion | threshold | 11; 31 px at 1.05 m | 0.149 | object on the floor 0.203, small object 0.188 | **lion 0.277** | 0.149: 6 FP |
| s76 dinosaur (dino_5) | threshold | 10; 51 px at 1.02 m | 0.191 | small object 0.185, object on the floor 0.169 | dinosaur 0.353 | 0.191: 3 FP (free) |
| s76 bowl | prompt, generic | 2; 55 px at 1.16 m | 0.077 | **object on the floor 0.544, small object 0.347** | bowl 0.559 | 0.077: 14 FP |

### OWLv2 large (threshold 0.15; 4 false positives over six nights)

| Miss | Class | Views; best view | 12 prompts | Generic, on the toy | Own name | Lowering the 12 to its score |
|---|---|---|---|---|---|---|
| s71 dinosaur (dino_5) | threshold | 7; see note | 0.126 | **object on the floor 0.508**, thing on the floor 0.258 | dinosaur 0.010 | 0.126: 5 FP |
| s71 bowl | prompt, generic | 10; 52 px at 1.23 m | 0.060 | **object on the floor 0.400, small object 0.248** | bowl 0.647 | 0.059: 25 FP |
| s76 Lego 1×1 | resolution | 3; 3 px at 1.65 m | 0.000 (0.101 off the toy) | 0.000 | 0.000 | - |
| s76 bowl | threshold | 2; 55 px at 1.16 m | 0.090 | **object on the floor 0.497**, thing on the floor 0.293, small object 0.286 | bowl 0.558 | 0.089: 10 FP |
| s96 lacing sheep | threshold | 2; 57 px at 0.66 m | 0.088 | **small object 0.378, object on the floor 0.358** | lacing sheep 0.017, sheep 0.004 | 0.088: 10 FP |
| s96 card game | not in view | 0 within 2.5 m (seen only at 2.7 to 6.0 m) | - | - | - | - |

**Notes on single toys:**

- **The s71 dinosaur** was knocked 0.57 m by the walk. In frame `s71_t2/1790928520.597` it stands upright at its
  drop point, about 31 × 48 px at 1.02 m, and the 12 prompts give it 0.062 there ("stuffed animal"). Later frames show
  its drop point empty. Its "best view" size (109 px) uses its final pose and is not what the camera saw; about 48 px
  is.
- **Large's own name for the s71 dinosaur scores lower** than "object on the floor" (0.010 against 0.508). Seen from
  this angle, the model does not take it for a dinosaur, but it does take it for a thing on the floor.
- **The house 96 card game** never comes within range for either model. Base "found" it with a "ball" box at 0.67 m
  (score 0.28) in frame `s96_t1/1791083633.853`, and that box is on the lacing sheep. The sheep's floor point lies
  within reach of the card game's end position too, so the scorer credited one box to two toys. Base's 41/48 includes
  this one coincidental find. For large, "12 + object on the floor" also "finds" the card game the same way.

### Sizes: found against missed (best view, long side in pixels)

| | Count | Min | Median | Max |
|---|---|---|---|---|
| Found by base | 41 (one is the coincidental card game, 0 px) | 11 | 37 | 257 |
| Missed by base | 7 | 2.7 (Lego) | 51 | 109 (the s71 dinosaur; about 48 as seen) |
| Found by large | 42 | 6 (the s71 Lego, scored 0.19 "ball" on the toy) | 40 | 257 |
| Missed by large | 6 | 0 (card game, out of range) | 53 | 109 |

Apart from the two Lego bricks, every miss is 31 to 57 px across in its best view, at or above the found toys'
median. Size does not explain them. The Lego bricks (0.6 cm² footprint) are 2 to 6 px: under base's 20 px floor
and at the edge of large's. Large's finer patches catch the s71 brick at 6 px.

### The generic prompts at the original false-positive level

| Set | Base: threshold, found (gained / lost) | Large: threshold, found (gained / lost) |
|---|---|---|
| the 12 (re-matched) | 0.19, 42 (+ s76 dinosaur) | 0.14, 42 (none) |
| **12 + "small object"** | **0.19, 44 (+ both bowls, s76 dinosaur; none lost)** | **0.14, 45 (+ both bowls, s96 sheep; none lost)** |
| 12 + "object on the floor" | 0.32, 42 (+ both bowls, s71 dinosaur; − s96 sheep, s96 card game) | 0.35, 39 (+5; − 8, among them both s76 figures and both aeroplanes) |
| 12 + "clutter" / "item" | 0.19, 42 (no gain beyond the re-match) | 0.14, 42 |
| 12 + "thing on the floor" | 0.39, 30 | 0.41, 26 |
| 12 + all five | 0.39, 37 | 0.42, 30 |
| "small object" alone | 0.16, 43 | 0.09, 45 (− s71 Lego) |

"object on the floor" scores the missed toys highest, but it also scores the empty floor. To hold the noise, the
threshold must rise to 0.32 to 0.35, and that loses other toys. "small object" adds what it finds at the old
threshold, with no extra noise on these nights. "clutter" and "thing on the floor" are poor prompts for OWLv2:
alone, they find 3 to 8 toys at the noise level. The oracle names add only their own toy (bowl: both bowls; lion:
the lion), and the dinosaur's name helps only the upright s76 dinosaur.

## Contact sheets

`misses_base.jpg` and `misses_large.jpg` show each miss's three largest views. The green box is the projected truth
box; the red box is the best box on the toy, where it lies in that view. They were looked at before writing this:

- The bowls, the s76 dinosaur and the sheep are plainly visible and unoccluded.
- The lion is visible, partly blurred in two of its views.
- The Lego bricks are specks.
- The s71 dinosaur's later views show its drop point empty (above).
- The projected box sits a few pixels off the toy in some views (pose lag), which the 0.10 IoU allows for.

## A proposed prompt set (fixed here, not yet tested)

The 12 prompts of [XEi], plus three:

```
["toy", "stuffed animal", "ball", "block", "shoe", "cup", "card game", "figure", "toy car", "doll", "sock", "book",
 "small object", "bowl", "toy animal"]
```

- **"small object"** is the generic prompt the diagnosis supports for both models.
- **"bowl"** and **"toy animal"** are everyday kinds a toy-and-household list should name. They were chosen with these
  misses in view, so they are fitted to them. "toy animal" was not scored here at all, so there is no number for it
  to flatter.
- The threshold is chosen as [XEi] says: on one tuning night alone, then applied to the rest.

**This set must be validated on nights not used here.** That means new tuning-house nights: not s70, s71, s73, s76,
s77 or s96, and never the hold-outs 80, 81 or 100-119. On these six nights it is fitted. No found-rate is claimed
for it, and none should be quoted from the gains above.

## Caveats

- **Sample.** Sim only, 6 nights, 13 misses over two models; one Lego brick, one bowl and one dinosaur kind recur.
- **Thresholds fitted on truth.** The matched thresholds use all six nights' truth (a diagnosis, not a tuned arm).
  With 3 or 4 false positives in all, one false positive more or less moves a threshold by a step.
- **Revised rule.** "On the toy" (IoU 0.10 and in reach) replaced the first pass's "in reach"; the reason is above,
  and the tables show both. The 20 px resolution line and the half-threshold line were fixed beforehand and not tuned.
- **Line of sight** is not computed. Occlusion was judged from the contact sheets.
- **The moved toy.** For toys the walk moved, the pose at each frame's time is not known. The s71 dinosaur's size is
  read from its upright view by hand.
- **No tests.** `misses.py` is analysis tooling with no unit tests. Its reproduction of both published arms (same
  found set, same false positives) is the check.

## Reproduce

The harness, `sim/house/baselines/misses.py` (`run`, then `diagnose`), was retired on 28 Oct 2026 once the
diagnosis was concluded; it is in the project's archived history. It scored every box OWLv2 base and large gave on
the fixed frame set (`sim/house/baselines/frontier_run/frames_manifest.json`) against all 30 queries, then classed
each miss against the two arms' detections.

## Files here (`detection-misses-2026-10-05/`)

The raw score files (`scores_owlv2_*.npz`) and `misses.json` are not kept.

- `misses.md`: per miss, its measures and class; each prompt set's matched threshold, found, gains and losses; as tables.
- `misses_base.jpg`, `misses_large.jpg`: the contact sheets.

No frames and no weights.
