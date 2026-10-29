# Detection baselines: the fixed frames and the frontier model's answers

Why this is here: the bench code beside it reads these files, and the answers are the only record of a paid
frontier-model run. Re-running it spends the project's subscription quota, so they are kept rather than regenerated.

- `frames_manifest.json`: the fixed 270-frame set from the six recorded nights (detection-baselines [8VL]).
  `floor_diag.subset_view()` reads it.
- `frontier_set.json`: the 70 of those frames sent to the frontier model ([DZr]); read with the manifest.
- `prompt.txt`, `answers/batch_00-03.json`: the question the frontier model was asked and its raw answers
  (4 Oct 2026, [6hv], [L9V]). `frontier.py load sim/house/baselines/frontier_run <out.json>` turns them back into a detections file.

Paths inside the JSON name where the frames were rendered on the day; the frames themselves are not kept.
