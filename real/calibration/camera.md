# The head camera's picture: noise, exposure, and how it compresses

Turns `go2_air.camera_imperfections` into a matched measurement ([4hm]): the sim's numbers (sensor noise, exposure,
rolling shutter, the camera's own noise reduction, the stream's compression and its dips) are what a sensor does
before the camera's processing; what the robot receives is after it. So the same thing is measured on a real
recording and a sim recording of a like scene, and the sim's numbers are tuned until the two agree.

## 1. A still minute (effective noise)

1. Evening, the house's usual lights, the robot standing still looking along the floor and a wall (the demo room's
   real counterpart: carpet, tile, a plain wall, a dark and a light patch in view).
2. Record two minutes, the robot not moving.
3. `measure.py noise <real memory.db> --sim <sim memory.db>`: per brightness level, the frame-to-frame noise in both
   (the sim night's still spans: it stands at the dock before it moves), fitted as variance = shot x level + read^2,
   and the ratio. Scale the sim's `shot_noise` and `read_noise` by the variance ratio it prints, re-render the sim
   night, and repeat until the ratio is within 10% of 1.

## 2. Motion blur and the rolling shutter (exposure, readout)

Turning on the spot (timing.md 2) at night: a vertical edge (a door frame) leans in the picture by the rolling
shutter (readout x turn rate) and blurs by the exposure. Measure both in a few frames (the lean in pixels over the
edge's height; the blur's width) and compare with the same turn in the sim: the sim's `rolling_shutter_s` and
`exposure_max_s` set them. By day, the blur sets `exposure_day_s`.

## 3. Compression on the darkest floor, and its dips

The sim stands JPEG in for the stream's H.264 (`jpeg_quality` 75, dips to `jpeg_low` 30 for `dip_s` seconds
`dip_per_min` times a minute). How hard the real stream compresses matters most where the floor is dark and plain: a
sensitivity study found q30 on every frame costs a third more false alarms, most of them on the dark tile, while q50
and q75 cost nothing (docs/research/sensitivity-2026-10-13.md; day one's band, README.md, [GWP]). Compression on a dark
floor shows as flatness (the fine texture and grain inside each 8x8 block rounded away) and as blocks (H.264's
deblocking filter hides most of them, until the bitrate runs short). `measure.py compression` reads the floor's dark
tiles (64 pixels, in the picture's lower half, mean grey 0.03-0.25 on both sides alike) and prints:

- **flat**: the share of each 8x8 block's frequencies (all but its mean) under one grey level; about 0.2-0.4
  uncompressed, rising toward 1 as the quality falls (and its per-frame 90th percentile: the dips);
- **ac**: their mean size (grey levels: the texture and grain left);
- **blockiness**: the step across the block edges less the step inside (grey levels).

1. Run the still minute (1) first, or in the same session: grain fills what compression flattens, so a noisier real
   camera reads kinder. Tune the noise first if it is off.
2. Evening, the house's usual lights. Stand the robot looking along the darkest floor (the dark tile, a dark rug),
   no people in view, and record two minutes still; then walk it slowly across the same floor for two minutes,
   recording, so the encoder has motion to spend its bits on.
3. Note the stream's bitrate, if the WebRTC statistics show it, beside the recording.
4. `measure.py compression <real memory.db> --sim <a sim night over a like dark floor> --sheet real/local/compression.png`
   (the demo room's twin, house 95, or house 96's dark tile: night 1 in `sim/house/runs/nights_s96/seed_recordings.txt`).
   It reads up to 60 frames standing and 60 walking from each, prints the three measures for the real frames, the
   sim's and the sim's re-encoded at q50 and q30 (the study's own `jpeg50` and `jpeg30` variants), and writes a
   side-by-side picture for each (`_standing`, `_walking`): a row of dark tiles each, stretched to their range and
   enlarged, phone-sized. The floor only, and it stays in the house.
5. **Judge by eye, the numbers beside it.** Pass: the real rows look no flatter or blockier than the sim's at q50.
   Fails: as flat or blocky as the sim's at q30, or worse. Between is untested: outside. There is no automatic
   verdict, on purpose: on the sim's own dark floor (14 Oct 2026, house 96 night 1) re-encoding at q50 or q30 moved
   these measures by a few per cent (flat 0.944 to 0.938 and 0.935 walking, blockiness 1.52 to 1.49 and 1.61), less
   than two houses' floors at the same quality differ (house 70's walking blockiness 1.93), so a number cannot place
   a real floor between q50 and q30 on its own. What it can show is a real stream far past the sim's q30: H.264 short
   of bits on a dark floor goes visibly blocky or flat, well beyond these few per cent.
6. Outside or fails: lower the sim's `jpeg_quality` (or, if only the walking or the 90th percentile is off, make the
   dips more often or longer: `dip_per_min`, `dip_s`) until a sim night of the dark floor looks the same, then let the
   detection bench and floor map error re-run ([7Xn]). On the robot, the stream's resolution and bitrate and the
   Wi-Fi where it works are what can move it.

The measures are checked on a dark floor made up and passed through JPEG at known qualities (q95, 75, 50 and 30 told
apart; the picture has its four rows: `tests/test_measure.py`).
