# [On0] — Depth at the wrist: what was tried, what it costs, what failed

**Status:** Proposed (research and sim trials; nothing adopted) · **Date:** 2026-09-29 · **Source:** the search for an
arm sensor that sees 3D shape without texture: moving the camera side to side (adaptive perception), a stereo camera of
our own, and the research and the parallax method's failure, recorded · **Serves:** look-closer [xzk], [pYZ], [c1X] ·
**ID:** [On0]

## Context

On the too-big bench (29 Sep) the close look measured parts for wholes and couldn't tell a printed floor from a thing on
it (look-closer [c1X], [xzk]): the camera is asked geometric questions (how big, how tall, where does it end) that a
picture model answers poorly. Depth answers them directly. Constraints from the house: nothing may flash in visible
light ([0Py], safe for photosensitive people), so a projector must be invisible (940 nm; 850 nm can glow
faint red, and most projectors pulse with each exposure, 15-60 Hz); and flat things (a Lego piece, ~1 cm, perhaps
pressed into carpet) must still be found, whatever is done.
Waymark 0010 ([6hI]) holds the lighting trials (infrared multi-flash, then a white light).

## Decisions

1. [x1U] **Plane plus parallax from the arm's own looks: tried, parked** (29 Sep 2026). The idea: warp one close look
   onto another as if everything were the floor; the floor, whatever its print, lines up, and anything standing on it
   doesn't (it is above the plane), so what doesn't line up is the item, whole. Scored against the sim's true pixels
   (an offline replay) on the first 12 trials of the white-light too-big bench (51-93 looks):

   | Floor alignment | Median overlap with the true item (IoU) | Looks at IoU >= 0.7 |
   |---|---|---|
   | the sim's exact camera poses | 0.62 | 33% |
   | the arm's pose with 2 mm and 0.3 deg of error | 0.26 | 0% |
   | from the images alone (SIFT matches, RANSAC homography) | 0.32 | 19% |
   | the arm's pose, then refined on the floor's image (ECC, the item masked out) | 0.23-0.24 | 3-4% |
   | *the detector's own mask, for comparison* | *0.89-0.94* | *69-75%* |

   Why it fails: it needs the views' relative pose to under a pixel, and the arm's joints, flex and backlash give
   several; a big toy fills enough of the frame to pull an image-only alignment off the floor; the ECC refinement
   made it worse, not better, and was not diagnosed (a fault in it is likely). And by geometry it can't find flat
   things: a 1 cm Lego piece at 30 cm across 6 cm of baseline shifts ~2.5 px, the size of the pose error. With
   exactly known poses it works moderately, which is what a rigid stereo pair provides ([A8K]). Parked, not deleted.

2. [GwO] **Depth cameras with their own projector, surveyed** (prices as listed 2026-09-29; manufacturers' figures;
   "closest" is the minimum depth at the lowest resolution, where most reach it):

   | Camera | Price | Emitter | Closest | Notes |
   |---|---|---|---|---|
   | pmd flexx2 (time of flight, Infineon REAL3) | ~$399 (launch; stock to confirm) | 940 nm VCSEL | 10 cm | 72 x 19 x 11 mm, USB3, 224 x 172; soft edges on dark toys; laser class on request |
   | Luxonis OAK-D Pro (active stereo) | $429 (resellers ~$320) | 940 nm dots + flood, Class 1 | ~20 cm | 97 mm, ~91 g, 4-7.5 W, colour camera and on-board AI |
   | Intel RealSense D435 / D435if | $314-354 | 850 nm, PWM, Class 1 | 10.5 cm | 90 x 25 x 25 mm, 75 g; the cheapest documented one that works close |
   | Orbbec Gemini 335 | $264 (out of stock) | 850 nm, pulsed, Class 1 | 10 cm | 90 x 25 x 30 mm, 97 g |
   | Orbbec Gemini 2 | $234 | 850 nm, pulsed, Class 1 | 15 cm | low-accuracy binned mode only |
   | RealSense D415, Orbbec Astra Mini Pro, Femto Bolt | $150-418 | 850 nm | 25-60 cm | too far for the wrist |
   | RealSense D405, Orbbec Gemini 305, OAK-D Lite | $149-272 | **none** (plain stereo) | 4-7 cm | need texture, like our own looks |
   | Arducam ToF, Sipeed MaixSense A010 | $30-50 | 940 nm | ~20 cm? | coarse (240 x 180, 100 x 100), thin documentation |

   Best fits for this house: the pmd flexx2 (invisible, 10 cm, tiny) or the OAK-D Pro (invisible, Class 1, bigger).
   The 850 nm projectors pulse with the exposure; their faint glow would flicker in the risky band, and their
   alternating-emitter modes (15 Hz blink) must stay off if one is ever used. Sources: the survey's report of 29 Sep
   (RealSense D400 datasheet Aug 2025; Orbbec Gemini 330 datasheet v1.6, Gemini 2 v1.7; Luxonis OAK-D Pro datasheet;
   pmd flexx2 page; Arducam and Sipeed wikis).

3. [A8K] **A stereo pair of our own, in the sim first** (two ELP 720p USB cameras, CMOS OV9712, are on
   hand). Two ELP 720p USB boards (OmniVision OV9712, 1/4", 1280 x 720, rolling
   shutter, M12 lens mount) side by side on the printed wrist mount, 50 mm apart, where the wrist camera is, calibrated
   once with a checkerboard. A rigid pair is the exactly-known-pose case [x1U] lacked; frames are taken while the arm
   holds still, so the boards needn't be synchronised. Like any plain stereo it needs texture (a plain toy's top shows
   holes; its edge against a textured floor usually doesn't). In the sim (`HK_WRIST_STEREO=1`): the second camera
   50 mm along the image's x, lenses assumed 2.8 mm (69 deg across, 42 deg up-down at 1280 x 720; not yet chosen), the
   second camera's pose off by a calibration's residual (est. 0.05 deg, 0.1 mm), frames from Filament with the
   camera's imperfections; depth from OpenCV's semi-global matching; scored against the true pixels, Lego included:
   a 2 x 4 brick (32 x 16 x 11.5 mm), and as hard tests a 1 x 1 brick (7.8 mm square, 9.6 mm tall) and a 1 x 2 plate
   (15.8 x 7.8 x 3.2 mm) (`items.py`).
   The Pi 3B carries the pair as snapshots on demand (two ~100 KB JPEGs a look), and the matching runs on the main
   computer; matching on the Pi (~0.5-2 s a pair) is the fallback if the network is the limit.

   Resolution (29 Sep; full resolution is worth a few extra seconds a look if it helps across the board, not just
   with the 1 x 1): the first runs matched at 640 x 360 (binned), where the
   1 x 1 brick is ~14 px across and ~1.8 px of disparity above the floor, and neither stereo nor the detector (OWLv2
   works in ~16 px patches) found it in a single look though it sat plainly in view. The close look now takes both
   frames at the whole 1280 x 720 (`HK_STEREO_FULL=1`, the default; ~70 ms a pair to match here, 4 threads).

   Power (the Pi 3B's added processing and the second camera): the battery model counts the second camera all night
   (`maker_arm.stereo_camera_W`, 0.6 W: ELP's 100-120 mA at 5 V) and, during a close look, the Pi's work grabbing and
   sending the pair (`look_host_extra_W`, 1.5 W est.) and the white light (`wrist_light_W`, 1.1 W, [J7G]), on the Pi's 5
   V side through its buck converter (`hud.Hud.extra_W`).

   The cameras themselves, a matched pair simulated as
   they are: the pair replaces the stock wrist camera, the left one the colour
   camera the detector uses, both with one lens and one camera model, `maker_arm.elp_camera` in place of the phone-class
   profile: from the ELP-USB100W03M page (observed 2026-09-29: OV9712, SNR 40 dB, dynamic range 69 dB, 3.7 V/lux-s,
   rolling shutter, 1280 x 720 MJPEG at 30 frames/s, 5 V 100-120 mA, M12 lenses 2.1-12 mm) a full well of ~10,000 e- and
   ~3.5 e- of read noise, rows read over ~30 ms, no stream to congest; estimated, unkindly, MJPEG q75, half the raw noise
   kept by its denoise, and the 2.1 mm lens's barrel distortion calibrated away: the edges softer where the image is
   stretched, and a 1% residual in k1 of opposite sign in the two cameras (`camera_imperfect.lens_roundtrip`).

   Which resolution for which job (29 Sep): matched on the same full-resolution frames, stereo was no better at
   1280 x 720 than binned to 640 x 360 (median IoU with the true item 0.73 against 0.79, 70-75 looks); an earlier jump
   between runs (0.66 to 0.78) came from the camera model changing (the shared profile's stream bitrate dips, which the
   pair's USB frames don't have). So both cameras deliver 1280 x 720, the detector runs on the whole left frame (where
   the 1 x 2 plate was first found), and the pair is matched binned to 640 x 360 (`stereo_eval.MATCH_W`), the heights
   scaled back to the full frame.

   The lens (the maker's distortion map, a photo of a 3 cm grid
   through it): two **Arducam M40210M09S** (also LN014), 1/4" M12, nominal 2.1 mm, f/2.2, <1% distortion as listed,
   650 nm IR cut, 105/80/65 deg (D/H/V) on a 1/4" 4:3 sensor, $11.99 each (Arducam's page, observed 2026-09-29).
   Fitted to the maker's photo: effective focal length ~2.31 mm, k1 +0.034, k2 -0.072 (~0.6% barrel at a 4:3 frame's
   sides, ~3% in its corners; 5 inner grid lines, 1 px rms). On the OV9712's 1280 x 720: ~79 deg across, ~50 up-down.
   The sim takes these (`go1_arm.STEREO_LENS_MM` 2.31, `maker_arm.elp_camera` lens_k1/k2) until the pair is measured.

   The gap between the lenses: 40 mm, down from
   50, the closest the 38 mm boards allow. On the same landings (live benches, the pair, its light, the fitted lens),
   the stereo outline's median IoU with the true item rose from 0.75 to 0.82 on the near-limit toys and from 0.68 to
   0.84 on the too-big ones (0.7 or better on 78% of looks, from 49%): the bands beside an item that one camera sees and
   the other doesn't shrink with the gap. Too-big calls and refusals were unchanged (11 and 10 of 41; 1 of 3). Depth at
   30 cm stays near 1.5 mm a step, still well within a Lego piece's height.

   Too big, live (29 Sep; look-closer [GMs]): run live on the benches, the pair calls 11 of 41 too-big toys too big
   (base40, and again with the cut-off looks' sizes counted, live3), where the same rules replayed on saved looks called
   27; it wrongly refuses 1 of 3 grippable toys in the too-big set and 2 of 20 in the grip set. Live, a too-big toy is
   often never seen whole at the close look's height, and a cut look under-reads it. Left here, not worth long effort:
   too-big and very awkward toys are an edge case (1 in 8 toys a night, [Afy]); a depth camera ([GwO]) is the next lever
   if they matter later.
