# [6hI] — A light for the wrist camera: infrared LEDs, then a white one (sim trials)

**Status:** Proposed (sim trial, not adopted) · **Date:** 2026-09-29 · **Source:** a trial brief: replace the
wrist camera with an inexpensive IR-sensitive one in a form factor similar to the arm's current camera, a camera and
lens with enough published data to model them correctly in the sim, four 940 nm LEDs around it, on a new printed arm
camera mount in a similar location; LEDs from a trusted name brand with reliable data sheets; nothing may flash in the
visible spectrum; judged on finding objects, measuring their shape and size, and picking them up · **Serves:**
look-closer [xzk], [pYZ], [c1X] · **ID:** [6hI]

## Context

The close look measures a toy from one colour camera's picture, and on the too-big bench (29 Sep) it measured parts
for wholes and couldn't tell a patterned floor from a thing on it (look-closer [c1X], [xzk]). Four infrared LEDs round
the lens, fired one frame each, cast a thin shadow beside anything standing on the floor and none from a printed
pattern: the shadows give an item's true outline whatever the floor (multi-flash imaging, Raskar et al., SIGGRAPH
2004). Nothing may flash in visible light ([0Py], safe for photosensitive people), so the LEDs
are 940 nm, which the eye doesn't see (850 nm glows faint red).

## Decisions

1. [m52] **The parts, and what their makers publish** (observed 2026-09-29; forced: these are the sim's numbers).
   - Camera: **Arducam B0332**, USB (UVC) board camera, OmniVision **OV9281**: 1 MP (1280 x 800) monochrome,
     **global shutter**, 3 um pixels, 1/4", 68 dB; M12 lens, EFL 2.8 mm, **70 deg horizontal** (47 deg vertical at
     1280 x 800, derived), distortion under 1%; **no IR filter**; MJPG up to 120 frames/s; 5 V, 200 mA max; board
     38 x 38 mm, holes 34 and 28 mm apart (B0332 datasheet v1.0, 2021-05-11). The sensor has strobe output and
     per-frame context switching (OV9281 product brief v1.4, May 2024), so each LED can be fired for its own frame,
     and a global shutter exposes the whole frame under one LED (a rolling shutter would mix two).
   - LEDs: **ams OSRAM SFH 4181BS** (OSLON P1616 Lens), in production on 2026-09-29 (the SFH 4181S it follows is "not
     for new design"): 940 nm centroid, 37 nm wide (FWHM), **half angle 35 deg**, radiant intensity **765 mW/sr typ.
     at 1 A** (binned 630-900), 1 A continuous, 2 A pulsed (<= 600 us, duty <= 0.5%), rises in 9 ns; 1.6 x 1.6 mm
     (ams OSRAM product page, and the SFH 4181S datasheet v1.5, 2026-05-12, for curves the two share). Light output
     is near proportional to current (its relative-flux curve).
   - Not published, so estimated, each on the unkind side ([x3f]): the OV9281's quantum efficiency at 940 nm
     (OmniVision claims "best-in-class NIR" without a number; silicon typically keeps about half its 850 nm QE at
     940 nm): **12%**, and 55% for the room's visible light; the lens's f-number: **f/2.4**; read noise **4 e-**,
     full well **8000 e-**, 10-bit. Re-measure all on the real part before believing a result that rests on them.

2. [GWg] **The mount and how it is simulated** (29 Sep 2026). A printed mount replacing the arm's camera bracket, the
   lens where the stock wrist camera's is (`params/robot.yaml` wrist_camera_pose), looking the same way; the four
   LEDs at 30 mm from the lens axis, above, below, left and right in the image (est.: as far out as a ring the size
   of the board allows; further gives wider shadows). In the sim (`HK_WRIST_IR=1`): the wrist camera takes the B0332's
   field of view and a 640 x 400 frame (2 x 2 binned); each LED is a spot light on the wrist, off in every other
   render, with the datasheet's beam; an infrared frame is the classic MuJoCo renderer's linear light from that LED
   alone (materials keep their visible brightness as their infrared brightness: a printed floor pattern doesn't fade
   in infrared the way many dyes do, so the sim is not kinder than a real floor), turned into electrons from the
   LEDs' radiant intensity, the distance and the lens, plus the room's own light, which this unfiltered camera also
   sees (the Filament frame in mono, at the regime's light level), then shot noise, read noise, 10 bits and JPEG.
   A look is five frames: room light alone, then each LED; each LED's own light is its frame less the first.
   Layout, from renders of three toys (29 Sep): the LEDs at the four **corners** of a 35 mm-radius square, and the
   **jaws open** while looking. With the LEDs above,
   below, left and right, the gripper blocked the one on its side and threw a dark wedge up the middle of the view;
   with the jaws closed, the palm and fingers did the same to the lower corners. Open, the fingers stand aside and the
   floor ahead of the gripper is lit from all four; only the fingers' own narrow shadows remain.

   Scored offline against the sim's true pixels (an offline replay, 85 looks of the first 20 trials): the
   best flash-derived mask (walls where the floor fit's spread passes 0.10, plus depth edges, closed and filled)
   overlapped the true item at a median IoU of 0.32, 27% of looks at 0.7 or more; the others 0.08-0.25.
3. [yZ4] **Steady, not flashing, until flashing earns its place** (a plain flashlight would be
   simpler, if the flashing IR LEDs add nothing). On the first 25 paired trials of the too-big
   bench (same landings as the colour camera's) the infrared camera saw more of the toys (3 of 23 unseen against 7),
   plausibly from its own strong, even light, which the detector's image already was (the four LED frames' brightest);
   but the flash-derived masks (shadow ring, then floor fit) made sizes worse (no toy called too big, against 5): they
   mark edges and tilted faces, not a toy's level top. So the trial runs the same parts as an infrared flashlight: the
   four LEDs on together through a look, one frame, the detector's own mask (`HK_IR_MODE=steady`, the default).
   Nothing flashes, and nothing visible sweeps the house at night. Flashing stays available (`HK_IR_MODE=flash`, the
   camera's strobe output would time it) if an outline method is shown to help on the saved frames and true pixels.
4. [J7G] **A steady white light on the wrist, the colour camera kept** (perhaps an interim until a proper depth
   camera; the question is whether losing colour costs more than the white light gains). The detector was trained on
   colour photos, and a toy's hue against the floor is lost in mono infrared, where many dyes also wash out; so the
   stock wrist camera stays and a white LED is added: **Cree XLamp XP-G3 White**, 90 CRI, 4500 K (XPGDWT-U1-0000-00GE4:
   flux bin R4, 130 lm minimum at 350 mA and 85 C), 125 deg viewing angle, up to 2 A, 3.45 mm (Cree data sheet CLD-DS139
   rev 21, observed 2026-09-29). At 350 mA and near-Lambertian, 41 cd on axis: ~460 lux at 30 cm, six times the lit
   house. On the printed mount 35 mm from the lens on the side away from the jaws (the jaws' shadows fall back toward
   the robot), aimed with the camera; on for the whole close look and off after, fading over ~0.5 s on the real one,
   never pulsed. In the sim (`HK_WRIST_LIGHT=1`) it is a Filament spot light that goes where the wrist is, at 8340 of
   Filament's units: its SPOT light gives intensity/pi candela, and the lit house's floor, taken as the ~70 lux it was
   set to, comes to ~4500 of them, 64 a lux (the same light for the head camera, which sees the patch it throws, as a
   real one would).

   Built into the camera holder, since new holders are being designed anyway: with the stereo pair ([A8K]) the LED sits
   midway between the two lenses, 35 mm toward the side away from the jaws, so both cameras see the same light (the
   matcher compares them); a diffusing window over it (a frosted cap or thin printed translucent wall) to soften the
   hotspot and glints, which look different from each camera; a small metal pad or star in the holder for its ~1 W;
   driven by a constant-current (linear or buck) driver the Pi only switches and ramps, never PWM-dimmed (kHz flicker is
   invisible to most people, but no light here may flicker).
   Three in a row, left, middle and right, for an even flood: with the pair, three XP-G3s 35 mm above the lens line, one
   midway between the lenses and one 45 mm either side, sharing the one light's current (~117 mA each: the same ~1.1 W
   and ~450 lux at 30 cm, spread evenly and with softer shadows than one point source; switched and ramped together).
