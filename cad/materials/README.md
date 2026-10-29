# Mount materials

Manufacturers' printed-specimen data for the mount's analysis ([3JA], [tY3]), in
`materials.json`. The datasheets themselves are not in git; fetch them by the URLs in `materials.json` (checked 4 Oct
2026). Prusament PC Blend publishes only a single tensile modulus (1900 MPa, ISO 527-1, direction not stated), so it is
not in the comparison until its directional data is found.

## Design values: independent tests, knock-downs and a safety factor ([Xtj])

The allowables rest on data from people who have tested the filaments, not only the makers, and carry a safety margin.
`independent.json` holds every independent source found on 4 Oct 2026, each with who tested, the method and the
numbers as published, and says whether the full text or only the abstract was read. `allowables.json` turns them into
the values the FEA ([tY3]) designs to. Every assumption there is marked `ASSUMPTION`.

### What the independent tests say

No independent test of any of the three candidates was found. The nearest are other PC grades on other printers:
CNC Kitchen's test of Prusament PC Blend, two IQS (Barcelona) papers on Stratasys PC-ISO, a NOVA Lisbon fatigue study on
Stratasys PC, a JHU/NIST moisture study, and one PC-CF strength (Prusa PC-CF, Remache 2026). They all agree on four points:

| | Makers say | Independent tests show |
|---|---|---|
| Strength along the layers | 53 to 73 MPa | 53 to 71 MPa (median 60.5); close to the makers |
| Strength across the layers (Z/XY) | 0.90 (Bambu PC), 0.78 (PolyMax PC), not published (CarbonX) | 0.35 to 0.61, median 29 MPa. The makers' annealed, dried specimens flatter Z badly |
| Modulus | 2110 to 2435 MPa (PC), 6980 (CarbonX) | 1549 to 2510 MPa for PC; Z modulus about 0.74 of XY. No credible independent PC-CF modulus |
| Fatigue | not published | 7% of static strength at 1e6 cycles (R = 0.05); about 3.3% at 1e7 by the authors' own fit |

The hardened steel nozzle matters: CNC Kitchen measured 25 MPa across the layers with a steel nozzle against 43 MPa
with brass, same filament and G-code. The X1 Carbon ships with a hardened steel nozzle, so PC should be printed at the
top of its temperature range. Wet PC loses about 30% of its strength (Fang 2020), so it is dried and fed from a dry
box.

### Knock-downs and safety factors

Start from the lower of the maker's value and the independent median, then:

| Factor | Value | Basis |
|---|---|---|
| Print quality, along / across layers | 0.90 / 0.80 | not annealed, passive chamber, steel nozzle (CNC Kitchen); size an assumption |
| Moisture (dried, dry box) | 0.90 | Fang 2020; use 0.70 if the filament was not dried |
| Infill | 1.0 | only if load paths are solid; sparse infill is not credited (30% infill gave 0.30 of solid, Akgumus 2026) |
| Temperature, 40 C beside the electronics: strength / modulus | 0.85 / 0.90 | assumption; no printed-PC data at 35 to 40 C found |
| Creep, the arm's permanent weight: strength / modulus | 0.50 / 0.70 | bulk PC creep modulus 0.79 at 1000 h (Covestro); creep rupture at 0.7 to 0.8 of yield (Mu 2022); extrapolated, assumption |
| Fatigue, 1e7 walking cycles (about two years) | 0.033 of strength, on the stress range | Rodrigues 2026, Basquin fit extrapolated from 1e6 (0.07 measured) |
| Safety factor: sustained and working / fall / fatigue | 3.0 / 2.0 / 1.5 | a failure drops the 3.7 kg arm; Z failure is brittle with up to 17% scatter; most data is not from this project's printer. 3 may come down to 2 once the printed adapter passes its load test ([vqz]) |

PC-CF has no Z, creep or fatigue data anywhere. Its Z strength is assumed 0.40 of XY, its Z modulus that of unfilled PC,
and its unchecked maker modulus is knocked down by 0.75 (all assumptions).

### The allowables (MPa)

| | Bambu PC | PolyMax PC | CarbonX ezPC+CF |
|---|---|---|---|
| Sustained (arm's weight), tension along / across layers | 6.9 / 3.0 | 6.1 / 3.0 | 6.1 / 2.2 |
| Working peak, tension along / across | 13.9 / 5.9 | 12.3 / 5.9 | 12.2 / 4.3 |
| Working peak, bending along / across | 17.6 / 11.1 | 17.6 / 11.1 | 15.6 / 5.5 |
| Fall, tension along / across | 20.8 / 8.9 | 18.4 / 8.9 | 18.2 / 6.5 |
| Walking stress range at 1e7 cycles, along / across | 0.92 / 0.39 | 0.81 / 0.39 | 0.80 / 0.29 |
| Design modulus, long-term at 40 C, along / across | 976 / 722 | 976 / 722 | 3298 / 722 |
| Density (g/cm3) | 1.20 | 1.19 | 1.36 |

Fatigue governs. If walking swings the load by about the arm's own weight (an assumption until the IMU log is read), the
arm's static stress in the mount must stay near 1 MPa. A mount stiff enough for [lUO] will probably be stressed this
lightly anyway; the FEA reports both.

### What the data favours

For a flex-conscious, weight-conscious mount whose loads run along the layers ([ngX]), **CarbonX ezPC+CF**. Even after
a 25% knock-down on its unchecked modulus, it is about 3.4 times as stiff as unfilled PC. A plate thickened to the
same bending stiffness is about 25% lighter in PC-CF, and its strength along the layers is about the same as PC's. Across
the layers it is weaker and brittle, so parts are oriented to keep load off the layer bonds. The choice rests on a modulus
nobody independent has measured. Before it is final, print three flat 3-point-bend coupons of CarbonX and of one PC on
the X1 and measure them (a ruler, dead weights and a dial gauge are enough, as CNC Kitchen does). If CarbonX comes in
under about 3000 MPa, re-run the comparison. Between the two unfilled PCs the data cannot choose. Bambu PC has the higher
HDT (117 C against 99 C) and is tuned for the X1.

### Replacing assumptions with measurements

1. Bend coupons from the X1, flat and upright, for each candidate: modulus and strength along and across the layers,
   with the actual nozzle, settings and drying.
2. The printed mount's stiffness under the arm, measured on day one and again after weeks under load: the measured
   creep replaces `creep_E` ([lUO]).
3. The dog's IMU while walking with the arm: the real dynamic load factor for the fatigue range.
4. Salazar Gabino 2018 (creep) and Puigoriol-Forcada 2018 (flexural S-N at R = -1) are paywalled. Buying them would
   replace two assumptions.
