# [wjx] — Jaw pads: printable TPU, a collapsible face, and a sim that grips with what the pad would

**Status:** Proposed · **Date:** 2026-09-25 · **Source:** design brief: printable TPU pads, designed to real TPU
values, with collapsible geometries (such as the zigzag rib common on robot grippers), tried in sim · **Serves:** [GU5],
[2ho], [v8J] (grasp spec), [4hm] (calibration) ·
**ID:** [wjx]

## Context

In the sim on 25 Sep a ballet flat, gripped within 2 cm of its centre of mass, turned 13-42 deg in the jaws on the way
to the bin, and once 172 deg and fell. The jaws met the curved shoe at single points, and a point contact resists
almost no twist. Rubber-like pads are the usual answer; the question was which pad, from which material, and how to
make the sim believe it no more than the printed part deserves. Three other things came to light on the way: the
sim's jaws had been modelled grippier than any real jaw (friction 1.2, a 2 cm twist allowance at every point),
MuJoCo had been using the items' friction for every jaw contact, and the widest-item limit was hard-coded in five
places. The pad generator is `cad/gripper_pads/build_pads.py`; the jaws the sims fit are chosen in `sim/arm/jaws.py`.

## Decisions

1. [8ME] **Forced: printable TPU stiffness, from the makers' datasheets** (all checked 2026-09-25). Young's modulus
   in the print plane, which is the pad's thickness direction as it prints tip-up:

   | Grade | Modulus | Test | Printing |
   |---|---|---|---|
   | Bambu TPU 95A HF | 9.8 +- 0.7 MPa (7.4 across layers) | ISO 527 | overhang ~55 deg, bridges 20 mm, 220-240 C |
   | Bambu TPU 85A | 6.8 +- 0.7 MPa (5.2 across layers) | ISO 527 | overhang ~40 deg, bridges 10 mm |
   | NinjaTek Cheetah 95A | 26.2 MPa (3,800 psi) tensile modulus | ASTM D638 | |
   | NinjaTek NinjaFlex 85A | 12.4 MPa (1,800 psi) tensile modulus | ASTM D638 | |

   The two makers' test methods differ, so compare within a maker. Bambu's 90A sheet was not read (a search summary
   gave 5.3 MPa, softer than their 85A, which is out of order; not used). Default: Bambu TPU 95A HF, made for the
   X1 Carbon the pads are printed on.

2. [Iin] **Estimate, to measure: TPU's sliding friction on household items is 0.6.** Printed TPU measured against a
   steel ball ranges 0.37 (a 98A at 100% infill) to 1.17 (the same at 10% infill), and 0.59-0.73 for a foamed 70A;
   nothing published measures it against fabric, cardboard or toy plastic. `params/robot.yaml` (`pad_tpu_mu`) holds
   it until `real/calibration/jaw_pads.md` measures it.

3. [2L1] **Hard-won: the pad's face collapses rather than compresses.** A thin solid rubber layer bonded to a rigid
   back can't bulge sideways where it's squeezed, so it acts stiffer than its modulus (its shape factor): a solid 3 mm
   95A face (v1) makes a contact patch only ~2.6 mm in radius on a curved item at the gripper's squeeze, and gives
   0.2 mm. Thin ribs at +-45 deg under a 0.8 mm skin (v2) bend instead, as a honeycomb collapses (bending-dominated,
   E*/E ~ (t/l)^3), give ~1.5 mm and make a ~7.7 mm patch, three times the twist resistance. The grade changes it by
   10-25%; the structure by 3x. The zigzag rib is also common on robots' grippers. Cost: each
   pad takes jaw travel, so the widest item the open jaws take falls from 95 mm bare to 89 mm (v1) or 87 mm (v2);
   the limit is computed in one place (`jaws.max_m`) from the fitted pads.

4. [WFg] **The sim grips with what the pad would, computed from the design, not tuned.** Each design's contact values
   come from its material and structure (`build_pads.contact`): the face as an elastic layer (a Winkler foundation,
   stiffness k = E_eff / thickness per unit area) pressed by an item surface of radius R = 20 mm (est.) with the sim's
   squeeze N = 7.5 N per jaw (est.; the real squeeze is unknown): patch radius a = (4 R N / (pi k))^(1/4), capped at
   the face's half-width; with that paraboloidal pressure the twist resisted is (8/15) mu N a, which is MuJoCo's
   torsional friction coefficient (a length, read as the patch's size). The jaws' geoms have contact priority, since
   MuJoCo otherwise takes the grippier surface of a pair, which was always the item's. The earlier "rubber" jaws stay
   selectable (HK_JAWS=rubber) so older results can be reproduced, but are not a real jaw.

5. [NKb] **Pencilled in: the 3 mm ribbed pad (v3), to revisit once there's hardware**. It
   keeps the widest item at 8.9 cm, losing ~1% of item landings to "too wide" (the same as solid v1; v2's 4 mm loses
   ~5%), for ~11 N mm of twist resisted per jaw against v1's 6 and v2's 19 (est., `build_pads.contact`). Its voids
   are ~0.7 mm tall, near a 0.4 mm nozzle's limit: the slicer preview must show them open. Revisit the thickness
   after the bench test (`real/calibration/jaw_pads.md`): if a thicker pad clearly grips better, 4 mm costs ~4 points
   of the widest items.

## Consequences

- **First sim result (25 Sep, 8 ballet-flat trials each, seed 5): the jaws' material made no measurable difference.**
  Binned 7/8 rubber, 7/8 bare, 6/8 pads v1, 6/8 pads v2; the shoe turned by similar amounts in all four, though
  friction differed 3x and twist resistance 200x. Every setup gripped with the fingertips' last ~2 cm, one contact
  point per jaw, and the shoe swung about that pivot until it settled against the fingers. So in this sim the grip's
  geometry (where on the fingers, and through the centre of mass or not) governs the turn, not the pad; the pad's
  real benefit has to be shown on the bench, where a pad actually conforms. Eight trials can't separate 6/8 from 7/8.

- Grasp results depend on which jaws were fitted: every reported result says HK_JAWS. Results up to 25 Sep were all
  on "rubber" jaws.
- The pads' values are estimates end to end until the bench procedure measures squeeze, sliding friction and twist
  resistance on a printed pad; then `params/robot.yaml` changes to measured and the grasp benchmark re-runs ([7Xn]).
- A pad revision is a new entry in `build_pads.DESIGNS`: the STL, its print check and its sim values come out
  together, so a design can't be printed without also being simulated, or simulated with values it wasn't built for.
- MuJoCo also lets a pinched item creep out of a grip even with ample friction; `noslip_iterations` cuts that creep
  about 110x (issue #3328), but it changed how the lying robot settled here and was set aside ([Udt]).

## Sources

- N. Xydas and I. Kao, "Modeling of contact mechanics and friction limit surfaces for soft fingers in robotics, with
  experimental results", International Journal of Robotics Research 18(9), 1999: contact radius grows as N^gamma,
  0 <= gamma <= 1/3; the friction limit surface is near an ellipse whose moment axis scales with the patch.
  <https://journals.sagepub.com/doi/abs/10.1177/02783649922066673>
- MuJoCo documentation, Computation chapter: torsional friction opposes rotation about the contact normal, models
  soft fingers and stabilises grasps; its coefficient has units of length, read as the contact patch's size.
  <https://mujoco.readthedocs.io/en/latest/computation.html> (read 2026-09-25)
- MuJoCo issue #3328, "Regularized friction never sticks: a pinched capsule ... creeps through the grip" (MuJoCo
  3.8.1, June 2026): `noslip_iterations=10` cut creep ~110x; condim 6 had no effect.
  <https://github.com/google-deepmind/mujoco/issues/3328>
- Bambu Lab, TPU 95A HF Technical Data Sheet v1.0.
  <https://store.bblcdn.eu/s8/default/16df21baf482453999b3dbb61cc110e7/Bambu_TPU_95A_HF_Technical_Data_Sheet.pdf>
- Bambu Lab, TPU 85A Technical Data Sheet v1.0.
  <https://www.createeducation.com/wp-content/uploads/2025/08/Bambu_TPU_85A_Technical_Data_Sheet_a7233668-497b-487f-af8a-3adb49fafaa1.pdf>
- NinjaTek, Technical Specifications (Imperial): Cheetah, NinjaFlex and others.
  <https://ninjatek.com/wp-content/uploads/tech-data-imperial.pdf>
- Tribological studies of FFF-printed TPU against a steel ball: SMARTFIL FLEX 98A 1.174 at 10% infill to 0.371 at
  100%; FILAFLEX FOAMY 70A 0.585-0.729 (as summarised in search results, 2026-09-25; primary text not read); TPU 82A
  at 10/50/100% gyroid infill, 5 N, 300 m (MDPI, 2025; page returned 403, values not read).
  <https://www.mdpi.com/2673-7590/5/4/184>
- L. J. Gibson and M. F. Ashby, Cellular Solids: Structure and Properties (2nd ed., Cambridge, 1997): bending-dominated
  cells have relative stiffness scaling as (t/l)^3 in the wall's slenderness (the rib estimate here).
- "Towards Flexible Biolaboratory Automation: Container Taxonomy-Based, 3D-Printed Gripper Fingers" (arXiv 2302.03644):
  hard 95A TPU made soft and grippy as a pad of 0.4 mm sheets acting as springs.
  <https://arxiv.org/pdf/2302.03644>
- "A 3D Printed Modular Soft Gripper Integrated With Metamaterials for Conformal Grasping", Frontiers in Robotics and AI,
  2021: TPU auxetic structures and compliant ribs printed in one step for conformal grasps.
  <https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2021.799230/full>
