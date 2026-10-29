"""Every dimension of the printed mount in one place ([3JA] [Lsn]): read from params/robot.yaml where it is a fact about
the robot or the arm, from cad/materials/materials.json for the filaments, and set here where it is a design choice.

Frame (all parts, all exports): the robot's, in millimetres: X forward, Y left, Z up, origin at the Go2's body centre
(models/README.md). The interface frame is the same axes with its origin on the arm's base axis at the interface plane
(X = MOUNT_X, Y = 0, Z = Z_IF): the module, the adapter and the bench plate all meet there."""
from pathlib import Path
import json
import sys

R = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(R / "src"))
from houndkeeper import params as P                                        # noqa: E402

# ---- the robot and the arm (params/robot.yaml) ----
MOUNT_X = 1000.0 * P.get("maker_arm.mount_x_m")                            # [Pvh]: arm base axis, forward of body centre
ARM_MASS = P.get("maker_arm.mass_kg")                                      # 3.7 kg
REACH = 1000.0 * P.get("maker_arm.reach_tip_m")                            # 670 mm, measured
PAYLOAD = P.get("maker_arm.payload_kg_at_50cm")                            # 1.5 kg
KIT_BUDGET = P.get("kit.budget_kg")                                        # [Rrm] 1.8 kg beyond the arm
SIM_FLEX = P.get("maker_arm.frame_flex_Nm_per_rad")["base_link"]           # 1500 N m/rad, the sim's estimate
BACK = P.get("go2_air.back_mount")                                         # Unitree CAD: the back's holes and cover height
ARM_BASE = P.get("maker_arm.base_bolts")                                   # the arm base's footprint and bolt pattern

# The arm's base plane (top of the module) stays where the sims put it, so the sims' reach and tipping still hold:
# the Menagerie body's top under the base (82.4 mm, models/README.md) plus the plate the sims assume (mount_plate_m).
SIM_SEAT_Z = 82.4
Z_BASE = SIM_SEAT_Z + 1000.0 * P.get("maker_arm.mount_plate_m")            # 118.4 mm above body centre
COVER_Z = BACK["cover_top_z"]                                              # 57.0: the flat of the back, X -120..+112

# ---- the module ([JW7], [vQO]) ----
MOD_H = 32.0                         # module height: the Pi 3B (20 mm over its board, 3 under) on 4 mm stand-offs, a 3 mm top
Z_IF = Z_BASE - MOD_H                # 86.4: the interface plane (module underside = adapter and bench-plate top)
MOD_X = (0.0, 152.0)                 # rear edge (behind the arm base, so the interface bolts stay reachable) .. front
MOD_HALF_W = 100.0                   # the arm base is 200 mm across (its flanges)
MOD_WALL = 3.0                       # module side walls and ribs
MOD_TOP = 4.0                        # module top skin under the arm base
EAR = dict(x=(98.0, 118.0), y_out=114.0)   # front ears beside the arm base, for the front pair of interface bolts

# Pi 3B 85 x 56 (Raspberry Pi mechanical drawing; holes 58 x 49), buck 45 x 25 (est., a 40 V-in 5 V 3 A module),
# USB-CAN adapter 60 x 22 (est.), speaker 40 mm round (est. 3 W), MEMS microphone board 14 x 14 (est., INMP441-class)
PI = dict(size=(85.0, 56.0), holes=(58.0, 49.0), hole_off=(3.5, 3.5), centre=(106.0, 0.0), standoff=4.0)
BAYS = dict(pi=((62.0, 148.0), (-33.0, 33.0)), power=((4.0, 58.0), (-33.0, 33.0)),
            left=((4.0, 148.0), (42.5, 97.0)), right=((4.0, 148.0), (-97.0, -42.5)))
SPEAKER_D = 40.0
CONNECTOR = dict(power=(10.2, 16.2), data=(16.0, 14.0))   # rear-wall cut-outs: XT30 panel mount, USB/RJ45 feed-through (est.)

# ---- the module interface (same on the adapter and the bench plate, [JW7]) ----
# robot frame (X, Y); M5 socket-head bolts from the module's top into M5 heat-set inserts; two 6 mm steel dowel pins
# pressed into the adapter / bench plate, a round hole and a slot (along Y) in the module.
IF_BOLTS = ((14.0, 60.0), (14.0, -60.0), (108.0, 108.0), (108.0, -108.0))
IF_DOWELS = ((34.0, 82.0), (34.0, -82.0))
DOWEL_D, DOWEL_PROUD = 6.0, 6.0

# ---- the adapter ([vqz]) ----
ADP_X = (-127.0, 112.0)              # the back's flat, rear edge of the top cover .. where the head's rise begins
ADP_HALF_W_FOOT = 38.0               # where it sits flat on the cover (the cover is flat within |Y| 38)
ADP_HALF_W = 100.0                   # its deck (the cover's sides fall away beyond 58: the deck overhangs them)
ADP_SIDE_LIFT = 7.0                  # beyond the foot the underside clears the cover's side ridge (<= 62.5 behind X 56)
REAR_BOLT = "M4"                     # assumed (the CAD does not model the thread; measure, then set)
BOLT_CLEAR = {"M3": 3.4, "M4": 4.5, "M5": 5.5, "M8": 8.5}                         # ISO 273 medium fit
HEAD_D = {"M3": 5.5, "M4": 7.0, "M5": 8.5, "M8": 13.0}                            # ISO 4762 socket heads
INSERT = {"M3": (4.0, 5.7), "M4": (5.6, 8.1), "M5": (6.4, 9.5)}                    # heat-set insert hole (dia, depth): Ruthex
                                                                                  # figures, est. (print a coupon first)
PRINT_HOLE_ADD = 0.2                 # holes print undersize in PC; tuned by the coupon (cad/mount/print/coupon.*)

# ---- the bin ([Frm]) ----
BIN_X = (ADP_X[0] + 1.0, MOD_X[0] - 6.0)  # -126 .. -6: the back behind the module (6 mm for the hat's brim)
BIN_HALF_W = 75.0
Z_BIN = Z_IF                         # the bin sits on the same deck: one flat top, so the adapter prints deck-down
                                     # without support (a 16 mm raised bin seat cut the walking stress by a quarter,
                                     # but its step would hang over the bed: see report, options)
BIN_TOP_Z = 188.0                    # the folded arm's rear segment is 198.4 at its lowest over the bin (X -80, stow pose
                                     # from models/export/maker_arm_folded_mm.stl at Z_BASE): 10 mm clear, checked in analysis
BIN_WALL, BIN_FLOOR = 1.6, 2.0
MAGNET = dict(d=10.0, h=3.0, grade="N42", pull_N=17.7, recess=0.3,
              src="supermagnete S-10-03-N, Ø10 x 3 mm N42: adhesive force ~1.8 kg (17.7 N) on steel (observed 4 Oct 2026)")
MAGNET_GAP_DERATE = 0.6              # a magnet pair 0.6 mm apart (two 0.3 mm recesses) keeps ~60% of the steel-plate pull (est.)
MAGNETS_XY = ((-114.0, 66.0), (-114.0, -66.0), (-50.0, 66.0), (-50.0, -66.0))
# the stated keep-out ([Frm] [4JP]): every magnet at least 120 mm from the arm's motors (RobStride magnetic encoders;
# their own encoder magnet is tens of mT at a few mm, a pair here adds < 0.05 mT, under Earth's field) and 80 mm from the
# Go2's IMU (~0.15 mT there from one pair: matters only if Unitree fuses a magnetometer, unknown)
MAGNET_MIN_MM = dict(encoder=120.0, imu=80.0)
CONES_XY = ((-98.0, 0.0), (-32.0, 0.0))  # two 10 mm stainless (304, non-magnetic) balls half-proud in the adapter's deck; a
                                         # 45 deg cone socket and a 45 deg V-slot (along X) in the bin's floor
CONE = dict(d=13.94, h=6.97, half_angle_deg=45.0, mu=0.3, ball_d=10.0)   # a 45 deg socket 6.97 deep: the ball (centre on the
                                         # deck) touches its flanks with the bin's floor 0.1 mm clear of the deck

# ---- the hat's seat ([4JP]): a boss the bowler's crown opening drops over, a magnet in its top, beside the arm base
HAT = dict(centre=(22.0, -80.0), brim_d=50.0, crown_open_d=31.0, height=40.0, mass_g=40.0, boss_d=29.0, boss_h=5.0, flange_d=46.0,
           flange_h=3.0, screw_r=19.0,
           note="the bowler's size and mass are estimates (a small printed hat with a steel disc or magnet in its crown): measure")
MIC_XY = (30.0, 72.0)

# ---- the bench plate ([JW7]) ----
BENCH_X = (-12.0, 138.0)
BENCH_HALF_W = 120.0
BENCH_T = 22.0
BENCH_HOLES = ((-2.0, 110.0), (-2.0, -110.0), (128.0, 110.0), (128.0, -110.0))   # M8 to the bench (or C-clamp the flange)

# ---- printing (the project's Bambu X1, [ngX]) ----
BED = 256.0
NOZZLE, LINE_W, LAYER = 0.4, 0.42, 0.2
WALLS, TOPBOT_LAYERS, INFILL = 8, 18, 0.15                # 3.4 mm walls, 3.6 mm top/bottom skins, 15% gyroid: the skins
                                                          # carry the load ([Xtj]: sparse infill is not credited)
SKIN_SIDE = WALLS * LINE_W
SKIN_TB = TOPBOT_LAYERS * LAYER

MATERIALS_FILE = R / "cad/materials/materials.json"
ALLOWABLES_FILE = R / "cad/materials/allowables.json"     # [Xtj]: long-term design moduli and stress allowables
CANDIDATES = ("polymaker_polymax_pc", "3dxtech_carbonx_ezpc_cf", "bambu_pc")
# CarbonX ezPC+CF publishes no across-layer modulus: [Xtj]'s allowables take unfilled PC's low Z value (fibre lies in the
# layers and does not stiffen the bond between them); the raw-datasheet fallback below is only for the record.
Z_FALLBACK_MPA = 1450.0


POISSON = 0.37                       # PC (est.), for the shear modulus G = E / 2(1 + nu)


def materials():
    """per candidate: density, the datasheet moduli, and [Xtj]'s long-term design moduli (the low end of the modulus,
    at 40 C, after creep) and allowables, which the flex and stress estimates use"""
    m = json.loads(MATERIALS_FILE.read_text(encoding="utf-8"))["materials"]; a = json.loads(ALLOWABLES_FILE.read_text(encoding="utf-8"))["materials"]
    out = {}
    for k in CANDIDATES:
        d = dict(m[k]); d["E_z_assumed"] = d["E_z_MPa"] is None
        if d["E_z_MPa"] is None: d["E_z_MPa"] = Z_FALLBACK_MPA
        d["E_xy_design"] = a[k]["stiffness"]["xy"]["E_design_long_term_MPa"]
        d["E_z_design"] = a[k]["stiffness"]["z"]["E_design_long_term_MPa"]
        d["E_xy_short_40C"] = a[k]["stiffness"]["xy"]["E_short_term_40C_MPa"]
        d["allow"] = a[k]["strength"]
        out[k] = d
    return out


# electronics and hardware on the robot, grams (est. unless noted)
HARDWARE_G = {"Raspberry Pi 3B (45 g, Raspberry Pi)": 45, "5 V buck converter": 25, "USB-CAN adapter": 20, "PTC polyfuse + holder": 5,
              "speaker 40 mm": 30, "microphone board": 4, "wiring, XT30, feed-through": 80, "M5/M4 bolts, inserts, dowels": 45,
              "magnets 9 x Ø10x3 (1.8 g each)": 16, "2 stainless balls 10 mm": 8}
