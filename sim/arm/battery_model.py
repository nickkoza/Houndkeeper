#!/usr/bin/env python3
"""Battery model for a Go2 Air + Maker Arm night. Every number here is either published (source noted) or an estimate
(est.) to be replaced by measurements on the real robot. Three ways to power the arm:
  direct  - arm runs off the Air's pack at 28-33.6 V (RobStride motors accept 24-60 V; lower top speed, same torque)
  boost   - switching boost converter from the Air's pack to 48 V
  own     - separate 12S LiPo for the arm (Air's pack only walks)"""
from dataclasses import dataclass, field
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from houndkeeper import params as P                  # the robot's sourced numbers live in params/robot.yaml, once

@dataclass
class Params:
    air_Wh: float = float(P.get("go2_air.battery_Wh"))   # 8000 mAh x 29.6 V = 236.8 Wh (Unitree, go2/battery)
    usable: float = 0.80                  # keep 20% reserve to get home (est.)
    walk_W: float = float(P.get("go2_air.walk_power_W"))   # Air walking, no payload (params: ~170 W from a measured 1.4 h patrol)
    stand_W: float = 90.0                 # standing still (est.)
    lie_W: float = 35.0                   # lying down, legs relaxed, onboard compute still on (est.)
    payload_kg: float = 3.7 + 1.8         # arm (MakerMods) + mount/tray/battery allowance (user)
    robot_kg: float = 15.0                # Unitree
    arm_standby_W: float = 6.0            # 7 motors x 48 V x 18 mA (RobStride manuals)
    arm_hold_W: tuple = (9.8, 20.9)       # arm rigid in stow while walking (sim + manuals; low/high winding resistance est.)
    arm_pick_Wh: tuple = (0.16, 0.46)     # one pick incl. unfold, look, grasp, tray, stow (sim, ~31 s)
    arm_look_Wh: tuple = (0.05, 0.15)     # look-only at a false alarm (est. ~1/3 of a pick)
    host_W: float = 5.0                   # arm host (Raspberry Pi 3B): ~400 mA typical bare board (Raspberry Pi docs) + camera, CAN, Wi-Fi (est.)
    stereo_camera_W: float = float(P.get("maker_arm.stereo_camera_W"))     # the stereo pair's second camera, all night ([A8K])
    look_host_extra_W: float = float(P.get("maker_arm.look_host_extra_W"))   # the Pi grabbing and sending the pair, during a look
    wrist_light_W: float = float(P.get("maker_arm.wrist_light_W"))         # the white wrist light, during a look ([J7G])
    buck_eff: float = 0.90                # 5 V buck converter from the Air's pack (est.)
    boost_eff: float = 0.94               # switching boost 30 -> 48 V, 50-300 W range (typical synchronous boost, est.)
    boost_q_W: float = 0.8                # boost quiescent/switching overhead at light load (est.)
    arm_pack_Wh: float = 44.4 * 1.3       # own battery option: 12S LiPo 1.3 Ah (est. ~0.4 kg)
    cradle: bool = False                  # arm unpowered in a cradle while walking
    sweep_m2_per_min: float = 12.0        # sim ground floor: ~164 m2 in ~14 min of sweeping at real time (s8 runs)
    map_min: float = 5.0                  # re-localise / update map before the sweep (est.; sim took ~10 min incl. exploration)
    stop_overhead_s: float = 25.0         # per stop: stop, lie down, arm up, stow, stand up (sim script ~20-25 s)
    look_s: float = 12.0                  # per candidate at a stop: move over, look twice, VLM (sim ~6 s + moves)
    pick_s: float = 20.0                  # per real item: grasp, lift, tray (sim)
    walk_between_stops_m: float = 1.5     # average extra walking per stop (replaced by the clustering route when available)
    walk_speed: float = 0.4

def night(area_m2, cand_per_m2, real_frac, stops_per_cand, p=Params(), option="direct", hi=True):
    """energy (Wh) drawn from each pack for one night over area_m2. hi=True uses the high end of every estimate range."""
    k = 1 if hi else 0; pay = (p.robot_kg + p.payload_kg) / p.robot_kg          # walking power scales ~ with total mass (est.)
    n_c = area_m2 * cand_per_m2; n_real = n_c * real_frac; n_stops = n_c * stops_per_cand
    t_sweep = area_m2 / p.sweep_m2_per_min * 60; t_map = p.map_min * 60
    t_between = n_stops * p.walk_between_stops_m / p.walk_speed
    t_walk = t_sweep + t_map + t_between
    t_lie = n_stops * p.stop_overhead_s + n_c * p.look_s + n_real * p.pick_s
    legs = (p.walk_W * pay * t_walk + p.lie_W * t_lie) / 3600
    arm = (0 if p.cradle else p.arm_hold_W[k] * t_walk / 3600) + n_real * p.arm_pick_Wh[k] + (n_c - n_real) * p.arm_look_Wh[k] \
          + (p.arm_standby_W * t_lie / 3600 if p.cradle else 0)
    host = p.host_W / p.buck_eff * (t_walk + t_lie) / 3600                       # the Pi, on the Air's pack all night
    legs = legs + host
    if option == "direct":  air, armpack = legs + arm, 0.0
    elif option == "boost": air, armpack = legs + arm / p.boost_eff + p.boost_q_W * (t_walk + t_lie) / 3600, 0.0
    else:                   air, armpack = legs, arm
    return {"air_Wh": air, "arm_pack_Wh": armpack, "minutes": (t_walk + t_lie) / 60, "stops": n_stops, "candidates": n_c}

def max_area(cand_per_m2, real_frac, stops_per_cand, p=Params(), option="direct", hi=True):
    lo_a, hi_a = 1.0, 5000.0
    for _ in range(60):
        a = (lo_a + hi_a) / 2; e = night(a, cand_per_m2, real_frac, stops_per_cand, p, option, hi)
        ok = e["air_Wh"] <= p.air_Wh * p.usable and e["arm_pack_Wh"] <= p.arm_pack_Wh * p.usable
        lo_a, hi_a = (a, hi_a) if ok else (lo_a, a)
    return lo_a
