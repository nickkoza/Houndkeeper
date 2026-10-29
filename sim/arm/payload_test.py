#!/usr/bin/env python3
"""Can DimOS's Go1 walking policy (trained without payload) stand and walk with the Maker Arm stowed on its back?
Same command script with and without the arm; reports falls, tilt, trunk height and speed tracking."""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import sys
import numpy as np, mujoco
sys.path.insert(0, str(_HK / "sim" / "arm")); import go1_arm as ga
STOW = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]            # the arm's factory fold (URDF zero; matches the MakerMods photo), gripper forward
STOW_SEARCHED = [0.05, -3.02, 0.09, -1.17, -0.61, 0.99, 0.0]
STOW_OFFCENTRE = [0.38, -2.77, 0.11, -0.93, 1.19, -2.26, 0.0]
SCRIPT = [(3, (0, 0, 0), "stand"), (6, (0.5, 0, 0), "walk 0.5 m/s"), (4, (0, 0, 0.8), "turn 0.8 rad/s"), (4, (-0.3, 0, 0), "back 0.3 m/s"),
          (4, (0, 0.3, 0), "sideways 0.3 m/s"), (4, (0.8, 0, 0), "walk 0.8 m/s"), (3, (0.5, 0, 0.8), "walk + turn")]
def run(with_arm, seed=0, mount=None):
    m = ga.build(with_arm, STOW, mount=mount); d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, 0)
    pol = ga.LegPolicy(m); rng = np.random.default_rng(seed); out = []; fell = None
    for dur, cmd, name in SCRIPT:
        pol.cmd = np.array(cmd, np.float32); zs, tilt, vx, vy, wz = [], [], [], [], []
        for k in range(int(dur / 0.005)):
            if k % 4 == 0: pol.act(m, d)
            ga.step(m, d)
            R = d.xmat[m.body("trunk").id].reshape(3, 3); t = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
            zs.append(d.qpos[2]); tilt.append(t)
            if k * 0.005 > 1.0:
                v = R.T @ d.qvel[0:3]; vx.append(v[0]); vy.append(v[1]); wz.append(d.qvel[5])
            if fell is None and (d.qpos[2] < 0.10 or t > 50): fell = name   # a real fall: tipped over or trunk on the floor
        out.append((name, np.mean(zs), np.max(tilt), np.mean(vx) if vx else 0, np.mean(vy) if vy else 0, np.mean(wz) if wz else 0, cmd))
    return fell, out
if __name__ == "__main__":
  for arm in (False, True):
      fell, out = run(arm)
      print(f"== {'WITH arm stowed (3.38 kg)' if arm else 'no arm'}: {'FELL during ' + fell if fell else 'no fall'}")
      for name, z, tl, vx, vy, wz, cmd in out:
          print(f"   {name:18s} trunk height {z:.3f} m, max tilt {tl:4.1f} deg, speed x {vx:+.2f} (cmd {cmd[0]:+.1f}) y {vy:+.2f} (cmd {cmd[1]:+.1f}) turn {wz:+.2f} (cmd {cmd[2]:+.1f})")
