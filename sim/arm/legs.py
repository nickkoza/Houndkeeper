"""Leg postures for the sim Go1 by leg PD targets (the walking policy is off while posing): lie, crouch, stand.
Transitions interpolate targets; stand-up hands back to the walking policy. Heights are sim-Go1, not Go2 Air."""
import numpy as np, mujoco
import go1_arm                                   # the arm's gravity feed-forward, once per control step ([cqG])
def pose(th, ca, hip=0.0): return np.array([hip, th, ca, -hip, th, ca, hip, th, ca, -hip, th, ca])
STAND = pose(0.9, -1.8, 0.1); CROUCH = pose(1.1, -2.3, 0.1); LIE = pose(1.2, -2.6, 0.1)
def start_standing(m, d, z=0.30, at=None):
    """the dog standing, settled on leg PD; at (x, y, yaw): where, else the world origin. It settles where it will start
    ([rUS]: settled at the origin and moved after, in a furnished house it stood on a bench and fell, 30 Sep)"""
    mujoco.mj_resetDataKeyframe(m, d, 0); d.qpos[7:19] = STAND; d.qpos[2] = z; d.qvel[:] = 0; d.ctrl[:12] = STAND
    if at is not None:
        import math
        d.qpos[0:2] = at[:2]; d.qpos[3:7] = [math.cos(at[2] / 2), 0, 0, math.sin(at[2] / 2)]
    for _ in range(int(round(2.0 / m.opt.timestep))): mujoco.mj_step(m, d)   # settle 2 s on leg PD, whatever the step
def move_legs(m, d, target, secs, step_cb=None, arm_ctrl=None):
    start = d.ctrl[:12].copy(); n = int(secs / 0.005); sub = max(1, int(round(0.005 / m.opt.timestep)))   # 5 ms control
    for k in range(n):                                                                                    # steps, each of
        a = 0.5 - 0.5 * np.cos(np.pi * min(1, k / (0.8 * n))); d.ctrl[:12] = (1 - a) * start + a * target   # sub physics steps
        if arm_ctrl is not None: d.ctrl[12:12 + len(arm_ctrl)] = arm_ctrl
        go1_arm.update_gravity_ff(m, d)                                                                   # [cqG]
        for _ in range(sub): mujoco.mj_step(m, d)
        if step_cb: step_cb(m, d)
