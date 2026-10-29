"""Live HUD for the MuJoCo viewer: battery/power graphs, state machine, wrist camera. Battery numbers come from
battery_model.Params (published figures + estimates) and the arm's simulated joint torques/speeds (RobStride Kt)."""
import collections
import numpy as np, mujoco
import battery_model as bm
KT = np.array([1.48, 1.22, 1.22, 1.48, 1.48, 1.48]); R_MID = 0.35          # ohm/phase, middle of the 0.15-0.6 estimate
STATES = ["SWEEP", "LOOK AGAIN", "SIDESTEP", "GO TO FIND", "LIE DOWN", "LOOK CLOSER: find and name", "GRASP", "INTO BIN", "STOW",
          "STAND UP", "DONE"]   # one forward pass, confirming finds on approach ([d8d], [QDc]); the older sweep-then-visit flow's
                                # states (DETECT, PLAN STOPS, WALK TO STOP) still show, appended, when a script names them
class Hud:
    def __init__(self, option="direct", cradle=False):
        self.p = bm.Params(); self.p.cradle = cradle; self.option = option; self.pay = (self.p.robot_kg + self.p.payload_kg) / self.p.robot_kg
        self.air_Wh = self.p.air_Wh; self.arm_Wh = self.p.arm_pack_Wh if option == "own" else None
        self.used_air = 0.0; self.used_arm = 0.0; self.state = "SWEEP"; self.note = ""; self.hist = collections.deque(maxlen=400)
        self.legs_W = self.arm_W = 0.0; self._acc = 0.0
    def extra_W(self):
        """the wrist's extra hardware on the 5 V side, by what is fitted and whether a close look is under way: the stereo
        pair's second camera all night, the Pi's work on the pair and the white light during a look ([A8K], [J7G])"""
        import go1_arm
        looking = self.state.startswith("LOOK CLOSER")
        w = self.p.stereo_camera_W if go1_arm.WRIST_STEREO else 0.0
        if looking and go1_arm.WRIST_STEREO: w += self.p.look_host_extra_W
        if looking and go1_arm.WRIST_LIGHT: w += self.p.wrist_light_W
        return w
    def tick(self, S, dt):
        d = S.d; walking = S.pol is not None and np.abs(S.pol.cmd).sum() > 0.01
        z = d.xpos[S.tr][2]; legs = self.p.walk_W * self.pay if walking else (self.p.stand_W if (S.pol is not None or z > 0.2) else self.p.lie_W)
        tau = d.actuator_force[12:18]; w = np.array([d.qvel[a] for a in S.ad]); i = np.abs(tau) / KT
        arm = self.p.arm_standby_W + np.maximum(tau * w, 0).sum() + (3 * i ** 2 * R_MID).sum()
        if self.p.cradle and walking: arm = 0.0
        if self.option == "boost": arm = arm / self.p.boost_eff + self.p.boost_q_W
        legs = legs + (self.p.host_W + self.extra_W()) / self.p.buck_eff        # the arm host (Pi) runs off the Air's pack too
        self.legs_W, self.arm_W = legs, arm
        if self.option == "own": self.used_air += legs * dt / 3600; self.used_arm += arm * dt / 3600
        else: self.used_air += (legs + arm) * dt / 3600
        self._acc += dt
        if self._acc >= 0.1: self._acc = 0; self.hist.append((S.t, legs, arm, 100 * (1 - self.used_air / self.air_Wh)))
    def figures(self, W, H):
        if __import__("os").environ.get("HK_POWER_CHART") != "1": return []   # off by default: the battery sim is
        f = mujoco.MjvFigure()                                                # trusted, the chart only for checking it
        mujoco.mjv_defaultFigure(f); f.title = "power, last 30 s (W)"; f.flg_legend = 1; f.gridsize = [4, 6]
        t_now = self.hist[-1][0] if self.hist else 0.0; f.range[0][:] = [t_now - 30, t_now]; f.range[1][:] = [0, 250]; f.flg_extend = 0
        f.xformat = "%.0f"; f.yformat = "%.0f"
        f.figurergba = [0.05, 0.05, 0.08, 0.75]; f.panergba = [0.05, 0.05, 0.08, 0.6]
        for k, (name, rgb) in enumerate((("legs", (0.3, 0.8, 1.0)), ("arm", (1.0, 0.55, 0.2)))):
            f.linename[k] = name; f.linergb[k] = rgb; pts = [h for h in self.hist if h[0] >= t_now - 30]; n = len(pts); f.linepnt[k] = n
            for j, h in enumerate(pts): f.linedata[k][2 * j] = h[0]; f.linedata[k][2 * j + 1] = h[1 + k]
        return [(mujoco.MjrRect(W - 420, H - 230, 410, 220), f)]          # top-right; battery text stays bottom-right
    def texts(self):
        lines = "\n".join(("> " if s == self.state else "  ") + s for s in STATES)
        if self.state not in STATES: lines += f"\n> {self.state}"                 # a state named by another loop (DimOS)
        opt = {"direct": "arm on Air pack (~30 V)", "boost": "arm via boost to 48 V", "own": "arm on own 12S pack"}[self.option]
        batt = f"Air pack {100 * (1 - self.used_air / self.air_Wh):5.1f}%  ({self.used_air:5.2f} of {self.air_Wh:.0f} Wh)\n"
        if self.arm_Wh: batt += f"Arm pack {100 * (1 - self.used_arm / self.arm_Wh):5.1f}%  ({self.used_arm:5.2f} of {self.arm_Wh:.0f} Wh)\n"
        batt += f"legs {self.legs_W:5.0f} W   arm {self.arm_W:5.1f} W\n{opt}{' | cradle' if self.p.cradle else ''}"
        return [(mujoco.mjtFontScale.mjFONTSCALE_50, mujoco.mjtGridPos.mjGRID_TOPLEFT, "STATE\n" + lines, ""),
                (mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT, batt, ""),
                (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, self.note, "")]
