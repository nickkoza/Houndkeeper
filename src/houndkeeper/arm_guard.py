"""The arm's one gate ([Bhf]): every motion command to the arm goes through GuardedArm, which refuses it unless the base
is still ([G48]), the robot is down in its picking posture ([FWp]) and the target is inside the tip-safety envelope for
the weight held ([qpO], [uMO]). It judges from the robot's own state only: its odometry, the posture it last took, the
arm's joint readings and the mass it weighed in the jaws. Never sim truth."""
from __future__ import annotations

import json
import math
import sys
from collections import deque
from pathlib import Path
from typing import Any, Callable

import numpy as np

HK = Path(__file__).resolve().parents[2]
ENVELOPE = HK / "sim" / "arm" / "tip_envelope.json"   # sim-tip-safety [YGD]: written by `sim/suite/run tip`

STOW = np.zeros(6)            # the arm's factory fold, where it is carried ([G48]; tidy_module folds to it)
STOW_TOL_RAD = 0.10           # every joint this close to STOW reads as stowed (the arm's tracking error is a few hundredths)
STAND_LOOK_RAD = 0.23         # standing, stopped: every joint within this of STOW, empty jaws, the region whose balance was
                              # measured for the standing look (sim-tip-safety [BuG]; sim-slice [QDc]); nothing else standing
ARM_VMAX = 1.0                # rad/s: the arm's global limit (vslice.Sim.ARM_VMAX), where the envelope measured no safe speed
SPEED_TOL = 0.15              # a streamed step may run this much over the limit (the cosine profile's sampling, clock jitter)
STEP_DT_MAX = 0.1             # s: a step after a pause is judged as if 0.1 s passed: a jump is a jump however long the wait
STILL_M_S, STILL_RAD_S = 0.05, 0.10   # the base reads still under these (odometry over STILL_WINDOW_S); the slowest walk
                                      # the tidy asks for is 0.1 m/s and its slowest turn 0.6 rad/s
RATE_STEP_S = 0.1                     # rates over steps this long: odometry's own jitter averaged out
STILL_WINDOW_S, ODOM_STALE_S = 0.5, 1.0   # odometry older than this is no reading: the base is not known to be still


class ArmRefused(RuntimeError):
    """an arm command the guard refused, with why"""


class BaseWatch:
    """whether the base is still, from the robot's odometry alone"""

    def __init__(self):
        self._odom: deque = deque(maxlen=200)

    def on_odom(self, t: float, x: float, y: float, yaw: float) -> None:
        self._odom.append((float(t), float(x), float(y), float(yaw)))

    def moving(self, now: float) -> str | None:
        """why the base is not known to be still, or None"""
        if not self._odom: return "no odometry"
        if now - self._odom[-1][0] > ODOM_STALE_S: return f"odometry {now - self._odom[-1][0]:.1f} s old"
        win = [o for o in self._odom if o[0] >= self._odom[-1][0] - STILL_WINDOW_S]
        span = win[-1][0] - win[0][0]
        if span < STILL_WINDOW_S / 2: return "too little odometry to judge the base still"
        v = w = 0.0                                   # the fastest over any RATE_STEP_S in the window, not end to end:
        j = 0                                         # a gait swaying back and forth reads still between its ends
        for i, a in enumerate(win):
            j = max(j, i + 1)
            while j < len(win) and win[j][0] - a[0] < RATE_STEP_S: j += 1
            if j == len(win): break
            b = win[j]; dt = b[0] - a[0]
            v = max(v, math.hypot(b[1] - a[1], b[2] - a[2]) / dt)
            w = max(w, abs((b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi) / dt)
        if v > STILL_M_S or w > STILL_RAD_S: return f"the base is moving ({v:.2f} m/s, {w:.2f} rad/s)"
        return None


class Envelope:
    """the tip-safety envelope (sim-tip-safety [YGD]): safe reach and safe joint speed per posture and payload. A
    payload is judged by the lightest one measured at or above it; none at or above it, or no file, is no envelope"""

    def __init__(self, path: Path | str | None = ENVELOPE):
        p = Path(path) if path is not None else None
        self.data: dict | None = json.loads(p.read_text()) if p is not None and p.exists() else None

    def _bracket(self, table: dict, posture: str, kg: float) -> tuple[float | None, bool]:
        """(the value for the lightest measured payload >= kg, whether there was one)"""
        rows = sorted((float(k.rsplit("_", 1)[1]), v) for k, v in table.items() if k.rsplit("_", 1)[0] == posture)
        for pay, v in rows:
            if pay >= kg - 1e-9: return v, True
        return None, False

    def reach_m(self, posture: str, kg: float) -> float | None:
        if self.data is None: return None
        return self._bracket(self.data.get("safe_reach_m", {}), posture, kg)[0]

    def speed(self, posture: str, kg: float) -> tuple[float | None, str]:
        """(rad/s limit or None for no move at all, where it came from)"""
        if self.data is None: return ARM_VMAX, "no tip-safety envelope: the stow only"
        v, found = self._bracket(self.data.get("safe_speed", {}), posture, kg)
        if not found: return None, f"no measured speed {posture} at {kg:.2f} kg or more"
        if v is None: return ARM_VMAX, f"no measured safe speed {posture} at {kg:.2f} kg: the arm's global {ARM_VMAX} rad/s"
        return min(float(v), ARM_VMAX), "the envelope"


class ArmKinematics:
    """the grasp point's reach from the arm's mount, from the robot's own arm model (the one DimosRobot plans with)"""

    def __init__(self):
        sys.path.insert(0, str(HK / "sim" / "arm"))
        import mujoco
        import go1_arm as ga
        import payload_test as pt
        self._mj = mujoco; self.m = ga.build(True, pt.STOW); self.d = mujoco.MjData(self.m)
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        self.aq = [self.m.jnt_qposadr[self.m.joint("arm_" + j).id] for j in ga.FF_JOINTS]
        self.tr, self.base, self.gc = (self.m.body(n).id for n in ("trunk", "arm_base_link", "arm_grasp_center"))

    def reach_m(self, q6) -> float:
        """the grasp point's distance from the mount across the trunk's floor plane (the envelope's r)"""
        self.d.qpos[self.aq] = np.asarray(q6, float)[:6]; self._mj.mj_kinematics(self.m, self.d)
        R = self.d.xmat[self.tr].reshape(3, 3); rel = R.T @ (self.d.xpos[self.gc] - self.d.xpos[self.base])
        return float(math.hypot(rel[0], rel[1]))


class ArmGuard:
    """the rules; GuardedArm applies them to every command, GuardedPosture keeps the posture it judges by"""

    def __init__(self, clock: Callable[[], float], envelope: Envelope | None = None, reach: Callable | None = None,
                 posture: str | None = None, read_q: Callable[[], Any] | None = None):
        self.clock = clock; self.base = BaseWatch(); self.env = envelope if envelope is not None else Envelope()
        self._reach = reach; self.posture = posture; self.carried_kg = 0.0; self.read_q = read_q
        self._last: tuple[float, np.ndarray] | None = None
        self.refusals: list[str] = []; self.notes: set[str] = set()

    def load_kinematics(self) -> None:
        """the arm model the reach is judged with, built now: built at the first lying check it stalled the process for
        seconds, and odometry piled up behind it (18 Oct)"""
        if self._reach is None: self._reach = ArmKinematics().reach_m

    def reach_m(self, q6) -> float:
        self.load_kinematics(); return self._reach(q6)

    def on_odom(self, t: float, x: float, y: float, yaw: float) -> None: self.base.on_odom(t, x, y, yaw)

    def stowed(self, q=None) -> bool:
        q = self.read_q() if q is None and self.read_q is not None else q
        return q is not None and bool(np.all(np.abs(np.asarray(q, float)[:6] - STOW) <= STOW_TOL_RAD))

    def vmax(self) -> float | None:
        if self.posture == "stand": return ARM_VMAX               # [BuG]: the standing look's poses measured at 1 rad/s
        v, why = self.env.speed(self.posture or "", self.carried_kg)
        if why != "the envelope": self.notes.add(why)
        return v

    def refuses(self, q, check_base: bool = True) -> str | None:
        """why the arm may not be at pose q now, or None: still ([G48]), down ([FWp]), inside the envelope ([qpO])"""
        q = np.asarray(q, float)[:6]
        if check_base and (why := self.base.moving(self.clock())): return f"{why} ([G48], [FWp])"
        if self.posture in ("lie", "crouch"):
            r_max = self.env.reach_m(self.posture, self.carried_kg)
            if r_max is None:
                if self.stowed(q): return None                   # sim-tip-safety: no envelope, the stow only
                return f"no tip-safety envelope for {self.posture} at {self.carried_kg:.2f} kg ([qpO])"
            r = self.reach_m(q)
            return None if r <= r_max + 1e-6 else f"reach {r:.2f} m past the envelope's {r_max:.2f} m ({self.posture}, {self.carried_kg:.2f} kg, [qpO])"
        if self.posture == "stand":                              # the standing look's measured region only ([FWp], [BuG])
            if np.all(np.abs(q - STOW) <= STAND_LOOK_RAD) and self.carried_kg <= 0.0: return None
            return "standing: only the standing look's measured poses, empty ([FWp], [BuG])"
        return f"posture {self.posture or 'unknown'}: the arm moves only lying or crouched ([FWp])"

    def check(self, target) -> None:
        """a position command: refused (ArmRefused) unless allowed now, at its pace; a hold of the last command passes"""
        q = np.asarray(target, float); now = self.clock()
        if self._last is not None and np.array_equal(q, self._last[1]): return
        why = self.refuses(q)
        if why is None:
            ref = self._last[1] if self._last is not None else (np.asarray(self.read_q(), float) if self.read_q else None)
            dt = min(max(now - self._last[0], 0.0), STEP_DT_MAX) if self._last is not None else STEP_DT_MAX
            v = self.vmax()
            if v is None: why = f"no safe arm speed known ({self.posture}, {self.carried_kg:.2f} kg, [qpO])"
            elif ref is not None and (step := float(np.max(np.abs(q[:6] - ref[:6])))) > v * dt * (1 + SPEED_TOL) + 1e-3:
                why = f"{step / max(dt, 1e-3):.2f} rad/s past the safe {v:.2f} ([qpO])"
        if why is not None:
            self.refusals.append(why); raise ArmRefused(why)
        self._last = (now, q.copy())

    def check_path(self, qs) -> None:
        """a whole move's poses, before any is sent: refused whole, never halfway"""
        for q in qs:
            if (why := self.refuses(q)) is not None:
                self.refusals.append(why); raise ArmRefused(why)

    def base_alarm(self) -> str | None:
        """the base moving with the arm out (arm-on-board's failure behaviour: the base stops)"""
        if self.read_q is None or self.posture in ("lie", "crouch") or self.stowed(): return None   # it walks only standing
        why = self.base.moving(self.clock())
        return why if why and why.startswith("the base is moving") else None


class GuardedArm:
    """a ManipulatorAdapter whose every motion command passes the guard. Reads pass straight through; of the writes, only
    positions (checked), stop, enable and clearing errors exist: velocity, effort and cartesian commands are refused"""

    PASS_WRITES = ("write_stop", "write_enable", "write_clear_errors")

    def __init__(self, adapter, guard: ArmGuard):
        self._inner = adapter; self.guard = guard
        if guard.read_q is None: guard.read_q = adapter.read_joint_positions
        guard.load_kinematics()                       # with the arm, before any command: no check waits on it

    def write_joint_positions(self, positions, *a, **k) -> bool:
        self.guard.check(positions); return self._inner.write_joint_positions(positions, *a, **k)

    def set_payload(self, kg: float) -> None:
        """the mass in the jaws as the robot weighed it: the envelope's payload; passed on where the adapter has one"""
        self.guard.carried_kg = max(0.0, float(kg))
        if hasattr(self._inner, "set_payload"): self._inner.set_payload(kg)

    def __getattr__(self, name):
        if name.startswith("write_") and name not in self.PASS_WRITES:
            def refused(*a, **k):
                why = f"{name} bypasses the arm guard's checks"; self.guard.refusals.append(why); raise ArmRefused(why)
            return refused
        return getattr(self._inner, name)


class GuardedPosture:
    """the robot's posture commands, kept by the guard: a change only with the arm stowed and the base still, and the
    arm held while it happens"""

    def __init__(self, posture, guard: ArmGuard):
        self._inner = posture; self.guard = guard

    def _to(self, name: str, call) -> bool:
        if not self.guard.stowed():
            why = f"{name} with the arm out ([G48])"; self.guard.refusals.append(why); raise ArmRefused(why)
        self.guard.posture = "changing"
        ok = bool(call())
        self.guard.posture = name if ok else None
        return ok

    def liedown(self) -> bool: return self._to("lie", self._inner.liedown)
    def standup(self) -> bool: return self._to("stand", self._inner.standup)
    def crouch(self) -> bool: return self._to("crouch", self._inner.crouch)

    def __getattr__(self, name): return getattr(self._inner, name)
