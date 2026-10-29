"""Houndkeeper's tidy loop as a DimOS module ([YWo], first version).

Skills (MCP tools):
  tidy_start()  finds items, then for each: park beside it with DimOS navigation, lie down, look closer with the wrist
                camera, pick it into the bin (or leave it for review, with the reason), stow, stand up. Runs in the
                background; returns at once.
  tidy_status() progress and the report so far.

Finding items (phase 1) in this first version: the head camera, turning in place, with the same open-vocabulary
detector look-closer uses, projected onto the floor from the camera's pose. The nightly floor comparison
([slD], house nights) replaces it once it runs as a DimOS module on this scene.

Everything goes through DimOS: navigation (NavigationInterfaceSpec), postures (the Go2 connection's liedown/standup),
the arm (DimOS's sim manipulator adapter, the same interface the real Maker Arm adapter will implement), the head and arm
camera streams, odometry. Numbers are sim on the Go1 stand-in ([WD6])."""
from __future__ import annotations

import json
import math
import os
import sys
import threading
from pathlib import Path
from typing import Any, Protocol

import numpy as np
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.base import NavigationState
from dimos.navigation.go2.replanning_a_star.spec import NavigationInterfaceSpec
from dimos.spec.utils import Spec
from dimos.utils import clock   # the robot's clock: sim time in the sim ([3Pv])
from dimos.utils.logging_config import setup_logger
from houndkeeper import arm_guard

logger = setup_logger()
HK = Path(__file__).resolve().parents[2]
def _park_offset():
    """where an item should sit in the robot's frame when it lies down: in front, where the arm can grip it from the most
    directions on average over the parking scatter, for the grasp's allowed lean ([S7o], [Hwc]); from the reach maps,
    never a constant (cluster_stops.front_offset)"""
    sys.path.insert(0, str(HK / "sim" / "arm")); import cluster_stops, pick
    return cluster_stops.front_offset(pick.MAX_LEAN)


def _lie_settle():
    sys.path.insert(0, str(HK / "sim" / "arm")); import cluster_stops
    return cluster_stops.LIE_SETTLE_M


def near_views(run_dir, prev) -> tuple[list[str] | None, list[int]]:
    """[BW3]: the baseline nights' views on the near grid (views_near_n<k>.npz, painted with every stitch: near_views.sh), each of
    the nights the floor is judged against, as the replay's --near-views; and the nights that have none"""
    run = Path(run_dir); have = [k for k in prev if (run / f"views_near_n{k}.npz").exists()]
    return [str(run / f"views_near_n{k}.npz") for k in have] or None, [k for k in prev if k not in have]


class PostureSpec(Spec, Protocol):
    def liedown(self) -> bool: ...
    def standup(self) -> bool: ...


def repark_target(ref) -> np.ndarray:
    """where a re-park puts in the arm's best reach after no grip direction was reachable (sim-grasp [Q8C]): the grip
    the pick tries first (the item's position plus that grip's offset from its middle), not the middle, which for a
    long toy can be 10 cm or more from any grip; the item's position when it has no grip options"""
    xy = np.asarray(ref["xy"], float)
    opts = [o for o in (ref.get("grip_options") or []) if o[0] >= 0.005]   # (a sliver of the mask is no grip)
    return xy + np.asarray(opts[0][2], float)[:2] if opts else xy



ARM_STEPS = ("the look from standing", "the close look", "the pick", "the bin")   # [jFy]: what each stop does with the arm
LAG_FILE = HK / "data" / "robot" / "frame_lag.json"   # the robot's frame lag ([gJq]; sim/house/calibrate_lag.py writes it)


def live_clock(env=None):
    """the clock the live robot's messages are stamped on ([gJq], decision [55G]): the robot's clock. In the sim it
    follows sim time when DIMOS_SIM_CLOCK names it ([3Pv]); a sim run without it stamps on the wall clock. The real
    robot's own clock is the wall clock, and is the robot clock (its lag measured on day one, real/calibration/timing.md)"""
    env = os.environ if env is None else env
    if env.get("DIMOS_SIM_CLOCK"): return "robot"
    return "wall" if any(k.startswith("DIMOS_SIM_") for k in env) else "robot"


def robot_frame_lag(baseline_lags, lag_file=None, override=None, clock=None):
    """the head camera's frame lag for the live tidy ([gJq]): the robot's stored one for the live clock ([55G]; a file
    with none for it is refused); the baselines' pictures' lags only check it. override (HK_LAG=night): the baselines'
    median stands. Returns (lag s, what it is)"""
    from houndkeeper import calibrate as _cal
    override = os.environ.get("HK_LAG", "") if override is None else override
    night = float(np.median(baseline_lags)) if baseline_lags else None
    if override == "night" and night is not None: return night, "the earlier nights' measured lag, as asked"
    lf = Path(lag_file or LAG_FILE)
    if not lf.exists(): return (night or 0.0), "no robot lag stored: the earlier nights' measured lag"
    d = _cal.load_lag(lf, clock or live_clock()); lag = _cal.floor_lag(d, "color_image")   # the floor rows' ([yFg])
    note = f"the robot's lag, {d['clock']} clock, {_cal.file_sha(lf)}" + (", the floor rows'" if lag != float(d["lag_s"]) else "")
    if night is not None and abs(night - lag) > _cal.LAG_CHECK_S:
        note += f"; the baselines' pictures used {night:.3f} s, {abs(night - lag) * 1000:.0f} ms off"
    return lag, note


def baseline_calibration(run_dir, prev, say=print, lag_file=None):
    """the floor relief the live tidy lays tonight on (the house's own, [yFg]; the plane if it has none, said), after
    checking the earlier nights' pictures and views were painted with tonight's calibration (the run's mount, the
    robot's lag, that relief): one painted under another is refused, the night stopped and why said ([Q9k]); so is a
    house map made by an older stitch than the current one ([dmT])"""
    from houndkeeper import calibrate as _cal, floor_relief as _frl
    run = Path(run_dir); relief, rpath, note = _frl.resolve("auto", run, log=lambda m: None); say(f"sweep: floor {note}")
    mp = run / "head_mount.json"
    want = _cal.stamp(mp if mp.exists() else None, lag_file or LAG_FILE, rpath)
    why = _cal.stale_products(run, want, [f"{k}_n{n}.npz" for n in prev for k in ("mosaic", "views")])
    if why: raise RuntimeError("the earlier nights' floor was painted under another calibration than tonight's; "
                               "the house's floor map must be made again: " + "; ".join(why))
    why = _cal.stale_map(run)                                  # a map made before its loops were verified ([dmT])
    if why: raise RuntimeError("the house's floor map must be made again: " + "; ".join(why))
    return relief


class AreaStop(Exception):
    """the run stops where it stands and says why ([Qd5]): localization lost, or outside the area with no way back"""


class ArmStop(Exception):
    """the arm could not be stowed: the run stops where it stands, the arm held where it is ([WkC])"""


class TidyArm(Module):
    odom: In[PoseStamped]
    color_image: In[Image]
    arm_color_image: In[Image]
    cmd_vel: Out[Twist]
    tf: In[TFMessage]          # DimOS's transform tree: the head camera's pose when each frame was taken
    keepout: Out[PointCloud2]  # floor discs (x, y, radius in z) the costmap marks lethal: items too low for it to see
    global_map: In[PointCloud2]  # DimOS's live 3D map: tonight's tall things (a moved chair) block the camera's view of the floor
    lidar: In[PointCloud2]     # each scan, matched to the house's lidar map to place the robot in the house's frame ([KkQ])
    mind_json: Out[String]     # what it thinks, for the phone ([ULM]): JSON text, the form --record keeps ([Fho])
    keepout_zones: Out[String]  # the operating area, placed in the odometry's frame, for DimOS's costmap layer ([JVr])

    _navigation: NavigationInterfaceSpec
    _posture: PostureSpec

    @rpc
    def start(self) -> None:
        super().start()
        self._odom = None; self._head = None; self._wrist = None
        self._guard = arm_guard.ArmGuard(clock.now, posture="stand")   # the arm's one gate ([G48], [FWp], [qpO]); the night
                                                                       # starts standing
        self._status: dict[str, Any] = {"state": "idle", "log": [], "items": []}
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.color_image.subscribe(lambda im: setattr(self, "_head", im))))
        self.register_disposable(Disposable(self.arm_color_image.subscribe(lambda im: setattr(self, "_wrist", im))))
        self._live_map = None
        self.register_disposable(Disposable(self.global_map.subscribe(lambda m: setattr(self, "_live_map", m))))
        self._live_loc = None
        self.register_disposable(Disposable(self.lidar.subscribe(self._on_lidar)))
        self._mind_stop = threading.Event()
        if self.MIND: threading.Thread(target=self._mind_loop, name="mind", daemon=True).start()
        self._odom_t = None; self._area_pub = None
        if os.environ.get("HK_AREA"):                            # the operating area ([JVr]): published from the start, so
            from houndkeeper.operating_area_module import AreaPublisher   # the costmap has it before the first goal
            self._area_pub = AreaPublisher(os.environ["HK_AREA"], lambda j: self.keepout_zones.publish(String(j)),
                                           self._house_correction)
            threading.Thread(target=self._area_loop, name="operating-area", daemon=True).start()

    @rpc
    def stop(self) -> None:
        ev = getattr(self, "_mind_stop", None)
        if ev is not None: ev.set()
        super().stop()

    # --- what it thinks, for the phone's Mind page ([RNC]) ---
    MIND = os.environ.get("HK_MIND") == "1"   # publish the mind summary ([ULM]); off unless asked for

    def _mind_loop(self) -> None:
        """the mind summary every mind.PERIOD_S, from its own thread: the loop blocks for seconds at a time (a lie
        down, an arm move), and the summary only reads its state"""
        from houndkeeper import mind
        m = mind.Mind()
        while not self._mind_stop.wait(mind.PERIOD_S):
            try: self.mind_json.publish(String(m.json(self._mind_state(), now=clock.now())))
            except Exception as e:                               # (the loop's state changing under the read): next tick
                if not getattr(self, "_mind_warned", False): logger.warning(f"[tidy] mind summary skipped: {e!r}"); self._mind_warned = True

    def _mind_state(self) -> dict:
        """the hook ([ULM]): the night's own state, by reference, for mind.summary; nothing computed here"""
        o = self._odom
        return dict(status=self._status, sweep=getattr(self, "_sweep_log", None), goal=getattr(self, "_goal", None),
                    route=getattr(self, "_route", None), route_at=getattr(self, "_route_at", 0), phase=getattr(self, "_phase", None),
                    note=getattr(self, "_note", ""), view=getattr(self, "_view", None), worth=self._worth_a_stop,
                    sweep_arm=bool(self.SWEEP_ARM), pose=None if o is None else (
                        o.position.x, o.position.y, o.orientation.w, o.orientation.x, o.orientation.y, o.orientation.z))

    def _aim(self, xy, why: str, kind: str, wp: int | None = None) -> None:
        """where it is going now and why, in the words of the code that chose it ([1XG]); wp: the sweep waypoint's index"""
        self._goal = {"x": float(xy[0]), "y": float(xy[1]), "kind": kind, "why": why}
        if wp is not None: self._route_at = wp

    def _on_lidar(self, msg: PointCloud2) -> None:
        loc = self._live_loc
        if loc is None: return
        p = msg.as_numpy(); loc.scan(getattr(msg, "ts", None) or clock.now(), np.asarray(p[0] if isinstance(p, tuple) else p))

    def _on_odom(self, msg: PoseStamped) -> None:
        self._odom = msg; self._odom_t = clock.now()
        g = getattr(self, "_guard", None)
        if g is not None:                                        # the arm guard judges the base still from odometry alone,
            # each reading at its own stamp (the robot's clock, as the connection received it), not when this module got
            # it: a stall here (the guard's kinematics loading, 18 Oct) delivered a burst that read as "too little odometry";
            # never later than it arrived, so a stamp ahead of the clock cannot hide a stream gone stale
            q_ = msg.orientation; g.on_odom(min(getattr(msg, "ts", None) or self._odom_t, self._odom_t), msg.position.x, msg.position.y,
                                            math.atan2(2 * (q_.w * q_.z + q_.x * q_.y), 1 - 2 * (q_.y * q_.y + q_.z * q_.z)))
            if (why := g.base_alarm()) is not None: self._base_alarm(why)
        if self._live_loc is not None:
            q_ = msg.orientation; yaw_ = math.atan2(2 * (q_.w * q_.z + q_.x * q_.y), 1 - 2 * (q_.y * q_.y + q_.z * q_.z))
            self._live_loc.odom((msg.position.x, msg.position.y), yaw=yaw_, t=getattr(msg, "ts", None) or clock.now())
        q = msg.orientation; b = getattr(self, "_balance", None)       # the trunk's tilt from vertical, every odometry message
        if b is not None:
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
            b["n"] += 1; b["max_deg"] = max(b["max_deg"], tilt); b["min_z"] = min(b["min_z"], float(msg.position.z))
            b["over5"] += tilt > 5.0; b["over10"] += tilt > 10.0
            b["fell"] = b["fell"] or tilt > self.FELL_TILT_DEG or msg.position.z < self.LIE_Z_M - self.FELL_BELOW_LIE_M

    FELL_TILT_DEG = 45.0        # a fall: the trunk tipped this far, or dropped this far below its lying height. Lying reads
    LIE_Z_M = 0.11              # 0.111-0.126 m on odometry with 11 deg of tilt at most (sim pickup nights, 18 Oct: a height
    FELL_BELOW_LIE_M = 0.03     # floor of 0.12 m called every lying stop a fall); legs folded under, the trunk can drop no
                                # lower unless they give way. The real robot's lying height to be read on first boot

    # --- skills ---
    @skill
    def tidy_start(self, flags: str = "", sweep: str = "") -> str:
        """Pick up each find into the bin on the robot's back; finds the arm can't take are left for review with the
        reason. flags: the sweep's finds as a JSON list of [x, y] floor positions (metres, map frame), from comparing
        tonight's floor with last night's ([2w8]); without them the robot looks around from where it stands with the
        head camera (a stand-in, to be removed). sweep: instead of flags, JSON {"route": [[x, y], ...], "run_dir": the
        earlier nights' floor pictures, "night": tonight's number}: walk the route and confirm finds on approach, stopping
        for each as it's confirmed ([d8d], [QDc]). Runs in the background; use tidy_status to follow it."""
        if self._status["state"] not in ("idle", "done", "failed", "stopped"):
            return f"already running: {self._status['state']}"
        if self._on_real_robot() and self._area() is None:      # [Qd5]: never a real run without its boundary
            return ("refused: no operating area is set. Draw it on the cockpit's Area page (or set HK_AREA to its file) "
                    "before starting a run on the robot.")
        self._given_flags = [np.array(f[:2], float) for f in json.loads(flags)] if flags else None
        self._given_sweep = json.loads(sweep) if sweep else None
        self._status = {"state": "starting", "log": [], "items": [], "t0": clock.now()}
        threading.Thread(target=self._run, daemon=True).start()
        return "tidy started"

    @skill
    def tidy_status(self) -> str:
        """Progress of the tidy run: state, log and a report per item."""
        s = dict(self._status); s["log"] = s["log"][-15:]
        return json.dumps(s, default=str)

    # --- the loop ---
    def _say(self, text: str) -> None:
        logger.info(f"[tidy] {text}"); self._status["log"].append(f"{clock.now() - self._status.get('t0', clock.now()):6.1f}s {text}")
        self._status["phase"] = text[7:] if text.startswith("state: ") else None   # the pick's phase while picking, else the loop's
        if text.startswith("state: "): self._phase = text[7:]                   # (kept till the loop's next state: the mind view)
        self._publish(text)

    def _publish(self, note: str = "") -> None:
        """the state and latest step for the HUD on DimOS's viewer (sim_hook.viewer, another process) ([5AP])"""
        from houndkeeper.hud_state import HUD_FILE
        self._note = note
        try:
            with open(HUD_FILE + ".tmp", "w") as fh:
                json.dump({"state": (self._status.get("phase") or self._status["state"]).upper(), "note": note}, fh)
            os.replace(HUD_FILE + ".tmp", HUD_FILE)
        except OSError:
            pass

    def _pose(self) -> tuple[np.ndarray, np.ndarray] | None:
        o = self._odom
        if o is None: return None
        p = np.array([o.position.x, o.position.y, o.position.z]); q = o.orientation
        return p, np.array([q.w, q.x, q.y, q.z])

    def _yaw(self) -> float:
        _, q = self._pose(); w, x, y, z = q
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def _go(self, x: float, y: float, yaw: float, timeout: float = 90.0, guard: bool = True) -> str:
        if guard: self._area_check()                              # [Qd5]: inside, and localized, before every goal
        goal = PoseStamped(position=Vector3(x, y, 0.0), orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)), frame_id="world")
        self._navigation.set_goal(goal); clock.sleep(1.0); t0 = clock.now(); idle = None
        while clock.now() - t0 < timeout:
            if self._navigation.is_goal_reached(): return "reached"
            if self._navigation.get_state() == NavigationState.FOLLOWING_PATH: idle = None
            elif idle is None: idle = clock.now()
            elif clock.now() - idle > 3.0: return "gave up"
            clock.sleep(0.2)
        return "timed out"

    def _in_robot(self, f: np.ndarray) -> np.ndarray:
        p, _ = self._pose(); y = self._yaw(); c, s = math.cos(y), math.sin(y); d = f - p[:2]
        return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]])

    def _approach(self, f: np.ndarray, offset: np.ndarray, clear: float = 0.40, walk: bool = True) -> tuple[str, float]:
        """Park so the item sits at `offset` in the robot's frame without any foot passing over it: walk (planner) to a
        spot that keeps the item `clear` m to the side, turn to the final heading, then side-step in under odometry.
        The lying reach is a thin band beside the body, so parking must be within a few cm."""
        p, _ = self._pose(); side = 1.0 if offset[1] >= 0 else -1.0
        heading = math.atan2(f[1] - p[1], f[0] - p[0]) - side * math.pi / 2       # face across the line to the item: it is on our side
        c, s = math.cos(heading), math.sin(heading); pre_off = np.array([offset[0], side * clear])
        pre = f - np.array([c * pre_off[0] - s * pre_off[1], s * pre_off[0] + c * pre_off[1]])
        nav = "shuffle only"
        if walk:                                                                    # (a re-park is only the final shuffle)
            nav = self._go(float(pre[0]), float(pre[1]), heading)
            self._turn(((heading - self._yaw() + math.pi) % (2 * math.pi)) - math.pi)
        for axis in (0, 1, 0, 1):                                                   # shuffle in: forward/back, then sideways, twice
            t0 = clock.now(); best = (9.0, clock.now())
            while clock.now() - t0 < 20.0:                                          # a fixed speed the walking policy tracks (it ignores
                e = -(self._in_robot(f) - offset)[axis]                             # very slow commands), stopped on odometry
                if abs(e) < 0.03: break
                if abs(e) < best[0] - 0.01: best = (abs(e), clock.now())
                elif clock.now() - best[1] > 3.0: break                             # no progress for 3 s: blocked; don't push on
                v = [0.0, 0.0]; v[axis] = -math.copysign(0.25, e)
                self.cmd_vel.publish(Twist(linear=Vector3(v[0], v[1], 0.0), angular=Vector3(0.0, 0.0, 0.0))); clock.sleep(0.05)
            self.cmd_vel.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))); clock.sleep(0.6)
        return nav, round(float(np.linalg.norm(self._in_robot(f) - offset)) * 100, 1)

    def _turn(self, angle: float, rate: float = 0.6) -> None:
        y0 = self._yaw(); t0 = clock.now()
        while clock.now() - t0 < abs(angle) / rate * 2.5:
            done = (self._yaw() - y0 + math.pi) % (2 * math.pi) - math.pi
            if abs(done) >= abs(angle) - 0.05: break
            self.cmd_vel.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, math.copysign(rate, angle))))
            clock.sleep(0.1)
        self.cmd_vel.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))

    def _run(self) -> None:
        try:
            self._tidy_loop()
            self._status["state"] = "done"
        except AreaStop as e:                                    # [Qd5]: stopped where it stood, and said why
            self._stop(); self._status["state"] = "stopped"; self._status["stopped"] = str(e); self._say(f"stopped: {e}")
        except (ArmStop, arm_guard.ArmRefused) as e:             # [WkC]: the arm out, never walked with: stopped where it
            self._stop(); self._status["state"] = "stopped"; self._status["stopped"] = str(e)   # stood, filed for review
            g = getattr(self, "_guard", None); q = g.read_q() if g is not None and g.read_q is not None else None
            self._status["arm_out"] = {"why": str(e), "q": None if q is None else [round(float(x), 3) for x in q],
                                       "posture": None if g is None else g.posture, "refusals": [] if g is None else g.refusals[-5:]}
            if self._status.get("items") and not self._status["items"][-1].get("outcome"):
                self._status["items"][-1]["outcome"] = f"review: stopped with the arm out ({e})"
            self._say(f"stopped: {e}")
        except Exception as e:                                   # a failed night is reported, never silent
            logger.exception("tidy failed"); self._status["state"] = "failed"; self._status["error"] = repr(e)

    NO_ARM = os.environ.get("HK_ARM") == "none"   # [jFy]: the dog alone, before the arm arrives: the rounds walked and
                                                  # watched only, every step that needs the arm skipped and reported

    def _walk_without_arm(self, sweep) -> None:
        """[jFy]: no arm: the sweep walked watching only (no stop: each stop's looks and pick need the arm), the steps
        skipped named in the report, so the morning log can say so"""
        self._status["arm"] = "none"; self._status["arm_skipped"] = list(ARM_STEPS)
        if sweep is None: raise RuntimeError("no arm: only a sweep can run (a route to walk), not finds to visit")
        self.WATCH_ONLY = True; self._ctx = {"handled": []}
        self._say(f"no arm: walking the rounds and marking what I see; skipped: {', '.join(ARM_STEPS)}")
        self._sweep_and_pick(sweep)

    def _tidy_loop(self) -> None:
        while self._odom is None or self._head is None: clock.sleep(0.2)
        self._area_check()                                        # [Qd5]: back inside before anything else
        sweep = getattr(self, "_given_sweep", None)
        if self.NO_ARM: return self._walk_without_arm(sweep)     # [jFy]: nothing of the arm is loaded
        sys.path.insert(0, str(HK / "sim" / "arm")); sys.path.insert(0, str(HK / "src"))
        import pick
        import wrist_refine as wr
        from dimos.hardware.manipulators.sim.adapter import ShmMujocoAdapter
        sys.path.insert(0, str(HK / "sim" / "dimos")); import env as hk_env
        if sweep is None:
            if getattr(self, "_given_flags", None) is not None:
                self._status["state"] = "finds from the sweep"; flags = self._given_flags   # sweep, then visit ([2w8])
            else:
                self._status["state"] = "finding items"; flags = self._find_items(wr)        # stand-in: head camera only
            flags, out = self._split_inside(flags)              # [OrO]: finds outside the area are noted, not visited
            for f in out: self._note_outside(f, "a find")
            self._status["flags"] = [list(np.round(f, 2)) for f in flags]; self._say(f"{len(flags)} candidate items")
        if not isinstance(self._posture, arm_guard.GuardedPosture): self._posture = arm_guard.GuardedPosture(self._posture, self._guard)
        arm = arm_guard.GuardedArm(ShmMujocoAdapter(dof=7, address=hk_env.ARM_KEY), self._guard)   # every arm command
        arm.connect(); arm.write_enable(True)                                                      # through the guard
        ck = os.environ.get("HK_PICK_POLICY", "")                   # a trained pick policy, else the scripted grasp
        self._policy = pick.PolicyClient(ck) if ck else None; self._policy_ee = os.environ.get("HK_PICK_EE", "1") == "1"
        if ck: self._say(f"picking with the policy {ck} ({'gripper-frame' if self._policy_ee else 'joint'} actions)")
        park = _park_offset() - np.array([_lie_settle(), 0.0]); self._say(f"park so each item sits at {np.round(park, 2)} in the robot's frame")
        self._ctx = {"pick": pick, "wr": wr, "arm": arm, "S": None, "park": park, "handled": []}   # handled: never twice ([XfI])
        if sweep is not None: self._sweep_and_pick(sweep)          # confirm on approach ([d8d], [QDc])
        else:
            why = "a find from the sweep" if getattr(self, "_given_flags", None) is not None else "seen by the head camera looking round"
            for i, f in enumerate(flags): self._aim(f, why, "find"); self._handle_find(i + 1, np.asarray(f), f"/{len(flags)}")
        arm.disconnect()

    STANDING_LOOK = os.environ.get("HK_STANDING_LOOK", "1") == "1"   # the close look's cheap first step
    LOOK_AT_M = (0.55, 0.95, 0.75)     # a find is looked at standing from this range (m; closer to 0.75 if outside): the
                                       # aiming poses measured safe for balance ([BPL], standing_look.py) look 0.5-1.0 m ahead
    LOOK_MATCH_M = 0.45                # a toy seen standing this close to the flag is the find (the flag is 13-30 cm off)

    def _robot(self):
        """the arm's kinematic copy, made once"""
        from houndkeeper.dimos_robot import DimosRobot
        c = self._ctx
        if c["S"] is None:
            getimg = lambda: None if self._wrist is None else np.asarray(self._wrist.to_rgb().data)
            c["S"] = DimosRobot(c["arm"], getimg, lambda: self._pose(), say=self._say)
            c["q_ready"], c["q_bin"] = c["S"].ready_and_bin()
        return c["S"]

    SETTLE_BASE_S = 5.0         # the most a standing look waits for odometry to read the base still before the arm moves
    SETTLE_HOLD_S = 1.0         # ... still this long without a break: after a turn the gait steps on in pulses for a second
                                # or two, and one reading in the lull between steps let the arm start (18 Oct: refused mid-move)
    LOOK_STRAIGHT_RAD = 0.35    # a standing look's pose within this of the stow on every joint goes straight (the look poses
                                # are within 0.23 rad, their balance measured, [BPL]); further, the planner
    PLACED_TOL = 0.10           # m: how far the close look's wide look may move a find the standing look placed (it places
                                # them within 4-10 cm, 28 Sep)
    LOOK_UP_DEG = 15.0          # the flag this far above the wrist image's centre: the jaws reach up to ~10 px over the
                                # centre (65 deg fovy, 640 x 360) and hid finds on their tip (27 Sep, night 70/3: the robot
                                # stopped 9 cm from a toy and dropped it as its own jaw); 15 deg puts it at ~y 100

    NOPE = os.environ.get("HK_NOPE", "1") == "1"   # manners [N2T]
    HAPPY = os.environ.get("HK_HAPPY", "1") == "1"  # manners [Yju]
    WIGGLE_RAD_S, WIGGLE_S = 1.2, 0.4               # manners [M7r]: turn rate asked of the walking controller, and one full
                                                    # swing's time (est.: +-10-15 deg of body yaw once the controller lags)

    @classmethod
    def wiggle_plan(cls) -> list:
        """the happy wiggle as (turn rate, seconds): half a swing, three full ones the other way and back, half a swing,
        so the swings cancel and it ends facing the way it was"""
        r, t = cls.WIGGLE_RAD_S, cls.WIGGLE_S
        return [(r, t / 2), (-r, t), (r, t), (-r, t), (r, t / 2)]

    def _happy(self) -> None:
        """an item in the bin: the whole dog twists side to side a couple of times, standing"""
        if not self.HAPPY: return
        for rate, secs in self.wiggle_plan():
            for _ in range(max(1, int(round(secs / 0.05)))):
                self.cmd_vel.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, rate))); clock.sleep(0.05)
        self.cmd_vel.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))); clock.sleep(0.3)
    NOPE_RAD = 0.08             # shoulder turn each way: ~3 cm side to side at the looks' 0.3-0.4 m reach (est.); the
                                # shake's path is 8x this, at ARM_VMAX 1 rad/s x 1.6 ~1.0 s in all (0.15 took ~1.9 s); a wrist
                                # roll of 0.35 rad read as a twist, not a "no", in the sim's viewer

    def _nope(self, S) -> None:
        """a look that found nothing: the whole arm turned side to side twice at the shoulder (the first joint, about the
        vertical: the gripper keeps its height, so it sweeps no lower than the look already was), then back where it was"""
        if not self.NOPE: return
        q = np.asarray(S.target()[:6], float).copy(); d = np.zeros(6); d[0] = self.NOPE_RAD
        if hasattr(S, "refuses") and any(S.refuses(q + sign * d) for sign in (1, -1)): return   # the arm guard's region
        for sign in (1, -1, 1, -1): S.arm_to(q + sign * d, 0.1)      # arm_to stretches each to its speed limit
        S.arm_to(q, 0.1)

    def _base_settled(self) -> bool:
        """whether odometry read the base still for SETTLE_HOLD_S without a break, waiting up to SETTLE_BASE_S"""
        t0 = clock.now(); since = None
        while clock.now() - t0 < self.SETTLE_BASE_S:
            now = clock.now()
            if self._guard.base.moving(now): since = None
            elif since is None: since = now
            elif now - since >= self.SETTLE_HOLD_S: return True
            clock.sleep(0.1)
        return False

    STOW_TRIES = 3              # a stow the arm guard refuses is tried this many times in all, each after the base settles
    STOW_READ_S = 2.0           # how long a stow that ran may take for the joints to read stowed
    STOW_STILL_S, STOW_STILL_RAD = 0.5, 0.01   # short of the stow and no joint moved this much in this long: blocked

    def _stow(self, move, hold=None) -> None:
        """[WkC]: the arm back in the stow before the robot walks or changes posture. A move refused partway leaves the
        arm held where the last command let through put it; the base is given time to settle and the stow tried again,
        STOW_TRIES in all. A stow nothing refused that comes to rest short of it is blocked ([G0m]: what was just binned
        standing under the fold): hold() keeps the arm where it rests rather than pressing on it, and the same move is
        not tried again. Still out: ArmStop, the night stopped where it stands"""
        why = "never read stowed"
        for attempt in range(1, self.STOW_TRIES + 1):
            if attempt > 1: self._base_settled()
            try: move()
            except arm_guard.ArmRefused as e:
                why = str(e); self._say(f"stow refused ({attempt}/{self.STOW_TRIES}), the arm held where it is: {e}"); continue
            t0 = clock.now()
            while not self._guard.stowed() and clock.now() - t0 < self.STOW_READ_S: clock.sleep(0.1)
            if self._guard.stowed(): return
            q0 = np.asarray(self._guard.read_q(), float)[:6]; clock.sleep(self.STOW_STILL_S)
            q = np.asarray(self._guard.read_q(), float)[:6]
            if self._guard.stowed(): return
            if float(np.max(np.abs(q - q0))) < self.STOW_STILL_RAD:              # come to rest short: something in the way
                out = ", ".join(f"joint {j + 1} {abs(d):.2f} rad" for j, d in enumerate(q - arm_guard.STOW) if abs(d) > arm_guard.STOW_TOL_RAD)
                if hold is not None:
                    try: hold()
                    except arm_guard.ArmRefused as e: self._say(f"blocked stow: holding the arm where it rests refused ({e})")
                raise ArmStop(f"the arm could not be stowed: blocked short of the stow ({out} out), something in the fold's way")
        raise ArmStop(f"the arm could not be stowed: {why}")

    def _standing_look(self, f: np.ndarray) -> tuple[np.ndarray | None, str]:
        """look at the find standing, the wrist camera aimed down at it from ~0.6 m: a wide view (the flag is 13-30 cm off
        the toy), seconds instead of a park and lie down. Returns the toy's floor position if one is seen near the flag,
        None if the flag was in clear view and nothing was there, or the flag unchanged if it couldn't be judged"""
        import legs
        import standing_look as sl
        import wrist_refine as wr
        S = self._robot(); pick = self._ctx["pick"]
        p, _ = self._pose(); d = float(np.hypot(*(f - p[:2]))); yaw = math.atan2(f[1] - p[1], f[0] - p[0])
        lo, hi, mid = self.LOOK_AT_M
        if not lo <= d <= hi:                                                     # close to (or back off to) 0.75 m, facing it
            stand = f - mid * np.array([math.cos(yaw), math.sin(yaw)]); self._go(float(stand[0]), float(stand[1]), yaw, timeout=30.0)
        self._turn(((math.atan2(f[1] - self._pose()[0][1], f[0] - self._pose()[0][0]) - self._yaw() + math.pi) % (2 * math.pi)) - math.pi)
        S.legs = legs.STAND; S.sync(); q_stow = S.target()[:6].copy()
        q, miss, cam_up = sl.aim(S.K, np.array([f[0], f[1], 0.0]), q_stow, up_deg=self.LOOK_UP_DEG)
        if miss > 5.0 or cam_up < sl.MIN_CAM_UP - 0.05:
            S.legs = legs.LIE; return f, f"standing look: no aim ({miss:.0f} deg off)"
        if not self._base_settled():                                              # the turn's last steps done ([G48])
            S.legs = legs.LIE; return f, "standing look: the base did not settle"
        if (why := S.refuses(q)) is not None:                                     # outside what the arm guard allows standing
            S.legs = legs.LIE; return f, f"standing look: the arm guard refused ({why})"
        straight = float(np.max(np.abs(np.asarray(q) - q_stow))) <= self.LOOK_STRAIGHT_RAD   # a look pose beside the stow:
        back = lambda: S.arm_to(q_stow, 2.0) if straight else S.arm_to_safe(q_stow, 2.0)   # moved straight there and back
        hold = lambda: S.arm_to(S.joint_state()[:6], 0.3)                         # a blocked stow: held where it rests ([WkC])
        try:                                                                        # (the planner's checker took these poses
            if straight: S.arm_to(q, 2.0)                                           # for collisions and flung the arm straight
            else: S.waypoints = []; S.arm_to_safe(q, 2.0)                           # up, 28 Sep)
        except arm_guard.ArmRefused as e:                                           # [WkC]: refused partway: held, then stowed
            self._base_settled(); self._stow(back, hold); S.legs = legs.LIE; return f, f"standing look: the arm guard refused partway ({e})"
        v = S.capture()
        img, H, W = v["img"], v["img"].shape[0], v["img"].shape[1]; fpx = (H / 2) / math.tan(math.radians(v["fovy"]) / 2)
        seen = []
        pc0 = v["R"].T @ (np.array([f[0], f[1], 0.0]) - v["c"])                    # the flag in the image: look closer there
        fl_px = (W / 2 + fpx * pc0[0] / -pc0[2], H / 2 - fpx * pc0[1] / -pc0[2]) if pc0[2] < 0 else (W / 2, H / 2)
        for (x1, y1, x2, y2), score, label in wr.owl_boxes_zoomed(img, fl_px, wr.FIND_QUERIES, thr=0.15):
            u, vv = (x1 + x2) / 2, y2 - 0.1 * (y2 - y1)                            # where it meets the floor: its box's foot
            if v["robot"][int(min(max(vv, 0), H - 1)), int(min(max(u, 0), W - 1))]: continue   # the robot's own parts
            ray = v["R"] @ np.array([(u - W / 2) / fpx, -(vv - H / 2) / fpx, -1.0])
            if ray[2] >= -1e-3: continue
            xy = v["c"][:2] + ray[:2] * (-v["c"][2] / ray[2]); seen.append((float(np.hypot(*(xy - f))), xy, label, score, (x1, y1, x2, y2)))
        pc = v["R"].T @ (np.array([f[0], f[1], 0.0]) - v["c"])                    # was the flag itself in clear view?
        in_view = pc[2] < 0 and abs(fpx * pc[0] / -pc[2]) < W / 2 - 20 and abs(fpx * pc[1] / -pc[2]) < H / 2 - 20
        self._status.setdefault("standing_looks", []).append({                    # each look, for checking the aim
            "flag": [round(float(x), 3) for x in f], "robot": [round(float(x), 3) for x in self._pose()[0]], "aim_miss_deg": round(miss, 1),
            "cam_up_planned": round(cam_up, 2), "q_planned": [round(float(x), 3) for x in q], "q_actual": [round(float(x), 3) for x in S.joint_state()[:6]],
            "cam": [round(float(x), 3) for x in v["c"]], "flag_px": [round(float(W / 2 + fpx * pc[0] / -pc[2]), 0), round(float(H / 2 - fpx * pc[1] / -pc[2]), 0)] if pc[2] < 0 else None,
            "img_wh": [W, H], "seen": [[round(float(s_[0]), 2), s_[2]] for s_ in seen]})
        if os.environ.get("HK_LOOKS_DIR"):
            import cv2; d_ = Path(os.environ["HK_LOOKS_DIR"]); d_.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(d_ / f"standing_{len(self._status['standing_looks'])}.png"), np.ascontiguousarray(img[..., ::-1]))
        near = sorted([s_ for s_ in seen if s_[0] <= self.LOOK_MATCH_M], key=lambda s_: s_[0])
        if not near and in_view:                                                   # nothing there: a shake of the head
            try: self._nope(S)
            except arm_guard.ArmRefused as e: self._say(f"standing look: the shake refused partway ({e})")
        self._stow(back, hold); S.legs = legs.LIE                                   # back to the stow ([WkC])
        if near:
            dist, xy, label, score, box = near[0]; mid = ""
            if pick.WHOLE_ITEM:                                                     # sim-slice [82p]: park and aim on its middle,
                m = wr.find_point(v, box, wr.segment_box(img, box, v["robot"]), middle=True)   # not the end nearest the robot
                if m is not None:
                    self._status["standing_looks"][-1].update(middle=[round(float(x), 3) for x in m["xy"]], length_cm=round(m["length_cm"], 1),
                                                             long_axis=[round(float(x), 3) for x in m["long_axis"]])
                    mid = f", its middle {np.hypot(*(m['xy'] - xy)) * 100:.0f} cm beyond its foot ({m['length_cm']:.0f} cm long)"; xy = m["xy"]
            return np.asarray(xy, float), f"standing look: {label} ({score:.2f}) {dist * 100:.0f} cm from the flag{mid}"
        if in_view: return None, "standing look: nothing near the flag"
        return f, "standing look: flag out of view"

    def _handle_find(self, k: int, f: np.ndarray, of: str = "") -> dict:
        """one find: face it head-on from 0.9 m and re-check with the head camera, shuffle in to park, lie down, look
        closer, pick (up to 3 tries, [m62]) or leave it for review with the reason, stow, stand up"""
        from houndkeeper.dimos_robot import DimosRobot
        c = self._ctx; pick, wr, S = c["pick"], c["wr"], c["S"]
        rep = {"flag": list(np.round(f, 2))}; self._status["items"].append(rep); tag = f"item {k}{of}"
        self._state("GO TO FIND", f"{tag}: re-checking the flag")
        f, rep["refine"] = self._refine_flag(np.asarray(f), wr); self._say(f"{tag}: {rep['refine']}")
        if self.STANDING_LOOK:
            self._state("LOOK STANDING", tag)
            g, rep["standing"] = self._standing_look(np.asarray(f)); self._say(f"{tag}: {rep['standing']}")
            if g is None:                                                  # nothing there: no park, no lie down
                rep["outcome"] = "review: not seen standing"; self._say(f"{tag}: {rep['outcome']}"); return rep
            f = g
        self._state("GO TO FIND", f"{tag}: walking to park head-on")
        rep["nav"], rep["park_err_cm"] = self._approach(np.asarray(f), c["park"], walk=False)   # head-on: facing it from 0.9 m, shuffle in
        self._say(f"{tag}: navigation {rep['nav']}, item {rep['park_err_cm']} cm from where it should sit")
        for again in (1, 2):                                     # parked well off (45-58 cm, 26 Sep: the look then misses it):
            if rep["park_err_cm"] <= self.PARK_TOL_CM: break          # walk to the spot beside it properly and shuffle in again
            rep["nav"], rep["park_err_cm"] = self._approach(np.asarray(f), c["park"], walk=True); rep["reparks"] = again
            self._say(f"{tag}: re-parked ({again}): navigation {rep['nav']}, item {rep['park_err_cm']} cm off")
        for attempt in (1, 2):                                   # a second try re-parks on look-closer's precise position
            self._state("LIE DOWN", tag); self._posture.liedown(); clock.sleep(4.0)
            S = self._robot()                                                  # one kinematic copy for the run
            self._base_settled()                                               # still, and seen to be, before the arm moves
            q_ready, q_bin = c["q_ready"], c["q_bin"]; ref = None
            try:
                S.max_tilt = 0.0; S.waypoints = [q_ready]; S.arm_to_safe(q_ready, 2.0)
                self._state("LOOK CLOSER: find and name", tag)
                placed = rep.get("standing", "").startswith("standing look: ") and " cm from the flag" in rep.get("standing", "")
                ref, notes = pick.look_closer(S, np.asarray(f), wide_tol=self.PLACED_TOL if placed else pick.WIDE_TOL)   # the standing
                rep[f"look{attempt}"] = notes; fail = None                                                           # look placed it
                if os.environ.get("HK_LOOKS_DIR"): pick.save_looks(S, Path(os.environ["HK_LOOKS_DIR"]) / f"item{k}_look{attempt}.npz", notes)
                if ref is None: fail = "not seen at the flag"; self._nope(S)          # lying, looked close: nothing there
                elif (same := pick.same_as(ref["xy"], [h for h in c["handled"] if not (ref.get("fits_jaws") and h[0] in c.setdefault("too_wide", set()))],
                                           this=k)) is not None:   # an object refused as too wide may be tried again by a look
                                                                   # that finds it fits: new information, not a second try (28 Sep:
                                                                   # a figure refused at its first stop, fitting by one leg at two more)
                    fail = f"same as item {same}"; rep["same_as"] = same   # a second flag on an object already handled
                else:
                    rep["label"] = ref.get("label"); rep["grip_width_cm"] = round(ref["grip_width_cm"], 1); rep["score"] = ref.get("score")
                    rep["size_cm"] = [None if ref.get(k) is None else round(float(ref[k]), 1) for k in ("length_cm", "breadth_cm")]   # as
                    # measured, for judging "too big for the bin" against the truth afterwards ([bOu], [pYZ])
                    self._state("GRASP", f"{tag}: {rep['label']}")
                    if self._policy is not None:                         # learned pick from the handover pose, scripted carry
                        q = pick.handover_pose(S, ref["xy"])
                        if q is None: fail = "no handover pose"
                        else:
                            S.arm_to(q, 1.5); S.step(40); rep["picker"] = "policy"
                            pick.policy_pick(S, self._policy, ee=self._policy_ee); fail = pick.carry_to_bin(S, q_ready, q_bin)
                    else:
                        fail, held = pick.pick_with_tries(S, ref, q_ready, q_bin,   # [m62]; the re-look held near where it was just seen
                                                          relook=lambda xy: pick.look_closer(S, xy, wide_tol=self.PLACED_TOL))
                        rep["picker"] = "scripted"; rep["tries"] = getattr(S, "tries", 1)
                        if getattr(S, "relook_notes", None): rep["relooks"] = S.relook_notes
                        if getattr(S, "try_log", None): rep["try_log"] = S.try_log
                    c["handled"] = [h for h in c["handled"] if h[0] != k] + [(k, np.asarray(ref["xy"]))]   # where it was last seen
                    if fail == "too wide to grip": c.setdefault("too_wide", set()).add(k)
            except arm_guard.ArmRefused as e:                         # [qpO]: refused before it moved; left for review
                fail = f"the arm guard refused: {e}"
            if not self._guard.stowed():                             # a refused first move left it folded: nothing to stow
                self._base_settled()                                 # fold (factory fold = the stow pose), retried if refused ([WkC]),
                self._stow(lambda: (pick.stow(S, q_ready), S.arm_to_safe(np.zeros(6), 2.0, grip=pick.CLOSED)),
                           hold=lambda: S.arm_to(S.joint_state()[:6], 0.3))   # blocked: held where it rests, not pressed
            # jaws closed as the night starts: folded open, they stuck out while walking and changed the wrist camera's
            # standing looks after the first find; the item was already let go over the bin
            rep["outcome"] = "binned" if fail is None else (fail if fail.startswith("same as") else f"review: {fail}")
            if attempt == 2 or ref is None or fail not in ("no reachable grip direction", "no handover pose", "no grasp pose"): break
            self._say(f"{tag}: out of reach from here; re-parking on the item as look-closer placed it")
            self._posture.standup(); clock.sleep(5.0); f = repark_target(ref)     # the grip, not the middle ([Q8C])
            rep["repark_err_cm"] = self._approach(f, c["park"], walk=False)[1]
        self._state("STAND UP", tag); self._posture.standup(); clock.sleep(5.0)
        if rep.get("outcome") == "binned": self._happy()
        self._say(f"{tag}: {rep['outcome']}")
        return rep

    # --- confirm on approach ([d8d], [QDc]) ---
    SIDESTEP_M, SIDESTEP_WAIT_S = 0.30, 4.0     # a sidestep's length; how long a sidestepped spot may stay unsure before the
                                                # arm's close look decides it (the next, dearer step)

    def _sweep_and_pick(self, sw: dict) -> None:
        """Walk the sweep route judging the floor ahead from the head camera as it goes (confirm_stream.Confirmer, the
        locked detector's comparison applied frame by frame against the earlier nights' floor). A spot confirmed on
        approach gets a stop at once; one still unsure close in gets a sidestep for a new view; one still unsure after
        that gets the arm's close look. Then the sweep resumes where it stopped ([Xsf])."""
        if self.SWEEP_ARM: raise RuntimeError("HK_SWEEP_ARM refused: the arm is carried stowed while the base moves ([G48])")
        sys.path.insert(0, str(HK / "sim" / "house")); import confirm_stream as cst
        base, Vb, band, hmap, prev = cst.baseline(Path(sw["run_dir"]), int(sw["night"]))
        self._say(f"sweep: judging the floor against nights {prev}")
        setting = os.environ.get("HK_DETECT_SETTING")                   # a named setting ([tTD]: "launch")
        a = cst.options(setting)                                        # the replay's own options at it ([BW3])
        if setting and a.check: self.CANDIDATE_CHECK = True
        if a.near_res:                                                  # [Ot0]: the near grid, on the baselines' near views
            a.near_views, missing = near_views(sw["run_dir"], prev)
            if missing: self._say(f"sweep: setting {setting}: no near views for nights {missing}; "
                                  + ("the near grid compares those with the blended floor" if a.near_views else "the near grid is off"))
            if not a.near_views: a.near_res = 0.0
        from scipy import ndimage as ndi
        self._room = ndi.distance_transform_edt(~band) * cst.RES   # m beyond the band kept clear of mapped structures
        self._cf_args = (base, Vb, band, hmap, a); self._cf = None; self._cst = cst; self._hmap0 = hmap
        self._run_dir, self._prev = sw["run_dir"], list(prev)
        lags = [float(z["lag_s"]) for k in prev for z in [np.load(Path(sw["run_dir"]) / f"mosaic_n{k}.npz")] if "lag_s" in z]
        self._mount = None                                         # the head camera's calibrated mount, as the pictures used it
        mp = Path(sw["run_dir"]) / "head_mount.json"               # ([KkQ]; night_tidy.sh's stitch step writes it)
        if mp.exists():
            from houndkeeper import calibrate as _cal
            self._mount, self._mount_off = _cal.mount(json.loads(mp.read_text()))   # its turn, and its place (body frame)
        self._lag, why = robot_frame_lag(lags)                    # the camera's stamps run late ([uKM] 2, [gJq]): each frame is
        self._say(f"sweep: frames placed {self._lag * 1000:.0f} ms before their stamps ({why})")   # placed where the camera was
        self._relief = baseline_calibration(sw["run_dir"], prev, self._say)   # the floor's relief, the baselines checked ([yFg], [Q9k])
        house = Path(sw["run_dir"]) / "house_lidar.npy"           # the house's lidar map, which the pictures were placed on
        if os.environ.get("HK_LIVE_PLACE") == "1":               # [x9X]: the house's pose graph places tonight, live: its
            self._live_loc = self._live_placer(sw["run_dir"]); a.track = False   # keyframes a fixed map, the floor's locks
                                                                 # constraints in it (not the tracking's gain). Off by default
        elif house.exists() and os.environ.get("HK_LIVE_LOC") == "1":   # ([KkQ]): tonight's frames are placed on it too.
                                                                   # Off by default: on a 25 min tidy night the live match slid
                                                                   # 0.4-0.9 m after 5 min (odometry: 0.14), with good fits,
                                                                   # against a map its own first night had warped (27 Sep)
            from houndkeeper.live_localize import LiveLocalizer
            self._live_loc = LiveLocalizer(np.load(house)); self._say("sweep: localizing against the house's lidar map")
        elif house.exists(): self._say("sweep: live localization off (HK_LIVE_LOC=1 turns it on); frames placed by odometry")
        else: self._say("sweep: no house lidar map; frames placed by odometry alone")
        self._sidestepped: dict = {}; self._sweep_log = self._status.setdefault("sweep", []); self._binned: list = []
        self._check = None; self._held: dict = {}                  # [Zaf]: the light check per candidate, made with the
        if self.CANDIDATE_CHECK: self._say("sweep: each find checked on crops of its nearest frames before a stop")   # first frame
        self._cleared: list = []
        route = [np.asarray(w[:2], float) for w in sw["route"]]; k = 0
        route, out = self._split_inside(route, inset=0.0)        # [OrO]: the route is planned inside; a waypoint outside is
        if out: self._say(f"{len(out)} route waypoint(s) outside the operating area: skipped")   # dropped, never walked
        self._route = route; whys = list(sw.get("why") or [])   # the route planner's reason for each waypoint, if it gives one
        self._balance = self._status.setdefault("balance", {"arm": self.SWEEP_ARM or "stowed", "n": 0, "max_deg": 0.0,
                                                           "min_z": 9.0, "over5": 0, "over10": 0, "fell": False})
        for wi, w in enumerate(route):
            unblock = 0
            while True:
                self._state("SWEEP", f"waypoint {wi + 1}/{len(route)}")
                self._aim(w, f"the sweep, waypoint {wi + 1} of {len(route)}" + (f": {whys[wi]}" if wi < len(whys) and whys[wi] else ""), "waypoint", wp=wi)
                outcome, rec = self._walk_watching(w)
                if rec is None and outcome in ("gave up", "timed out") and unblock < 2 and not self.WATCH_ONLY:
                    blockers = self._blockers()                    # walled in by its own evidence ([tmq]): settle the spots in
                    if blockers:                                   # the way with the close look, then try again: the
                        unblock += 1                               # robot can unblock itself)
                        for xy in blockers[:3]:
                            k += 1; self._state("GO TO FIND", f"find {k} at ({xy[0]:.2f}, {xy[1]:.2f}): in the way to waypoint {wi + 1}")
                            self._aim(xy, f"in the way to waypoint {wi + 1}", "find")
                            rep = self._handle_find(k, np.asarray(xy)); rep["confirm"] = {"x": xy[0], "y": xy[1], "decision": "blocking"}
                            self._settled(k, xy, rep)
                        continue
                if rec is None:                                    # a waypoint it couldn't reach is reported, not counted as
                    if outcome != "reached":                       # walked (26 Sep: walled in by its own keep-out discs, the robot
                        self._sweep_log.append({"waypoint": wi + 1, "not_reached": outcome})   # never moved again and
                        self._say(f"waypoint {wi + 1} not reached ({outcome})")               # reported the sweep done)
                    break
                if rec["decision"] == "unsure":
                    self._sidestep(rec); continue
                k += 1; self._state("GO TO FIND", f"find {k} at ({rec['x']:.2f}, {rec['y']:.2f}), {rec['why']}")
                self._aim((rec["x"], rec["y"]), rec["why"], "find")
                rep = self._handle_find(k, np.array([rec["x"], rec["y"]])); rep["confirm"] = rec
                self._settled(k, (rec["x"], rec["y"]), rep)
        for rec in self._checked_at_end():                         # [Zaf]: finds the check kept after the last waypoint
            if self.WATCH_ONLY: self._sweep_log.append({k_: v_ for k_, v_ in rec.items() if k_ != "cells"}); continue   # logged, never stopped for
            if rec["decision"] != "confirmed" or not self._worth_a_stop(rec) or not self._inside_area((rec["x"], rec["y"])): continue
            k += 1; rec["why"] = "confirmed on approach, kept by the check after the sweep"
            self._state("GO TO FIND", f"find {k} at ({rec['x']:.2f}, {rec['y']:.2f}), {rec['why']}")
            self._aim((rec["x"], rec["y"]), rec["why"], "find")
            rep = self._handle_find(k, np.array([rec["x"], rec["y"]])); rep["confirm"] = rec
            self._settled(k, (rec["x"], rec["y"]), rep)
        self._state("DONE", f"{k} stops, {sum(r.get('outcome') == 'binned' for r in self._status['items'])} binned")

    def _live_placer(self, run_dir):
        """[x9X]: the house's graph placing tonight's frames (live_place.LivePlacer) on the house's keyframes, made from
        the run's own nights and solve the first time they are asked for"""
        sys.path.insert(0, str(HK / "sim" / "house")); import live_place as lp
        kf = Path(run_dir) / "house_keyframes.npz"
        if not kf.exists(): self._say("sweep: making the house's keyframes for the live graph"); lp.build(run_dir, kf)
        hk = lp.HouseKeyframes.load(kf); self._say(f"sweep: tonight placed by the house's graph ({len(hk)} keyframes)")
        return lp.LivePlacer(hk)

    SWEEP_ARM = os.environ.get("HK_SWEEP_ARM", "")   # "look": the arm held up through the sweep (to re-evaluate
                                                     # the arm deployed during the sweep): refused since the arm guard, the
                                                     # arm is carried stowed while the base moves ([G48])

    def _state(self, name: str, note: str = "") -> None:
        self._status["state"] = name; self._status["phase"] = None; self._phase = None; self._publish(note); logger.info(f"[tidy] {name}: {note}")

    def _head_camera(self, ts: float | None = None):
        """the head camera's pose (world <- optical, 4x4) when the frame was taken, from DimOS's transform tree: the same
        transform the recordings (and so the earlier nights' floor pictures) were built with. The
        robot's kinematic copy places the sim's camera 1 cm higher than DimOS declares it (25 Sep), which moves a floor
        point 2 m ahead by ~7 cm, so it is only the fallback. The pose is interpolated to the frame's own time between
        the poses either side ([Feg]), as the floor pictures' are (houndkeeper.rec_poses); a frame newer than the last
        pose takes the nearest one, as before."""
        tr = None
        if ts is not None:
            tr = self.tfbuffer.get("world", "camera_optical", ts, 0.2, interpolate=True, max_gap=0.2, warn=False)
            if tr is None: tr = self.tfbuffer.get("world", "camera_optical", ts, 0.2)
        if tr is not None: return tr.to_matrix()                   # the real robot has no kinematic copy to fall back on
        import mujoco                                              # ([Lzt]): the sim's model is built only when it is used
        if getattr(self, "_K", None) is None:
            sys.path.insert(0, str(HK / "sim" / "arm")); import vslice as vs
            self._K = vs.Sim("sock", np.array([30.0, 30.0]), with_obj=False, watch=False)
        m, d = self._K.m, self._K.d; cid = m.camera("head_camera").id
        p, q = self._pose()
        d.qpos[0:3] = p; d.qpos[3:7] = q; mujoco.mj_kinematics(m, d); mujoco.mj_camlight(m, d)
        T = np.eye(4); T[:3, :3] = d.cam_xmat[cid].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0]); T[:3, 3] = d.cam_xpos[cid]
        return T

    def _head_lens(self, img: np.ndarray):
        """the head camera's lens ([4hm], [kJ6]): straightens a frame, and gives the straightened view's (fx, fy, cx, cy)"""
        from houndkeeper import lens
        H, W = img.shape[:2]
        if getattr(self, "_lens", None) is None or self._lens.size != (W, H): self._lens = lens.head(W, H)
        return self._lens(img), self._lens.K

    LIVE_TALL_M = 0.35          # tonight's lidar counts for lines of sight above this: taller than any item we pick, so a
    WATCH_ONLY = os.environ.get("HK_TIDY_WATCH_ONLY") == "1"   # the walk-through alone: judge every frame, log each
                                # decision, never stop for one (the detection bench's recordings: passes of
                                # the walk-through alone); a sweep's 240 s exploration had stood in for it, half its frames standing still
    PARK_TOL_CM = 12.0          # parked further than this from where the item should sit: walk in again (good parks: 3-10 cm, 26 Sep)
                                # toy never hides its own footprint, while a chair moved tonight or a table's legs do

    def _refresh_sightlines(self) -> None:
        """the lines of sight the Confirmer checks: the earlier nights' tall things, plus tonight's live map above
        LIVE_TALL_M (every few seconds; the head camera must not read furniture as floor)"""
        m = self._live_map
        if m is None or m is getattr(self, "_live_used", None) or clock.now() - getattr(self, "_live_t", 0.0) < 3.0: return
        self._live_used, self._live_t = m, clock.now(); cst = self._cst
        pts = m.as_numpy(); pts = np.asarray(pts[0] if isinstance(pts, tuple) else pts)
        pts = pts[pts[:, 2] > self.LIVE_TALL_M]
        if hasattr(self._cf, "house_points"): pts = self._cf.house_points(pts)   # into the frame the Confirmer judges in
        live = np.zeros_like(self._hmap0)
        r = ((cst.HY - pts[:, 1]) / cst.RES).astype(int); c = ((pts[:, 0] + cst.HX) / cst.RES).astype(int)
        ok = (r >= 0) & (r < live.shape[0]) & (c >= 0) & (c < live.shape[1])
        np.maximum.at(live, (r[ok], c[ok]), pts[ok, 2].astype(np.float32))
        from scipy import ndimage as ndi
        self._cf.hmap = np.maximum(self._hmap0, ndi.maximum_filter(live, size=3))

    def _watch_frame(self) -> None:
        """judge the newest head frame (the Confirmer paints and compares the floor 0.5-2.5 m ahead)"""
        im = self._head
        if im is None or im is getattr(self, "_last_im", None): return
        ts = getattr(im, "ts", None); ts = None if ts is None else ts - getattr(self, "_lag", 0.0)   # when it was taken, not stamped
        self._last_im = im; T = self._head_camera(ts)
        if getattr(self, "_mount", None) is not None:
            T = T.copy(); T[:3, :3] = T[:3, :3] @ self._mount
            f_ = T[:2, 2] / (np.linalg.norm(T[:2, 2]) + 1e-9); Rb = np.array([[f_[0], -f_[1], 0], [f_[1], f_[0], 0], [0, 0, 1.0]])   # the body's
            T[:3, 3] = T[:3, 3] + Rb @ getattr(self, "_mount_off", np.zeros(3))   # heading, from where the camera looks
        raw = np.asarray(im.to_rgb().data); img, K = self._head_lens(raw)
        if self.CANDIDATE_CHECK: self._check_frame(im, raw, T)
        self._view = (T, K, getattr(getattr(self, "_lens", None), "D", None), img.shape[1::-1], clock.now())   # the pose this frame is judged
        if self._cf is None:                                                               # with: the mind view's outlines ([Izb])
            H_, W_ = img.shape[:2]
            self._cf = self._cst.Confirmer(*self._cf_args[:4], K, (W_, H_), self._cf_args[4])
            self._cf.relief = getattr(self, "_relief", None)                # tonight laid where the views were ([yFg])
            views = [v for v in sorted(Path(self._run_dir).glob("views_n*.npz"))   # the earlier nights' own views ([WFI]);
                     if v.stem[7:].isdigit() and int(v.stem[7:]) in self._prev]      # not views_near_n<k>, the near grid's
            a = self._cf_args[4]                                           # [BW3]: and their view noise, as the replay
            self._cst.use_views(a, self._cf, [str(v) for v in views])      # uses them; the near grid beside it
            cfn = self._cst.near_layer(a, *self._cf_args[:4], K, (W_, H_), self._cf.relief)
            if cfn is not None: self._cf = self._cst.Layers(self._cf, cfn, a)
            if self._live_loc is not None:
                from houndkeeper.live_localize import HouseFramed
                if hasattr(self._live_loc, "on_lock"): self._cf.on_lock = self._live_loc.on_lock   # [x9X]: the floor's locks
                self._cf = HouseFramed(self._cf, self._live_loc)
            elif getattr(self._cf_args[4], "track", False):            # the floor keeps the pose ([fXD]): spots come back
                from houndkeeper.live_localize import FloorTrack, HouseFramed   # to odometry's frame for the navigator
                self._cf = HouseFramed(self._cf, FloorTrack(self._cf), moves_frames=False)
        self._refresh_sightlines()
        if self._live_loc is not None: self._cf.frame(img, T, ts=ts)
        else: self._cf.frame(img, T)

    STOP_GATE = dict(dict(present_hits=4, present_area_cm2=15, gone_hits=40, unsure=False, unsure_stop_hits=None),
                     **json.loads(os.environ.get("HK_STOP_GATE", "{}")))   # (HK_STOP_GATE: json overrides, e.g. more false
    # positives accepted where that bins more toys: realistic bench, toys getting a
    # stop / stops a night: these 16/24 / 15; every confirmed 18/24 / 29; present 4 hits any size, gone 5 hits, and
    # unsure spots with 2+ hits looked at standing 20/24 / 49 (unsure_stop_hits=2)) [6Pn]: set 27 Sep on the 14-night
    # bench's labelled decisions: 95% of toys stopped for, 12 stops for nothing a night (every confirmed-present or unsure
    # spot: 40 a night, and live nights made 75 stops, binning 0-1). "Gone" was never stopped for; a strong one (dark
    # toys on dark wood) now is. Unsure spots no longer sidestep: 21 of 199 were at toys, every one also confirmed

    def _worth_a_stop(self, rec: dict) -> bool:
        """a confirmed spot worth walking to and looking at closely, by how much evidence it has ([6Pn])"""
        g, h = self.STOP_GATE, rec.get("hits", 0)
        if rec.get("kind") == "present": return h >= g["present_hits"] and rec.get("area_cm2", 0) >= g["present_area_cm2"]
        return rec.get("kind") == "gone" and h >= g["gone_hits"]

    def _decisions(self) -> dict | None:
        """a spot that needs acting on now: confirmed (stop for it), newly unsure close in (sidestep), or still unsure
        SIDESTEP_WAIT_S after its sidestep (the arm decides)"""
        p = self._pose()[0][:2]; t = clock.now()
        for rec in self._checked(self._cf.decide(t, p)):
            log = {k_: v for k_, v in rec.items() if k_ != "cells"}; self._sweep_log.append(log)   # log["why"]: the mind view's ([1XG])
            if not self._inside_area((rec["x"], rec["y"])):     # [OrO]: seen outside the area: noted, never visited
                log["outside_area"] = True; log["why"] = "outside the operating area: noted, not visited"; continue
            key = (round(rec["x"] / 0.15), round(rec["y"] / 0.15))
            if rec["decision"] == "confirmed" and self._worth_a_stop(rec):
                self._sidestepped.pop(key, None); rec["why"] = log["why"] = f"confirmed on approach at {rec['range_m']} m"; return rec
            if rec["decision"] == "confirmed": log["why"] = f"confirmed, too little for a stop ({rec.get('hits', 0)} hits)"
            if (rec["decision"] == "unsure" and self.STOP_GATE.get("unsure_stop_hits") is not None
                    and rec.get("hits", 0) >= self.STOP_GATE["unsure_stop_hits"]):   # unsure but seen: the standing look decides
                rec["decision"] = "look"; rec["why"] = log["why"] = f"unsure at {rec['range_m']} m, {rec.get('hits')} hits: a standing look"; return rec
            if rec["decision"] == "unsure" and self.STOP_GATE["unsure"] and key not in self._sidestepped:
                self._sidestepped[key] = rec; rec["why"] = log["why"] = f"unsure at {rec['range_m']} m"; return rec
        for key, rec in list(self._sidestepped.items()):
            if rec.get("t_side") and t - rec["t_side"] > self.SIDESTEP_WAIT_S:
                del self._sidestepped[key]; rec = dict(rec, decision="escalated")   # the next, dearer check ([QDc])
                rec["why"] = "still unsure after a sidestep: the arm's close look decides"; return rec
        return None

    CANDIDATE_CHECK = os.environ.get("HK_CANDIDATE_CHECK") == "1"   # [Zaf]: each candidate checked by YOLO-E on crops of
                                                     # the frames nearest in time to its decision before it gets a stop
                                                     # (houndkeeper.candidate_check; GPU if free, else CPU). Off by default

    def _check_frame(self, im, raw, T) -> None:
        """[Zaf]: a head frame to the check (its stamp, the camera's pose then, the raw fisheye frame)"""
        from houndkeeper import candidate_check as cc
        if getattr(self, "_check", None) is None:
            self._held = getattr(self, "_held", {}); lens = self._lens; H_, W_ = raw.shape[:2]; fx, fy, cx, cy = lens.K
            self._check = cc.CandidateCheck(np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]]), lens.D,
                                            "equidistant" if lens.D is not None else "", (W_, H_),
                                            recognizer=cc.Yoloe(say=self._say), max_wait_s=cc.WAIT_S)
        self._frame_ts = float(getattr(im, "ts", None) or clock.now()); self._check.frame(self._frame_ts, T, raw)

    def _checked(self, recs) -> list:
        """[Zaf]: the confirmer's decisions with each candidate held until the check has scored it, then passed on if
        kept (a candidate no frame saw is kept) or logged as dropped; other decisions pass straight on"""
        recs = list(recs)
        if getattr(self, "_check", None) is None: return recs           # the check off (the default): unchanged
        from houndkeeper import candidate_check as cc
        out = []
        for rec in recs:
            if not cc.is_candidate(rec): out.append(rec); continue
            rec = dict(rec, ts=getattr(self, "_frame_ts", clock.now()))   # decided on the newest frame
            self._check.candidate(rec); self._held.setdefault(cc.key(rec["x"], rec["y"]), []).append(rec)
        for k in [k for k in self._held if k in self._check.verdicts]:
            v = self._check.verdicts[k]; note = {"score": v["score"], "kept": v["kept"], "views_ts": [m["ts"] for m in v["views"]]}
            for rec in self._held.pop(k):
                rec["check"] = note
                if v["kept"]: out.append(rec); continue
                self._sweep_log.append(dict({k_: v_ for k_, v_ in rec.items() if k_ != "cells"},
                                            why=f"dropped by the check (score {v['score']:.2f} under {v['threshold']})"))
        return out

    def _checked_at_end(self) -> list:
        """[Zaf]: at the sweep's end every candidate still held is scored with the views it has"""
        if getattr(self, "_check", None) is None or not getattr(self, "_held", None): return []
        self._check.finish(); return self._checked([])

    def _walk_watching(self, w: np.ndarray, timeout: float = 180.0) -> tuple[str, dict | None]:
        """walk to waypoint w while judging every new head frame; stop early for a spot that needs acting on"""
        self._area_check()                                        # [Qd5]
        p = self._pose()[0][:2]; yaw = math.atan2(w[1] - p[1], w[0] - p[0])
        goal = PoseStamped(position=Vector3(float(w[0]), float(w[1]), 0.0), orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)), frame_id="world")
        self._navigation.set_goal(goal); t0 = clock.now(); idle = None; last = 0.0
        while clock.now() - t0 < timeout:
            self._watch_frame()
            if clock.now() - last >= 1.0:
                if self._area_check(walking=True):                # [Qd5]: found outside on the way: back in, then on
                    self._navigation.set_goal(goal); idle = None
                last = clock.now(); self._publish_keepout(); rec = self._decisions()
                if rec is not None and not self.WATCH_ONLY:
                    self._navigation.cancel_goal(); self._stop(); return "interrupted", rec
            if self._navigation.is_goal_reached(): return "reached", None
            if self._navigation.get_state() == NavigationState.FOLLOWING_PATH: idle = None
            elif idle is None: idle = clock.now()
            elif clock.now() - idle > 3.0: return "gave up", None
            clock.sleep(0.02)
        return "timed out", None

    def _settled(self, k, xy, rep) -> None:
        """after a find's close look: binned (gone), or nothing there (cleared): either way its spot's evidence no longer
        keeps the feet off. An item left for review stays kept out ([tmq])"""
        o = str(rep.get("outcome"))
        if o == "binned":
            self._binned += [h[1] for h in self._ctx["handled"] if h[0] == k] or [np.asarray(xy, float)]
        elif "not seen at the flag" in o or "not re-seen" in o or "not seen standing" in o:
            self._cleared.append(np.asarray(xy, float))

    def _blockers(self, reach: float = 1.5) -> list:
        """the kept-out spots nearest the robot, still unsettled (not binned, cleared or handled): the ones walling it in"""
        if self._cf is None: return []
        p = self._pose()[0][:2]; done = self._binned + self._cleared + [np.asarray(h[1], float) for h in self._ctx["handled"]]
        spots = [(d[0], d[1]) for d in self._cf.keepout() if all(np.hypot(d[0] - b[0], d[1] - b[1]) > 0.2 for b in done)]
        spots = [s_ for s_ in spots if self._inside_area(s_)]   # [OrO]: never walks out to settle one
        return sorted([s_ for s_ in spots if np.hypot(s_[0] - p[0], s_[1] - p[1]) < reach], key=lambda s_: np.hypot(s_[0] - p[0], s_[1] - p[1]))

    def _publish_keepout(self) -> None:
        """tell the costmap where the feet mustn't go: every spot the head camera has evidence on, and every find left
        on the floor for review; not what has been binned ([d8d]: the lion was kicked walking to another find)"""
        if self._cf is None: return
        gone = self._binned + getattr(self, "_cleared", [])       # binned, or the close look found nothing there
        discs = [d for d in self._cf.keepout() if all(np.hypot(d[0] - b[0], d[1] - b[1]) > 0.2 for b in gone)]
        strong = [(r["x"], r["y"]) for r in self._sweep_log if r.get("decision") == "confirmed" and self._worth_a_stop(r)]
        discs = [d for d in discs if any(np.hypot(d[0] - c[0], d[1] - c[1]) < 0.15 for c in strong)]   # only spots worth a
        # stop are kept out ([6Pn]'s gate): with every weak spot kept out the robot walled itself in and settled them one by
        # one (27 Sep: 39 of a night's 43 stops were unblocking, 0 toys binned; watch-only walks: every waypoint "no path")
        discs += [(h[1][0], h[1][1], 0.12) for h in self._ctx["handled"]
                  if all(np.hypot(h[1][0] - b[0], h[1][1] - b[1]) > 0.2 for b in self._binned)]
        pts = np.array(discs, np.float32).reshape(-1, 3)
        self.keepout.publish(PointCloud2.from_numpy(pts, frame_id="world", timestamp=clock.now()))

    # --- the operating area ([JVr]) ---
    ODOM_LOST_S = 2.0          # no odometry for this long (robot time): he stands still
    LOST_WAIT_S = 10.0         # and if none comes for this much longer, he no longer knows where he is: the run stops
    RETURN_TRIES = 3           # tries at the way back in before he stops and reports

    def _on_real_robot(self) -> bool:
        g = self.config.g
        return not (g.simulation or g.replay)

    def _area(self):
        pub = getattr(self, "_area_pub", None)
        return None if pub is None else pub.area

    def _area_loop(self) -> None:
        while not self._mind_stop.wait(1.0):
            try: self._area_pub.tick(clock.now())
            except Exception as e: logger.error(f"[tidy] operating area not published: {e!r}")

    def _house_correction(self):
        """the localization's world -> house correction (3x3), or None when the frames are taken to agree"""
        from houndkeeper.live_localize import HouseFramed
        cf = getattr(self, "_cf", None)
        return cf.live.correction() if isinstance(cf, HouseFramed) else None

    def _to_house(self, xy) -> np.ndarray:
        C = self._house_correction(); xy = np.asarray(xy[:2], float)
        return xy if C is None else C[:2, :2] @ xy + C[:2, 2]

    def _inside_area(self, xy, inset: float = 0.0) -> bool:
        """whether a point in the odometry's frame is inside the operating area (always, with none set)"""
        from houndkeeper import operating_area as oa
        a = self._area()
        return a is None or oa.inside(a, self._to_house(xy), inset)

    def _split_inside(self, pts, inset: float = 0.0) -> tuple[list, list]:
        ins, out = [], []
        for p_ in pts: (ins if self._inside_area(p_, inset) else out).append(p_)
        return ins, out

    def _note_outside(self, xy, what: str) -> None:
        self._status.setdefault("outside_area", []).append({"x": round(float(xy[0]), 2), "y": round(float(xy[1]), 2), "what": what})
        self._say(f"{what} at ({xy[0]:.2f}, {xy[1]:.2f}) is outside the operating area: noted, not visited")

    def _area_check(self, walking: bool = False) -> bool:
        """[Qd5]: localization lost -> AreaStop; outside the area -> the shortest way back in, nothing else until inside
        (AreaStop if it cannot get there). True if it had to go back in"""
        t = getattr(self, "_odom_t", None)
        if t is not None and clock.now() - t > self.ODOM_LOST_S:   # no pose: stand still at once, and say so; the run
            gap = clock.now() - t                                   # ends only if it stays lost (a 2.6 s gap in the
            if walking: self._navigation.cancel_goal()              # tidy's own odometry, none in the recording's, ended
            self._stop(); self._say(f"no pose for {gap:.1f} s: standing still until it returns")   # a sim night, 5 Oct)
            t0 = clock.now()
            while clock.now() - t0 < self.LOST_WAIT_S and clock.now() - self._odom_t > self.ODOM_LOST_S: clock.sleep(0.2)
            gap = clock.now() - self._odom_t
            if gap > self.ODOM_LOST_S: raise AreaStop(f"localization lost: no pose for {gap:.1f} s")
            self._status.setdefault("pose_gaps", []).append(round(float(clock.now() - t0 + self.ODOM_LOST_S), 1))
            self._say("pose back: going on")
            if walking: return True                                 # (the caller sets its goal again)
        a = self._area()
        if a is None or self._pose() is None or self._inside_area(self._pose()[0][:2]): return False
        from houndkeeper import operating_area as oa
        if walking: self._navigation.cancel_goal(); self._stop()
        for k in range(self.RETURN_TRIES):
            hx = self._to_house(self._pose()[0][:2]); tgt = oa.nearest_inside(a, hx)
            if tgt is None: raise AreaStop("the operating area has no room to stand in")
            C = self._house_correction(); w = np.asarray(tgt) if C is None else np.linalg.solve(C[:2, :2], np.asarray(tgt) - C[:2, 2])
            self._state("RETURN INSIDE", f"outside the operating area at ({hx[0]:.2f}, {hx[1]:.2f}): back in at ({tgt[0]:.2f}, {tgt[1]:.2f})")
            self._aim(w, "outside the operating area: the shortest way back in", "return")
            self._status.setdefault("returns", []).append({"from": [round(float(v), 2) for v in hx], "to": [round(v, 2) for v in tgt]})
            self._go(float(w[0]), float(w[1]), self._yaw(), timeout=60.0, guard=False)
            if self._inside_area(self._pose()[0][:2]): self._say("back inside the operating area"); return True
        raise AreaStop("outside the operating area and could not get back in")

    def _base_alarm(self, why: str) -> None:
        """the base moving with the arm out: the base stops (arm-on-board's failure behaviour, [G48])"""
        try: self._navigation.cancel_goal()
        except Exception: pass
        self.cmd_vel.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
        if not getattr(self, "_alarmed", False): self._alarmed = True; logger.warning(f"[tidy] arm out, base stopped: {why}")

    def _stop(self) -> None:
        self.cmd_vel.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))); clock.sleep(0.3)

    def _sidestep(self, rec: dict) -> None:
        """step SIDESTEP_M sideways, to whichever side keeps further from mapped structures, judging frames as the view
        shifts: people sway to see what they're looking at ([QDc])"""
        self._state("LOOK AGAIN", f"unsure of ({rec['x']:.2f}, {rec['y']:.2f}) at {rec['range_m']} m")
        p = self._pose()[0][:2]; y = self._yaw(); left = np.array([-math.sin(y), math.cos(y)])
        def clear(side):                                     # room at the step's end, beyond the 40 cm kept from mapped structures
            q = p + side * self.SIDESTEP_M * left
            if hasattr(self._cf, "to_house"): q = self._cf.to_house(q)   # the room map is in the house's frame
            r, c_ = self._cst.w2rc(q[0], q[1])
            if not (0 <= r < self._room.shape[0] and 0 <= c_ < self._room.shape[1]): return 0.0
            return float(self._room[r, c_])
        side = max((1.0, -1.0), key=clear)
        if not self._inside_area(p + side * self.SIDESTEP_M * left, inset=0.15):   # [OrO]: not a step out of the area
            self._say("no room inside the area to sidestep: the arm's close look will decide"); rec["t_side"] = clock.now() - self.SIDESTEP_WAIT_S; return
        self._aim(p + side * self.SIDESTEP_M * left, f"unsure of ({rec['x']:.2f}, {rec['y']:.2f}) at {rec['range_m']} m: a new view", "sidestep")
        if clear(side) <= 0:
            self._say("no room to sidestep: the arm's close look will decide"); rec["t_side"] = clock.now() - self.SIDESTEP_WAIT_S; return
        self._state("SIDESTEP", f"{'left' if side > 0 else 'right'} {self.SIDESTEP_M:.2f} m")
        t0 = clock.now(); p0 = p.copy()
        while clock.now() - t0 < 6.0 and float(np.linalg.norm(self._pose()[0][:2] - p0)) < self.SIDESTEP_M:
            self.cmd_vel.publish(Twist(linear=Vector3(0.0, side * 0.25, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
            self._watch_frame(); clock.sleep(0.05)
        self._stop(); t1 = clock.now()
        while clock.now() - t1 < 1.5: self._watch_frame(); clock.sleep(0.05)   # a still look from the new spot
        rec["t_side"] = clock.now()

    def _head_finds(self, wr) -> list[tuple[np.ndarray, float, str]]:
        """items in the head camera now, projected onto the floor from the camera's pose: [(xy, score, label)]"""
        img, (fx, fy, cx, cy) = self._head_lens(np.asarray(self._head.to_rgb().data)); p, q = self._pose()
        T = self._head_camera(getattr(self._head, "ts", None)); c = T[:3, 3]; R = T[:3, :3] @ np.diag([1.0, -1.0, -1.0])   # as MuJoCo's
        def floor_pt(u, v):                                                    # camera frame: x right, y up, looking down -z
            ray = np.array([(u - cx) / fx, -(v - cy) / fy, -1.0]) @ R.T
            return None if ray[2] >= -1e-3 else c + ray * ((0.01 - c[2]) / ray[2])
        out = []
        for (x1, y1, x2, y2), score, label in wr.owl_boxes(img, thr=0.10):
            a_, b_ = floor_pt(x1, y2), floor_pt(x2, y2)                        # the box's bottom edge on the floor: its real width
            if a_ is None or b_ is None or np.linalg.norm(a_[:2] - b_[:2]) > 0.45: continue      # furniture, not an item
            pt = floor_pt((x1 + x2) / 2, y2 - 0.15 * (y2 - y1))               # near the bottom of the box: where it meets the floor
            if pt is not None and np.linalg.norm(pt[:2] - p[:2]) < 3.0: out.append((pt[:2], score, label))
        return out

    def _refine_flag(self, f: np.ndarray, wr, standoff: float = 0.9) -> tuple[np.ndarray, str]:
        """Walk to a spot `standoff` m short of the flag, facing it, and re-detect the item with the head camera from
        close range; the refined flag keeps the side approach from walking into an item that was flagged off."""
        p, _ = self._pose(); v = f - p[:2]; dist = float(np.linalg.norm(v))
        if dist > standoff + 0.2:
            spot = f - v / dist * standoff; self._go(float(spot[0]), float(spot[1]), math.atan2(v[1], v[0]))
        self._turn(((math.atan2(f[1] - self._pose()[0][1], f[0] - self._pose()[0][0]) - self._yaw() + math.pi) % (2 * math.pi)) - math.pi)
        clock.sleep(1.0)
        near = [(xy, sc, lb) for xy, sc, lb in self._head_finds(wr) if np.linalg.norm(xy - f) < 0.35]
        if not near: return f, "not re-seen from close"
        xy, sc, lb = min(near, key=lambda t: np.linalg.norm(t[0] - f))
        return xy, f"refined {np.linalg.norm(xy - f) * 100:.0f} cm ({lb})"

    def _find_items(self, wr) -> list[np.ndarray]:
        """Turn in place in eight steps; at each, detect items in the head camera and project them onto the floor."""
        found: list[np.ndarray] = []; p0, _ = self._pose(); yaw0 = self._yaw()
        for k in range(8):
            if k: self._turn(math.pi / 4)                       # turn on the spot by velocity (the planner treats a same-spot goal as reached)
            clock.sleep(1.0)
            for xy, score, label in self._head_finds(wr):
                if all(np.linalg.norm(xy - g) > 0.3 for g in found): found.append(xy); self._say(f"saw {label} ({score:.2f}) at {np.round(xy, 2)}")
        return found
