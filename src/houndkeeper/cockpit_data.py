"""The phone cockpit's data ([Swt] [vRJ] [F2q]): the calibration procedures run live from the Cockpit's Procedures
page, and the robot's status for its Status page. Robot side of the houndkeeper extensions in cockpit/extensions/.

A procedure records what it needs from the robot's own streams while the user carries it out (real/calibration/),
shows its numbers as they come, and at the end shows the result beside the value in params/robot.yaml. Nothing is
written to the file until the user confirms on the page; every finished run is also appended to
real/local/calibration-runs.jsonl (gitignored), so runs that are combined by hand (odometry.md) are kept.

The page sends commands on procedure_cmd (JSON): {"op": "start", "id": ...}, {"op": "stop"}, {"op": "truth", ...},
{"op": "write", "run": n}, {"op": "reset"}. A write names the run whose numbers the user confirmed (procedure_state's
run), so a stale tab or a second viewer cannot write a later run's numbers; only finite numbers are ever offered for
writing. The module publishes procedure_state and link_status about twice a second."""
from __future__ import annotations

import json, math, sys, threading, time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils import clock   # the robot's clock: sim time in the sim ([3Pv])
from dimos.msgs.std_msgs.String import String

from houndkeeper import mind
from houndkeeper.battery_module import Battery

HK = Path(__file__).resolve().parents[2]
CAL = HK / "real" / "calibration"
RUNS_LOG = HK / "real" / "local" / "calibration-runs.jsonl"
PUBLISH_S = 0.5
TRUTH_LIMITS = {"forward_cm": 500.0, "left_cm": 500.0, "heading_deg": 180.0}   # what a hand measurement can be
ISOLATION_MAX_AGE_S = 3600.0           # an isolation pass older than this shows as stale (recordings.preflight's rule)
KEEP_S = 40 * 60                       # buffers hold the last 40 minutes (the battery walk is 20)
LINK_WINDOW_S = 60.0                   # the Status page's link numbers: the last minute


def _measure():
    if str(CAL) not in sys.path: sys.path.insert(0, str(CAL))
    import measure
    return measure


@dataclass(frozen=True)
class Procedure:
    id: str
    title: str
    doc: str                           # real/calibration/<doc>, the full procedure
    steps: tuple[str, ...]
    needs_truth: tuple[str, ...] = ()  # numbers the user measures by hand and enters at the end


PROCEDURES = (
    Procedure("stream", "What never arrives", "timing.md#4-what-never-arrives-video_stream",
              ("Let the robot work as usual (walking or standing) on the robot network, at night.",
               "Ten minutes or more; stop when done.")),
    Procedure("still-minute", "A still minute (camera noise)", "camera.md#1-a-still-minute-effective-noise",
              ("Evening light. The robot standing still, looking along the floor and a wall.",
               "Two minutes without moving; stop when done.")),
    Procedure("turn-lag", "Turning on the spot (frame lag)", "timing.md#2-the-frames-against-odometry-from-turning",
              ("Clear space round the robot.", "Turn on the spot a full turn each way at walking turn speed.",
               "Stop when done.")),
    Procedure("taped-square", "Five laps of the taped square (odometry)", "odometry.md#2-five-laps-of-a-square-the-heading-and-position-drift",
              ("Start on the taped cross, heading along the taped line.", "Walk five laps, stopping on the start cross each lap.",
               "Stop, then measure where the robot stands against the cross and enter it."),
              needs_truth=("forward_cm", "left_cm", "heading_deg")),
    Procedure("power", "Walking power", "power_and_pose.md#1-walking-power-walk_power_w-direct",
              ("A full battery, rested 10 minutes.", "Stand still 5 minutes, then walk the room 20 minutes.",
               "Stop when done.")),
)
BY_ID = {p.id: p for p in PROCEDURES}


@dataclass
class ProcedureState:
    catalog: list[dict[str, Any]] = field(default_factory=lambda: [
        dict(id=p.id, title=p.title, doc=p.doc, steps=list(p.steps), needs_truth=list(p.needs_truth)) for p in PROCEDURES])
    active: str | None = None
    run: int = 0                       # which run the result belongs to: a write must name it
    phase: str = "idle"                # idle | recording | done
    elapsed_s: float = 0.0
    live: dict[str, Any] = field(default_factory=dict)
    result: dict[str, Any] = field(default_factory=dict)
    current: dict[str, Any] = field(default_factory=dict)     # the values in params/robot.yaml the result would replace
    writes: list[str] = field(default_factory=list)          # parameters --write would set
    message: str = ""


@dataclass
class LinkStatus:
    camera_fps: float | None = None
    drop: float | None = None
    stalls_per_min: float | None = None
    odom_hz: float | None = None
    battery_soc: float | None = None
    battery_v: float | None = None
    power_w: float | None = None
    isolation: str = "not checked"


def _rate(ts: np.ndarray) -> float | None:
    return round(float((len(ts) - 1) / (ts[-1] - ts[0])), 1) if len(ts) > 2 and ts[-1] > ts[0] else None


class Buffers:
    """the robot's streams since a procedure started (all), and the last minutes for the Status page"""

    def __init__(self) -> None:
        self.odom: deque[tuple[float, float, float, float]] = deque()
        self.frames: deque[float] = deque()
        self.grey: deque[tuple[float, np.ndarray]] = deque(maxlen=900)     # half-size grey frames (lag, noise)
        self.battery: deque[tuple[float, float, float, float]] = deque()   # t, soc, V, W
        self.lock = threading.Lock()

    def trim(self, now: float) -> None:
        for d in (self.odom, self.frames, self.battery):
            while d and now - (d[0] if isinstance(d[0], float) else d[0][0]) > KEEP_S: d.popleft()

    def arrays(self, since: float) -> dict[str, np.ndarray]:
        with self.lock:
            o = np.array([r for r in self.odom if r[0] >= since]).reshape(-1, 4)
            f = np.array([t for t in self.frames if t >= since])
            b = np.array([r for r in self.battery if r[0] >= since]).reshape(-1, 4)
            g = [(t, im) for t, im in self.grey if t >= since]
        return dict(odom=o, frames=f, battery=b, grey=g)


def live_numbers(pid: str, a: dict[str, Any], t0: float, now: float) -> dict[str, Any]:
    """what the page shows while a procedure records"""
    m = _measure(); o, f, b = a["odom"], a["frames"], a["battery"]; out: dict[str, Any] = dict(frames=len(f))
    if len(o) > 1:
        out["walked_m"] = round(float(np.hypot(*np.diff(o[:, 1:3], axis=0).T).sum()), 2)
        out["turned_deg"] = round(float(np.degrees(np.abs(np.diff(np.unwrap(o[:, 3]))).sum())), 1)
        out["from_start_m"] = round(float(np.hypot(*(o[-1, 1:3] - o[0, 1:3]))), 2)
    if pid == "stream" and len(f) > 10:
        out.update({k: v for k, v in m.frame_gaps(f).items() if k in ("drop", "stall_per_min", "stall_s", "period_s")})
    if pid == "still-minute" and len(o) > 1:
        out["still_s"] = round(sum(t1 - t0_ for t0_, t1 in m.still_spans(o[:, 0], o[:, 1], o[:, 2], np.unwrap(o[:, 3]), min_s=1.0)), 1)
    if pid == "taped-square" and len(o) > 1:
        far = np.hypot(*(o[:, 1:3] - o[0, 1:3]).T) > 1.0; near = np.hypot(*(o[:, 1:3] - o[0, 1:3]).T) < 0.3
        out["laps"] = int(np.sum(near[1:] & ~near[:-1] & np.maximum.accumulate(far)[1:]))
    if pid == "power" and len(b):
        out["soc"] = float(b[-1, 1]); out["power_w"] = round(float(b[-60:, 3].mean()), 1); out["used_pct"] = round(float(b[0, 1] - b[-1, 1]), 1)
    return out


def noise_from_frames(grey: list[tuple[float, np.ndarray]]) -> dict[str, float] | None:
    """measure.camera_noise's fit on frames already in memory: per brightness level, the variance of consecutive
    frames' difference, fitted as shot x level + read^2 (levels 0..1; halved frames, so lower than full-size)"""
    lv, vv = [], []
    for (_, a), (_, b) in zip(grey, grey[1:]):
        a = a.astype(np.float32) / 255; b = b.astype(np.float32) / 255
        lv.append(((a + b) / 2).ravel()); vv.append(((a - b) ** 2 / 2).ravel())
    if not lv: return None
    lv, vv = np.concatenate(lv), np.concatenate(vv); L, V = [], []
    for lo, hi in zip(np.linspace(0.05, 0.9, 18)[:-1], np.linspace(0.05, 0.9, 18)[1:]):
        k = (lv >= lo) & (lv < hi)
        if k.sum() > 2000: L.append(lv[k].mean()); V.append(np.median(vv[k]) / 0.4549)
    if len(L) < 3: return None
    shot, read2 = np.polyfit(L, V, 1)
    return dict(shot_noise=round(float(max(shot, 0)), 6), read_noise=round(float(math.sqrt(max(read2, 0))), 5))


def finish(pid: str, a: dict[str, Any], truth: dict[str, float]) -> tuple[dict[str, Any], dict[str, Any], str]:
    """the procedure's result, the parameter values it would write ({path: value}), and a note"""
    m = _measure(); o, f, b = a["odom"], a["frames"], a["battery"]
    if pid == "stream":
        if len(f) < 100: return {}, {}, "too few frames: record longer"
        r = m.frame_gaps(f)
        return r, {"go2_air.video_stream": {k: r[k] for k in ("drop", "stall_per_min", "stall_s")}}, ""
    if pid == "still-minute":
        r = noise_from_frames(a["grey"][-60:])
        if r is None: return {}, {}, "too few still frames"
        return r, {}, "effective noise: compare with a sim night of a like scene before tuning (camera.md 1)"
    if pid == "turn-lag":
        if len(o) < 20 or len(a["grey"]) < 50: return {}, {}, "too little turning recorded"
        sys.path.insert(0, str(HK / "src")); from houndkeeper import calibrate
        lag, c = calibrate.frame_lag(a["grey"], o[:, 0], np.unwrap(o[:, 3]))
        return dict(frame_lag_vs_odom_s=None if lag is None else round(lag, 3), correlation=round(float(c), 3)), {}, \
            "frame delay = the screen-clock delay (timing.md 1); odometry's own delay = that minus this lag"
    if pid == "taped-square":
        if len(o) < 20: return {}, {}, "no walk recorded"
        if not all(k in truth for k in BY_ID[pid].needs_truth): return {}, {}, "enter where the robot stands against the cross"
        walked = float(np.hypot(*np.diff(o[:, 1:3], axis=0).T).sum())
        th = -o[0, 3]; c, s = math.cos(th), math.sin(th); d = o[-1, 1:3] - o[0, 1:3]
        odom_fwd, odom_left = c * d[0] - s * d[1], s * d[0] + c * d[1]                       # the odometry's end, start frame
        err = math.hypot(odom_fwd - truth["forward_cm"] / 100, odom_left - truth["left_cm"] / 100)
        dyaw = math.degrees(math.remainder(o[-1, 3] - o[0, 3], 2 * math.pi)) - truth["heading_deg"]
        r = dict(walked_m=round(walked, 2), end_error_m=round(err, 3), xy_per_root_m=round(err / math.sqrt(walked), 4),
                 heading_error_deg=round(dyaw, 2), yaw_deg_per_root_m=round(abs(dyaw) / math.sqrt(walked), 3))
        return r, {}, "odom_drift combines several walks (odometry.md 3): kept in calibration-runs.jsonl"
    if pid == "power":
        if len(b) < 120 or len(o) < 20: return {}, {}, "record at least a few minutes with the battery stream"
        odom = (o[:, 0], o[:, 1], o[:, 2], np.unwrap(o[:, 3]))
        r = m.power_split(b[:, 0], b[:, 1], b[:, 3], odom, 236.8)
        writes = {f"go2_air.{k}": r[k] for k in ("walk_power_W", "stand_power_W") if r.get(k) is not None}
        return r, writes, ""
    return {}, {}, f"unknown procedure {pid}"


def _finite(v: Any) -> bool:
    if isinstance(v, dict): return all(_finite(x) for x in v.values())
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def current_values(writes: dict[str, Any]) -> dict[str, Any]:
    import yaml
    d = yaml.safe_load((HK / "params" / "robot.yaml").read_text()); out = {}
    for path in writes:
        g, k = path.split(".")[:2]; ent = (d.get(g) or {}).get(k)
        out[path] = None if ent is None else dict(value=ent.get("value"), source=ent.get("source"))
    return out


class CockpitData(Module):
    odom: In[PoseStamped]
    color_image: In[Image]
    battery: In[Battery]
    procedure_cmd: In[dict]
    procedure_state: Out[ProcedureState]
    link_status: Out[LinkStatus]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._buf = Buffers(); self._state = ProcedureState(); self._t0 = 0.0; self._truth: dict[str, float] = {}
        self._pending: dict[str, Any] = {}; self._pub_stop = threading.Event(); self._pub_thread: threading.Thread | None = None
        self._frame_n = 0; self._runs = 0

    @rpc
    def start(self) -> None:
        super().start()
        for stream, cb in ((self.odom, self._on_odom), (self.color_image, self._on_image), (self.battery, self._on_battery),
                           (self.procedure_cmd, self._on_cmd)):
            self.register_disposable(Disposable(stream.subscribe(cb)))
        self._pub_stop.clear(); self._pub_thread = threading.Thread(target=self._publish_loop, name="cockpit-data", daemon=True); self._pub_thread.start()

    @rpc
    def stop(self) -> None:
        self._pub_stop.set(); super().stop()

    def _on_odom(self, msg: PoseStamped) -> None:
        q = msg.orientation; yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        with self._buf.lock: self._buf.odom.append((float(msg.ts), float(msg.position.x), float(msg.position.y), yaw))

    def _on_image(self, msg: Image) -> None:
        t = float(msg.ts)
        with self._buf.lock: self._buf.frames.append(t)
        self._frame_n += 1
        if self._state.phase == "recording" and self._state.active in ("still-minute", "turn-lag"):
            a = np.asarray(msg.as_numpy() if hasattr(msg, "as_numpy") else msg.data)
            g = (a.mean(-1) if a.ndim == 3 else a)[::2, ::2].astype(np.uint8)
            with self._buf.lock: self._buf.grey.append((t, g))

    def _on_battery(self, msg: Battery) -> None:
        with self._buf.lock: self._buf.battery.append((float(msg.ts), float(msg.soc), float(msg.voltage), float(msg.power)))

    def _on_cmd(self, cmd: Any) -> None:
        try: self.handle(cmd if isinstance(cmd, dict) else json.loads(cmd))
        except Exception as e: self._state.message = f"command failed: {e}"

    def handle(self, cmd: dict[str, Any], now: float | None = None) -> None:
        now = clock.now() if now is None else now; op = cmd.get("op"); s = self._state
        if op == "start" and cmd.get("id") in BY_ID and s.phase != "recording":
            self._runs = getattr(self, "_runs", 0) + 1
            self._state = ProcedureState(active=cmd["id"], run=self._runs, phase="recording"); self._t0 = now; self._truth = {}; self._pending = {}
            with self._buf.lock: self._buf.grey.clear()
        elif op == "stop" and s.phase == "recording":
            s.phase = "done"; self._conclude(now)
        elif op == "truth" and s.phase == "done":
            got = {k: float(v) for k, v in cmd.items() if k in BY_ID[s.active].needs_truth}
            bad = [k for k, v in got.items() if not (math.isfinite(v) and abs(v) <= TRUTH_LIMITS.get(k, 1e3))]
            if bad: s.message = "not a measurement: " + ", ".join(bad); return
            self._truth.update(got); self._conclude(now)
        elif op == "write" and s.phase == "done" and self._pending:
            if cmd.get("run") != s.run:
                s.message = "not written: the page confirmed a different run (reload the page)"; return
            m = _measure(); how = f"the phone cockpit's {s.active} procedure, {time.strftime('%Y-%m-%d %H:%M')}"
            params = HK / "params" / "robot.yaml"
            for path, value in self._pending.items():
                if path.endswith("stand_power_W"): m.set_param(path, value, how, params=params, add_after="walk_power_W")
                else: m.set_param(path, value, how, params=params)
            s.message = "written to params/robot.yaml: " + ", ".join(self._pending); s.current = current_values(self._pending)
            self._pending = {}; s.writes = []
        elif op == "reset":
            self._state = ProcedureState()

    def _conclude(self, now: float) -> None:
        s = self._state; a = self._buf.arrays(self._t0)
        result, writes, note = finish(s.active, a, self._truth)
        bad = [k for k, v in writes.items() if not _finite(v)]
        if bad: writes = {k: v for k, v in writes.items() if k not in bad}; note = (note + " " if note else "") + "not finite, not offered: " + ", ".join(bad)
        s.result = result; s.message = note; s.writes = list(writes); self._pending = writes
        s.current = current_values(writes) if writes else {}
        if result:
            RUNS_LOG.parent.mkdir(parents=True, exist_ok=True)
            with RUNS_LOG.open("a") as fh:
                fh.write(json.dumps(dict(when=time.strftime("%Y-%m-%dT%H:%M:%S"), id=s.active, started=self._t0, ended=now,
                                         result=result, truth=self._truth, would_write=writes), default=float) + "\n")

    def link(self, now: float) -> LinkStatus:
        a = self._buf.arrays(now - LINK_WINDOW_S); f, o, b = a["frames"], a["odom"], a["battery"]; st = LinkStatus()
        if len(f) > 10:
            g = _measure().frame_gaps(f); st.camera_fps = _rate(f); st.drop = g["drop"]; st.stalls_per_min = g["stall_per_min"]
        st.odom_hz = _rate(o[:, 0]) if len(o) else None
        if len(b): st.battery_soc, st.battery_v, st.power_w = float(b[-1, 1]), round(float(b[-1, 2]), 2), round(float(b[-1, 3]), 1)
        iso = HK / "real" / "local" / "isolation.json"
        if iso.exists():
            try:
                summary = json.loads(iso.read_text()).get("summary", "recorded"); age = now - iso.stat().st_mtime
                st.isolation = summary if isinstance(summary, str) else json.dumps(summary)[:80]
                if st.isolation.startswith("passed") and age > ISOLATION_MAX_AGE_S: st.isolation = f"stale: {st.isolation} ({age / 3600:.0f} h ago)"
            except Exception: st.isolation = "unreadable"
        return st

    def _publish_loop(self) -> None:
        while not self._pub_stop.wait(PUBLISH_S):
            now = clock.now()
            with self._buf.lock: self._buf.trim(now)
            s = self._state
            if s.phase == "recording":
                s.elapsed_s = round(now - self._t0, 1)
                try: s.live = live_numbers(s.active, self._buf.arrays(self._t0), self._t0, now)
                except Exception as e: s.message = f"live numbers failed: {e}"
            self.procedure_state.publish(s); self.link_status.publish(self.link(now))


class MindFeed(Module):
    """the mind summary ([ULM]) onto the cockpit's mind channel: the tidy module publishes it as JSON text, the form
    --record keeps ([Fho]), so a recorded night replays through here as a live one does ([B4u]). Anything that isn't a
    version-1 message within mind.MAX_BYTES is dropped."""
    mind_json: In[String]
    mind: Out[dict]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.mind_json.subscribe(self._on_json)))

    def _on_json(self, msg: Any) -> None:
        m = mind.decode(getattr(msg, "data", msg))
        if m is not None: self.mind.publish(m)


MORNING_RUNS = HK / "data" / "runs" / "dimos_tidy"   # where the nights' logs are (sim: a tidy run's directory each)
MORNING_CHECK_S = 30.0                               # how often the feed looks for a newer night's log


class MorningLogFeed(Module):
    """the latest night's morning log ([uoW]) onto the cockpit's morning_log channel for the Morning page ([Ndt]):
    morning_log.morning_message, its crops JPEG base64 and the whole at most morning_log.PHONE_MAX_BYTES. Published at
    start and again only when a newer log is written (looked for every MORNING_CHECK_S); the channel resends the last to
    a page that opens later. Its root: HK_MORNING_RUNS, else MORNING_RUNS and the real nights ([q66])."""
    morning_log: Out[dict]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event(); self._seen: tuple | None = None

    @rpc
    def start(self) -> None:
        super().start(); self._stop.clear()
        threading.Thread(target=self._watch, name="morning-log", daemon=True).start()

    @rpc
    def stop(self) -> None:
        self._stop.set(); super().stop()

    def root(self) -> Path | tuple[Path, ...]:
        """HK_MORNING_RUNS when it names one place, else the sim's tidy runs and the real nights ([q66])"""
        import os
        if os.environ.get("HK_MORNING_RUNS"): return Path(os.environ["HK_MORNING_RUNS"])
        from houndkeeper.real_night import nights_root
        return (MORNING_RUNS, nights_root())

    def check(self) -> dict | None:
        """the message if the latest log is new since the last look (its directory and the log's time), else None"""
        from houndkeeper import morning_log
        d = morning_log.latest_night(self.root())
        if d is None: return None
        key = (str(d), (d / "morning_log.json").stat().st_mtime)
        if key == self._seen: return None
        m = morning_log.morning_message(self.root())
        self._seen = key
        if m is not None: self.morning_log.publish(m)
        return m

    def _watch(self) -> None:
        while True:
            try: self.check()
            except Exception as e: print(f"[morning-log] {e}", file=sys.stderr)
            if self._stop.wait(MORNING_CHECK_S): return
