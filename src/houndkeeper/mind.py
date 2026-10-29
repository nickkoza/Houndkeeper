"""What Barrymore thinks, live ([RNC]): the night's state as one small message, a few times a second, for the phone's
Mind page (cockpit/extensions/hk-mind.js). The tidy module publishes it as JSON text (TidyArm.mind_json), the form
DimOS's --record keeps, so a recorded night replays it like a live one ([B4u]); cockpit_data.MindFeed hands it on.

[ULM] Built from the modules' own state (TidyArm._mind_state: references, nothing recomputed); the work here is only
choosing, rounding and projecting a few dozen points. [1XG] Every reason is the deciding module's words, passed
through and cut to WHY_CHARS; this file writes none of its own. At most MAX_BYTES of compact JSON, PERIOD_S apart.

Schema (v 1):
  v       1
  t       when (unix s)
  stage   sweep | look closer | pick | put away | dock | starting | idle | done | failed
  step    the module's own name for what it is doing (the loop's state, or the pick's phase: "INTO BIN")
  note    the module's latest note
  pose    [x, y, yaw]: m, m, rad, in odometry's frame (the costmap's)
  goal    {x, y, kind: waypoint | find | sidestep, why} or null
  route   {walked: [[x, y], ...], plan: [[x, y], ...]}: walked thinned to WALKED_SENT; plan, the sweep's waypoints to go
  cands   [{id, x, y, r, status: new | looked at | ruled out | picked, conf: 0-1 or null, why, label}]
  more    candidates left out to fit
  arm     {state, item}
  view    [{id, ring: [[u, v], ...]}]: candidates the head camera sees, outlined on the floor, u and v as fractions of
          the frame's width and height (the raw frame, through the lens's own distortion)"""
from __future__ import annotations

import json
import math
from typing import Any, Callable

import numpy as np

V = 1
PERIOD_S = 1 / 3                 # three a second
MAX_BYTES = 8192                 # a message never exceeds this (fit); typical nights 2-4 KB
MAX_CANDS = 40
WALKED_KEEP, WALKED_SENT, WALKED_STEP_M = 400, 120, 0.25
MAX_PLAN = 60
MAX_VIEW = 12
WHY_CHARS, LABEL_CHARS, NOTE_CHARS = 100, 40, 120
SAME_SPOT_M = 0.15               # a stop and the sweep's record of it: the same spot (the tidy module's spot key)
VIEW_MAX_AGE_S = 1.0             # the head camera's pose older than this (no frame judged lately): no outlines
RING = 8                         # points round each outline

STAGE = {"SWEEP": "sweep", "LOOK AGAIN": "sweep", "SIDESTEP": "sweep", "finding items": "sweep",
         "finds from the sweep": "sweep",
         "GO TO FIND": "look closer", "LOOK STANDING": "look closer", "LIE DOWN": "look closer",
         "LOOK CLOSER: find and name": "look closer",
         "GRASP": "pick", "STAND UP": "pick", "INTO BIN": "put away", "STOW": "put away", "DOCK": "dock",
         "starting": "starting", "idle": "idle", "DONE": "done", "done": "done", "failed": "failed"}
ARM = {"LOOK STANDING": "looking", "LOOK CLOSER: find and name": "looking", "GRASP": "grasping",
       "INTO BIN": "into the bin", "STOW": "stowing"}
NOTHING_THERE = ("not seen at the flag", "not re-seen", "not seen standing")   # TidyArm._settled's cleared outcomes
STATUSES = ("new", "looked at", "ruled out", "picked")


def _num(v: Any, nd: int = 2) -> float | None:
    try: f = float(v)
    except (TypeError, ValueError): return None
    return round(f, nd) if math.isfinite(f) else None


def _text(v: Any, n: int) -> str | None:
    if v is None: return None
    s = str(v)
    return s if len(s) <= n else s[:n - 3] + "..."


def stage_of(state: str | None, phase: str | None) -> tuple[str, str]:
    """(the spec's stage, the module's own step name)"""
    step = phase or state or "idle"
    return STAGE.get(step, STAGE.get(state or "", str(step).lower())), step


def arm_of(step: str, stage: str, sweep_arm: bool, item: str | None) -> dict[str, Any]:
    state = ARM.get(step) or ("raised to look" if sweep_arm and stage == "sweep" else "stowed")
    return {"state": state, "item": _text(item, LABEL_CHARS) if state in ("grasping", "into the bin") else None}


def outcome_status(rep: dict[str, Any]) -> tuple[str, str | None]:
    """a stop's status and its reason, from the tidy module's report of it"""
    o = rep.get("outcome")
    if o == "binned": return "picked", o
    if o is not None:
        o = str(o)
        if o.startswith("same as") or any(s in o for s in NOTHING_THERE): return "ruled out", o
        return "looked at", o                                     # left for review: looked at, not taken
    seen = rep.get("label") is not None or " cm from the flag" in str(rep.get("standing", ""))
    why = rep.get("standing") or rep.get("refine") or (rep.get("confirm") or {}).get("why")
    return ("looked at" if seen else "new"), why


def _conf(rec: dict[str, Any] | None) -> float | None:
    """the confirmer's own measure: the share of the frames that saw the spot which found it changed"""
    if not rec or not rec.get("seen"): return None
    return _num(min(1.0, rec.get("hits", 0) / rec["seen"]))


def _radius(rec: dict[str, Any] | None) -> float:
    a = (rec or {}).get("area_cm2")
    return _num(max(0.04, math.sqrt(a / math.pi) / 100), 3) if a else 0.08


def candidates(status: dict[str, Any], sweep: list[dict] | None, worth: Callable[[dict], bool] | None) -> list[dict]:
    """every find the night holds: its stops (newest first), then the sweep's records not stopped for, newest first"""
    items = list(status.get("items") or []); recs = [r for r in list(sweep or []) if "decision" in r and "x" in r]
    used: set[int] = set(); out = []
    for k in range(len(items), 0, -1):
        rep = items[k - 1]; c = rep.get("confirm") or {}; xy = (c.get("x"), c.get("y")) if "x" in c else tuple((rep.get("flag") or [None, None])[:2])
        rec = None
        if xy[0] is not None:
            for i in range(len(recs) - 1, -1, -1):
                if i not in used and math.hypot(recs[i]["x"] - xy[0], recs[i]["y"] - xy[1]) <= SAME_SPOT_M:
                    used.add(i); rec = rec or recs[i]
        st, why = outcome_status(rep)
        conf = _num(rep.get("score")) if rep.get("score") is not None else _conf(rec)
        out.append({"id": f"i{k}", "x": _num(xy[0]), "y": _num(xy[1]), "r": _radius(rec), "status": st, "conf": conf,
                    "why": _text(why, WHY_CHARS), "label": _text(rep.get("label"), LABEL_CHARS)})
    flags = list(status.get("flags") or [])[len(items):]         # flags given (or seen) still to visit
    for j, f in enumerate(flags, len(items) + 1):
        out.append({"id": f"f{j}", "x": _num(f[0]), "y": _num(f[1]), "r": 0.08, "status": "new", "conf": None,
                    "why": None, "label": None})
    seen_keys: set[tuple[int, int]] = set(); rest_new, rest_out = [], []
    for i in range(len(recs) - 1, -1, -1):                        # the latest record of each spot
        r = recs[i]; key = (round(r["x"] / SAME_SPOT_M), round(r["y"] / SAME_SPOT_M))
        if i in used or key in seen_keys: continue
        seen_keys.add(key); d = r.get("decision")
        stop = d in ("unsure", "look", "escalated") or (d == "confirmed" and (worth is None or worth(r)))
        why = r.get("why") or r.get("why_not") or str(d).replace("_", " ")
        (rest_new if stop else rest_out).append({"id": f"s{i + 1}", "x": _num(r["x"]), "y": _num(r["y"]), "r": _radius(r),
                                                 "status": "new" if stop else "ruled out", "conf": _conf(r),
                                                 "why": _text(why, WHY_CHARS), "label": None})
    return out + rest_new + rest_out


def project(pts: np.ndarray, T: np.ndarray, K, D, wh) -> np.ndarray:
    """world points (n x 3) into the head camera's raw frame, as fractions of its width and height (n x 2); NaN where
    behind the camera. T: world <- optical (x right, y down, z ahead); K: (fx, fy, cx, cy); D: the fisheye's four
    equidistant coefficients (OpenCV's model), or None for a pinhole"""
    T = np.asarray(T, float); p = (np.asarray(pts, float) - T[:3, 3]) @ T[:3, :3]
    fx, fy, cx, cy = K; W, H = wh; out = np.full((len(p), 2), np.nan)
    ok = p[:, 2] > 1e-6
    a, b = p[ok, 0] / p[ok, 2], p[ok, 1] / p[ok, 2]
    if D is not None:
        k1, k2, k3, k4 = (list(np.asarray(D, float).ravel()) + [0.0] * 4)[:4]
        r = np.hypot(a, b); th = np.arctan(r); th2 = th * th
        thd = th * (1 + k1 * th2 + k2 * th2 ** 2 + k3 * th2 ** 3 + k4 * th2 ** 4)
        s = np.where(r > 1e-9, thd / np.maximum(r, 1e-9), 1.0); a, b = a * s, b * s
    out[ok, 0] = (fx * a + cx) / W; out[ok, 1] = (fy * b + cy) / H
    return out


def in_view(cands: list[dict], view: Any, now: float) -> list[dict]:
    """the candidates the head camera sees now: each outline (a ring on the floor at its radius) in the frame"""
    if not view: return []
    T, K, D, wh, t = view
    if now - t > VIEW_MAX_AGE_S: return []
    out = []; ang = np.linspace(0, 2 * np.pi, RING, endpoint=False)
    for c in cands:
        if c["x"] is None or c["status"] == "ruled out": continue
        ring = np.stack([c["x"] + c["r"] * np.cos(ang), c["y"] + c["r"] * np.sin(ang), np.zeros(RING)], 1)
        uv = project(ring, T, K, D, wh)
        if np.isnan(uv).any() or not ((uv >= 0) & (uv <= 1)).all(axis=1).any(): continue
        out.append({"id": c["id"], "ring": np.round(uv, 3).tolist()})
        if len(out) >= MAX_VIEW: break
    return out


class Walked:
    """the route walked, kept thin: a point every WALKED_STEP_M; past WALKED_KEEP, every other one dropped"""

    def __init__(self) -> None:
        self.pts: list[tuple[float, float]] = []

    def add(self, x: float, y: float) -> None:
        if self.pts and math.hypot(x - self.pts[-1][0], y - self.pts[-1][1]) < WALKED_STEP_M: return
        self.pts.append((x, y))
        if len(self.pts) > WALKED_KEEP: self.pts = self.pts[:-1:2] + [self.pts[-1]]

    def sent(self, n: int = WALKED_SENT) -> list[list[float]]:
        p = self.pts
        if len(p) > n: p = [p[int(i)] for i in np.linspace(0, len(p) - 1, n)]
        return [[round(x, 2), round(y, 2)] for x, y in p]


def _yaw(q) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def summary(state: dict[str, Any], walked: Walked, now: float) -> dict[str, Any]:
    """the message, from TidyArm._mind_state's dict: status, sweep, goal, route, route_at, phase, note, pose (x, y, qw,
    qx, qy, qz), view (T, K, D, (W, H), t), worth (the module's stop gate), sweep_arm"""
    st = state.get("status") or {}
    stage, step = stage_of(st.get("state"), state.get("phase"))
    pose = state.get("pose")
    if pose is not None: walked.add(float(pose[0]), float(pose[1]))
    g = state.get("goal")
    goal = None if g is None or stage in ("done", "failed", "idle") else {
        "x": _num(g.get("x")), "y": _num(g.get("y")), "kind": g.get("kind"), "why": _text(g.get("why"), WHY_CHARS)}
    route = list(state.get("route") or [])[int(state.get("route_at") or 0):][:MAX_PLAN]
    cands = candidates(st, state.get("sweep"), state.get("worth"))
    items = st.get("items") or []
    current = items[-1] if items and items[-1].get("outcome") is None else {}
    return {"v": V, "t": round(now, 2), "stage": stage, "step": _text(step, LABEL_CHARS),
            "note": _text(state.get("note") or "", NOTE_CHARS),
            "pose": None if pose is None else [_num(pose[0]), _num(pose[1]), _num(_yaw(pose[2:6]), 3)],
            "goal": goal, "route": {"walked": walked.sent(), "plan": [[_num(w[0]), _num(w[1])] for w in route]},
            "cands": cands, "more": 0, "arm": arm_of(step, stage, bool(state.get("sweep_arm")), current.get("label")),
            "view": in_view(cands, state.get("view"), now)}


def encode(msg: dict[str, Any]) -> str:
    return json.dumps(msg, separators=(",", ":"), allow_nan=False)


def fit(msg: dict[str, Any]) -> str:
    """the message as JSON within MAX_BYTES: candidates beyond MAX_CANDS left out (counted in more), then, while too
    big, the walked route thinned, outlines, the plan, then candidates from the end (the sweep's ruled-out first)"""
    msg = dict(msg); c = msg["cands"]
    if len(c) > MAX_CANDS: msg["more"] += len(c) - MAX_CANDS; msg["cands"] = c[:MAX_CANDS]
    s = encode(msg)
    while len(s) > MAX_BYTES:                                     # ASCII (json escapes the rest): characters are bytes
        r = msg["route"]
        if len(r["walked"]) > 16: msg["route"] = dict(r, walked=r["walked"][::2] + ([r["walked"][-1]] if len(r["walked"]) % 2 == 0 else []))
        elif msg["view"]: msg["view"] = msg["view"][:-1]
        elif len(r["plan"]) > 8: msg["route"] = dict(r, plan=r["plan"][:len(r["plan"]) // 2])
        elif msg["cands"]: msg["cands"] = msg["cands"][:-1]; msg["more"] += 1
        else: msg["note"] = ""; msg["route"] = {"walked": [], "plan": []}; s = encode(msg); break
        s = encode(msg)
    return s


def decode(text: Any) -> dict[str, Any] | None:
    """a mind message from its JSON text, or None if it isn't one (too big, not version 1)"""
    if not isinstance(text, (str, bytes)) or len(text) > MAX_BYTES: return None
    try: m = json.loads(text)
    except ValueError: return None
    return m if isinstance(m, dict) and m.get("v") == V else None


class Mind:
    """the summary's own memory between messages (the route walked); json() is what the tidy module publishes"""

    def __init__(self) -> None:
        self.walked = Walked()

    def json(self, state: dict[str, Any], now: float | None = None) -> str:
        from dimos.utils import clock   # the robot's clock: sim time in the sim ([3Pv])
        return fit(summary(state, self.walked, clock.now() if now is None else now))
