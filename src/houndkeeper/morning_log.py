#!/usr/bin/env python3
"""The butler's morning log ([uoW]): a finished night turned into Barrymore's report to the household, from the night's
own outputs ([9X9]), in his voice from templates over the night's facts ([W0w], [ZNG]), with a picture, time and place
for each find ([hKP]) and the night's engineering figures one tap down, each linked to its source file ([6wJ]).

A night is a tidy run's directory (sim: data/runs/dimos_tidy/<stamp>): report.json (the robot's own account: each stop,
what it saw, what it did), dimos.log (the tidy's timed lines), traj.csv (only its odometry columns), energy.json (the
battery), looks/ (the close and standing looks' images), house_map/ (the map he started from) and recording.txt (which
recording the night is). truth.json, score.json, score.txt and traj.csv's true_* columns are the sim's truth: the
household's part never opens them; morning_score.py writes the same log with a section scored against them, for a sim
night only, marked as such.

Pictures come only from a recording the catalog clears for its frames ([R1X]: recordings.require_screened; a recording
the catalog doesn't know gets no pictures), and every crop goes through a people screen before it is kept ([hKP]): on a
real night tools/screen_people.py's detector, on a sim night none (the sim houses hold no people).

night.json (the tidy writes it, tidy_run.sh: write_night_json) names the night: its seed, mode, item seed and round,
the night_tidy folder, and when it began and ended; route.json beside it is the route he was given. Only NIGHT_KEYS are
allowed in it, so it carries nothing of the sim's truth: a night.json with any other key is refused.

Writes morning_log.html (self-contained: no scripts, no fetches; crops as JPEG) and morning_log.json (every fact the
prose uses, with its source) into the night's directory ([t0a]) or --out. morning_message() is the same log for the
phone cockpit's Morning page ([Ndt]): the latest night's, crops as JPEG base64, at most PHONE_MAX_BYTES.
Usage: morning_log.py <night dir> [--out DIR]   (dimos venv python)"""
from __future__ import annotations

import argparse, base64, datetime as dt, html, io, json, math, os, re, shutil, sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HK / "src"))
from houndkeeper import recordings   # noqa: E402

TRUTH_FILES = ("truth.json", "score.json", "score.txt")   # the sim's truth: never opened for the household's part
ODOM = ("wall_t", "odom_x", "odom_y", "odom_yaw")         # traj.csv's own columns (its true_* are the sim's)
NIGHT_KEYS = ("seed", "mode", "item_seed", "round", "night_tidy", "started", "ended", "route")   # night.json's only keys
NIGHT_ENV = {"seed": "NIGHT_SEED", "mode": "NIGHT_MODE", "item_seed": "NIGHT_ITEM_SEED", "round": "NIGHT_ROUND",
             "night_tidy": "NIGHT_TIDY_DIR", "started": "NIGHT_STARTED", "ended": "NIGHT_ENDED"}   # what the tidy passes
PHONE_MAX_BYTES = 384 * 1024    # the phone's message, compact JSON: never more (crops thinned, then dropped, to fit)
PHONE_CROP_PX, PHONE_JPEG_Q = 240, 70                    # the phone's crops: smaller than the file's (FILE_CROP_PX, quality 85)
CROP_FRAC = 300 / 640       # a find's crop: this share of its picture's width (300 px of the 640 x 360 looks it was set on;
                            # the same view of the floor at any frame size, [7cU]: 600 px of a 1280 x 720 look)
FILE_CROP_PX = 300          # the file's crops are shown at this size, whatever the frame they were cut from


def crop_side(W) -> int:
    """the side of a find's crop in a picture W px wide (CROP_FRAC of it)"""
    return max(1, round(CROP_FRAC * W))
NUM = "no one two three four five six seven eight nine ten eleven twelve".split()
STAGES = {"SWEEP": "walking the rounds", "GO TO FIND": "going to see", "LOOK STANDING": "looking from standing",
          "LIE DOWN": "lying down to reach", "LOOK CLOSER": "looking closer", "GRASP": "taking hold", "INTO BIN": "into the bin",
          "STOW": "stowing the arm", "STAND UP": "getting up"}
# what became of a stop, from the robot's own outcome: (key, the group's sentence for one, for several, the card's line)
FATES = [("binned", "put_away", "{L|cap} I put in the bin.", "{c|cap} I put in the bin: {L}.", "Put in the bin."),
         ("not seen standing", "nothing", "", "", "Nothing there."),
         ("lost on the way to the bin", "lost_en_route", "{L|cap} I took up but lost on the way to the bin; it may be in the "
          "bin, and I should be grateful if you would look.", "{c|cap}, {L}, I took up but lost on the way to the bin; they may "
          "be in it, and I should be grateful if you would look.", "Taken up, but lost on the way to the bin. It may be in it."),
         ("not seen again after a failed try", "slipped", "{L|cap} I tried for once, and could not see again after.",
          "{c|cap}, {L}, I tried for once each, and could not see again after.", "Tried for once; not seen again after."),
         ("not seen at the flag", "not_found_again", "{L|cap} I glimpsed, but it was not where I looked when I came close.",
          "{c|cap} things I glimpsed were not where I looked when I came close: {L}.", "Glimpsed; not where I looked when I came close."),
         ("no reachable grip", "no_grip", "{L|cap} I could find no way to take hold of; it is where it was.",
          "{c|cap}, {L}, I could find no way to take hold of; they are where they were.", "No way to take hold of it. It is where it was."),
         ("grasp", "not_lifted", "{L|cap} would not come up off the floor, try as I might.",
          "{c|cap}, {L}, would not come up off the floor, try as I might.", "It would not come up off the floor."),
         ("too ", "too_big", "{L|cap} is too large for my grip; I left it where it lies.",
          "{c|cap}, {L}, are too large for my grip; I left them where they lie.", "Too large for my grip. Left where it lies."),
         ("review", "other", "{L|cap} I left for you to see to.", "{c|cap}, {L}, I left for you to see to.", "Left for you to see to."),
         ("", "unfinished", "{L|cap} I had not finished with when the night ended.",
          "{c|cap}, {L}, I had not finished with when the night ended.", "Not finished with when the night ended.")]


class Night:
    """a finished night's directory, read only through its own outputs ([9X9]): a truth file is refused"""
    def __init__(self, d):
        self.dir = Path(d)

    def path(self, name: str) -> Path:
        if Path(name).name in TRUTH_FILES: raise PermissionError(f"{name} is the sim's truth: not for the household's part")
        return self.dir / name

    def json(self, name, default=None):
        p = self.path(name)
        if not p.exists(): return default
        t = p.read_text(); return json.loads(t[t.index("{"):])

    def odometry(self) -> np.ndarray:
        """traj.csv's own columns only: wall time, odometry x, y, yaw (N x 4); with no traj.csv (the sim writes it, a
        real robot does not, [Lzt]) the recording's own odometry stream; empty if there's neither"""
        p = self.path("traj.csv")
        if not p.exists():
            self.odometry_source = "recording.txt"
            db = self.recording_path()
            return recording_odometry(db) if db is not None and db.exists() else np.zeros((0, 4))
        self.odometry_source = "traj.csv"
        with open(p) as f: head = f.readline().strip().split(",")
        a = np.loadtxt(p, delimiter=",", skiprows=1, usecols=[head.index(c) for c in ODOM], ndmin=2)
        return a[np.isfinite(a).all(1)]

    def recording_path(self) -> Path | None:
        p = self.path("recording.txt")
        return Path(p.read_text().strip()) if p.exists() else None

    def record(self) -> dict:
        """night.json: the night's names and times, only NIGHT_KEYS ({} if there is none); any other key is refused, so
        nothing of the sim's truth can ride in on it"""
        d = self.json("night.json", {}) or {}
        bad = sorted(set(d) - set(NIGHT_KEYS))
        if bad: raise PermissionError(f"night.json holds {', '.join(bad)}: only {', '.join(NIGHT_KEYS)} are allowed")
        return d

    def recording_id(self) -> str | None:
        p = self.path("recording.txt")
        return recordings.run_id_of(Path(p.read_text().strip())) if p.exists() else None

    def battery(self) -> tuple[float, float] | None:
        """the charge (%) at the recording's first and last battery reading, when it has the robot's battery stream
        (the real robot's, battery_module); None without one (the sim has no low state)"""
        db = self.recording_path()
        if db is None or not db.exists(): return None
        try: soc = recording_stream(db, "battery", lambda o: float(o.data.soc))
        except Exception: return None
        return (soc[0], soc[-1]) if soc else None

    def frame(self, ts: float):
        """the recording's head-camera frame stamped ts (RGB array), or None"""
        db = self.recording_path()
        if db is None or not db.exists(): return None
        from dimos.memory.store.sqlite import SqliteStore
        st = SqliteStore(path=str(db)); st.start()
        try:
            r = list(st.stream("color_image").time_range(ts - 1e-4, ts + 1e-4))
            if not r: return None
            im = r[0].data
            return np.asarray(im.to_rgb().data) if hasattr(im, "to_rgb") else np.asarray(getattr(im, "data", im))
        finally:
            st.stop()


def recording_stream(db, name, value) -> list:
    """value(o) of each message of a recording's stream, in order"""
    from dimos.memory.store.sqlite import SqliteStore
    st = SqliteStore(path=str(db)); st.start()
    try: return [value(o) for o in st.stream(name)]
    finally: st.stop()


def recording_odometry(db) -> np.ndarray:
    """a recording's odometry stream as (stamp, x, y, yaw) rows (N x 4), on the clock it was stamped on"""
    from dimos.memory.store.sqlite import SqliteStore
    st = SqliteStore(path=str(db)); st.start()
    try:
        rows = [(o.ts, o.data.position.x, o.data.position.y, o.data.yaw) for o in st.stream("odom")]
    finally:
        st.stop()
    a = np.array(rows, float).reshape(-1, 4)
    return a[np.isfinite(a).all(1)]


def _num(v):
    try: f = float(v); return int(f) if f.is_integer() else f
    except (TypeError, ValueError): return v


def write_night_json(out, env=None) -> Path:
    """the tidy's night.json in its run directory `out`, from NIGHT_ENV's variables (unset or empty: null), with the route
    it was given (NIGHT_ROUTE, a route.json) copied in beside it; only NIGHT_KEYS are ever written"""
    env = os.environ if env is None else env; out = Path(out)
    d = {k: (_num(env[v]) if k in ("seed", "item_seed", "started", "ended") else env[v]) if env.get(v) else None
         for k, v in NIGHT_ENV.items()}
    route = env.get("NIGHT_ROUTE")
    if route and Path(route).is_file():
        json.loads(Path(route).read_text())                            # a route, not a half-written file
        if Path(route).resolve() != (out / "route.json").resolve(): shutil.copyfile(route, out / "route.json")
        d["route"] = "route.json"
    else: d["route"] = None
    assert set(d) <= set(NIGHT_KEYS)
    (out / "night.json").write_text(json.dumps(d, indent=1)); return out / "night.json"


def kind_of(rid) -> str:
    """sim or real, as the recordings catalog has it; a recording it doesn't know is "unknown" (treated as unscreened)"""
    if not rid: return "unknown"
    try: return recordings.get(rid)["kind"]
    except SystemExit: return "unknown"


def tidy_lines(night, t0):
    """the tidy's timed lines from dimos.log, [(epoch s, text)]; the log stamps UTC time of day, placed by the report's t0"""
    p = night.path("dimos.log"); out = []
    if not p.exists() or t0 is None: return out
    day0 = t0 % 86400
    for line in open(p, errors="replace"):
        m = re.match(r"^(\d\d):(\d\d):(\d\d(?:\.\d+)?) .*?\[tidy\] (.*)$", line.rstrip())
        if not m: continue
        sod = int(m[1]) * 3600 + int(m[2]) * 60 + float(m[3])
        out.append((t0 + (sod - day0 + 43200) % 86400 - 43200, m[4]))
    return out


def stage_times(lines) -> dict:
    """seconds spent in each of the tidy's stages, from its stage lines ("SWEEP: ...", "state: GRASP")"""
    tot, cur, since = {}, None, None
    for t, text in lines:
        m = re.match(r"^(?:state: )?([A-Z][A-Z ]*[A-Z])(?::|$)", text)
        if not m: continue
        if cur is not None: tot[cur] = tot.get(cur, 0.0) + t - since
        cur, since = (None if m[1] == "DONE" else m[1]), t
    return tot


def fate(outcome) -> tuple:
    for key, name, one, many, card in FATES:
        if outcome is not None and key in outcome: return name, one, many, card
    return FATES[-1][1:]


def what(it) -> str:
    """what he took it for: his close look's name, else his standing look's, else "something" """
    if it.get("label"): return it["label"]
    m = re.match(r"standing look: (an? [^(]+?) \(", it.get("standing", ""))
    return m[1] if m else "something"


def finds(report, lines) -> list:
    """each stop the night made, in order: where (his flag, house frame), when he set off for it and was done, what he
    took it for, and what became of it"""
    t_find, t_item = {}, {}
    for t, text in lines:
        m = re.match(r"GO TO FIND: find (\d+) at", text)
        if m: t_find.setdefault(int(m[1]), t)
        m = re.match(r"item (\d+): (.*)", text)
        if m: t_item.setdefault(int(m[1]), []).append((t, m[2]))
    out = []
    for n, it in enumerate(report.get("items", []), 1):
        f, outcome = it.get("flag"), it.get("outcome")
        done = [t for t, s in t_item.get(n, []) if outcome and s == outcome]
        out.append({"n": n, "x": round(float(f[0]), 2), "y": round(float(f[1]), 2), "t": t_find.get(n, (it.get("confirm") or {}).get("t")),
                    "t_end": done[-1] if done else None, "what": what(it), "outcome": outcome, "fate": fate(outcome)[0],
                    "why_stopped": (it.get("confirm") or {}).get("why"), "tries": it.get("tries", 0)})
    return out


def walked(od) -> tuple:
    """the route he walked (odometry, a point every 10 cm) and its length in metres"""
    if not len(od): return [], 0.0
    pts, last = [od[0, 1:3]], od[0, 1:3]
    for p in od[1:, 1:3]:
        if np.hypot(*(p - last)) >= 0.10: pts.append(p); last = p
    pts = np.array(pts); return pts.round(2).tolist(), float(np.hypot(*np.diff(pts, axis=0).T).sum()) if len(pts) > 1 else 0.0


def clock(t) -> str:
    d = dt.datetime.fromtimestamp(t); return f"{(d.hour - 1) % 12 + 1}:{d.minute:02d} {'am' if d.hour < 12 else 'pm'}"


def words(n) -> str:
    return NUM[n] if 0 <= n < len(NUM) else str(n)


def plural(label, n) -> str:
    """"a toy" x3 -> "three toys"; "a roll of tape" x2 -> "two rolls of tape" """
    w = label.split(" ", 1)[1] if label.startswith(("a ", "an ")) else label
    head, _, tail = w.partition(" of "); hw = head.rsplit(" ", 1)
    hw[-1] += "es" if hw[-1].endswith(("s", "x", "ch", "sh")) else "s"
    return f"{words(n)} {' '.join(hw)}" + (f" of {tail}" if tail else "")


def listing(labels) -> str:
    """"a box, a cup and three toys": each name once, counted, in the order first seen"""
    n = {}
    for l in labels: n[l] = n.get(l, 0) + 1
    out = [l if c == 1 else plural(l, c) for l, c in n.items()]
    return out[0] if len(out) == 1 else ", ".join(out[:-1]) + " and " + out[-1]


def fact(value, text, source, how):
    return {"value": value, "text": text, "source": source, "how": how}


def household(night) -> dict:
    """the household's facts ([9X9], [W0w]): only the night's own outputs; each with its text, source and how it's had"""
    rep = night.json("report.json")                                        # no report, no night: never an "empty" one
    if rep is None: raise FileNotFoundError(f"{night.dir}: no report.json, so not a finished night")
    t0 = rep.get("t0"); lines = tidy_lines(night, t0)
    od = night.odometry(); route, metres = walked(od); F = finds(rep, lines); en = night.json("energy.json")
    rid = night.recording_id(); nr = night.record()
    if t0 is None and nr.get("started") is not None: t0 = float(nr["started"]); lines = tidy_lines(night, t0)
    t_start = t0 if t0 is not None else (float(od[0, 0]) if len(od) else None)
    t_end = lines[-1][0] if lines else (float(nr["ended"]) if nr.get("ended") is not None else float(od[-1, 0]) if len(od) else None)
    stops = [f for f in F if f["fate"] != "nothing"]
    fx = {}
    if t_start is not None: fx["started"] = fact(t_start, clock(t_start), "report.json", "t0, when the tidy began")
    end_src = ("dimos.log", "the tidy's last line") if lines else ("night.json", "ended, when the night ended") \
        if nr.get("ended") is not None else ("traj.csv" if getattr(night, "odometry_source", "") == "traj.csv" else "recording.txt", "the last odometry")
    if t_end is not None: fx["ended"] = fact(t_end, clock(t_end), *end_src)
    if t_start is not None and t_end is not None:
        fx["minutes"] = fact(round((t_end - t_start) / 60), str(round((t_end - t_start) / 60)), end_src[0], "the end less t0")
    fx["walked_m"] = fact(round(metres), str(round(metres)), getattr(night, "odometry_source", "traj.csv"), "odometry path length, points 10 cm apart")
    if rep.get("watch_only"): fx["watch_only"] = fact(True, "watched only", "report.json", "watch_only: the rounds walked, no stop made")
    if rep.get("arm") == "none":                                          # [jFy]: the dog alone, every arm step skipped
        sk = list(rep.get("arm_skipped") or ["every step that needs it"])
        fx["no_arm"] = fact(sk, listing(sk), "report.json", "arm: none, and the steps skipped for it")
    wp = [tuple(map(int, m.groups())) for _, s in lines for m in [re.match(r"SWEEP: waypoint (\d+)/(\d+)", s)] if m]
    if wp: fx["waypoints"] = fact(wp[-1][1], words(wp[-1][1]), "dimos.log", "the route's waypoints")
    if wp: fx["waypoints_reached"] = fact(max(w for w, _ in wp), str(max(w for w, _ in wp)), "dimos.log", "the last waypoint started")
    for k, lab in (("seed", "the house seed"), ("mode", "the night's mode"), ("item_seed", "the item seed"),
                   ("round", "the round it ran in"), ("night_tidy", "the night_tidy folder that ran it")):
        if nr.get(k) not in (None, ""): fx["night_folder" if k == "night_tidy" else f"night_{k}"] = fact(nr[k], str(nr[k]), "night.json", lab)
    rt = night.json(nr["route"]) if nr.get("route") else None
    if rt and rt.get("waypoints"):
        fx["route_planned_m"] = fact(round(float(rt.get("length_m", 0.0)), 1), f"{float(rt.get('length_m', 0.0)):.1f}", "route.json", "length_m, the route he was given")
        fx["route_waypoints"] = fact(len(rt["waypoints"]), str(len(rt["waypoints"])), "route.json", "its waypoints")
    fx["stops"] = fact(len(F), words(len(F)), "report.json", "items: every stop he made")
    nothing = len(F) - len(stops)
    fx["fate_nothing"] = fact(nothing, words(nothing), "report.json", "stops whose standing look saw nothing near the flag")
    for _, name, *_ in FATES:
        if name == "nothing": continue
        g = [f for f in F if f["fate"] == name]
        if g:
            fx[f"fate_{name}"] = fact(len(g), words(len(g)), "report.json", f"stops whose outcome reads as {name}")
            fx[f"fate_{name}_list"] = fact([f["n"] for f in g], listing([f["what"] for f in g]), "report.json", "what he took each for")
    kinds = {}
    for f in stops: kinds[f["what"]] = kinds.get(f["what"], 0) + 1
    fx["finds_by_kind"] = fact(kinds, ", ".join(f"{k} {v}" for k, v in sorted(kinds.items(), key=lambda kv: -kv[1])) or "none",
                               "report.json", "what he took each find for (his looks' names)")
    took = [f for f in stops if f["t"] is not None and f["t_end"] is not None]
    if took:
        w = max(took, key=lambda f: f["t_end"] - f["t"]); m_ = (w["t_end"] - w["t"]) / 60
        the = re.sub(r"^an? ", "the ", w["what"])
        fx["trouble_what"] = fact(w["n"], the, "report.json", "the find he spent longest on")
        fx["trouble_minutes"] = fact(round(m_, 1), words(round(m_)) if round(m_) >= 1 else "under a minute", "dimos.log", "its first line to its outcome")
        fx["trouble_time"] = fact(w["t"], clock(w["t"]), "dimos.log", "when he set off for it")
    if not en and (b := night.battery()) is not None:     # a real night: the robot's own battery stream ([jFy])
        fx["battery_used_pct"] = fact(round(b[0] - b[1]), str(round(b[0] - b[1])), "recording.txt", "the battery stream's charge, first less last")
        fx["battery_left_pct"] = fact(round(b[1]), str(round(b[1])), "recording.txt", "the battery stream's last charge")
    if en:
        used = 100.0 * en["air_used_Wh"] / en["air_Wh"]
        fx["battery_used_pct"] = fact(round(used), str(round(used)), "energy.json", "air_used_Wh / air_Wh")
        fx["battery_used_wh"] = fact(round(en["air_used_Wh"]), str(round(en["air_used_Wh"])), "energy.json", "air_used_Wh")
        fx["battery_left_pct"] = fact(round(en["air_left_pct"]), str(round(en["air_left_pct"])), "energy.json", "air_left_pct")
    st = stage_times(lines)
    if st:
        k = max(st, key=st.get)
        fx["slowest_stage"] = fact({"stage": k, "s": round(st[k])}, f"{STAGES.get(k, k.lower())}, {st[k] / 60:.0f} min", "dimos.log",
                                   "time between the tidy's stage lines, summed per stage")
    fx["tries"] = fact(sum(f["tries"] for f in F), str(sum(f["tries"] for f in F)), "report.json", "pick attempts, all stops")
    parks = [it["park_err_cm"] for it in rep.get("items", []) if "park_err_cm" in it]
    if parks: fx["park_err_cm"] = fact(float(np.median(parks)), f"{np.median(parks):.1f} cm", "report.json", "median park_err_cm")
    bal = rep.get("balance") or {}
    if "max_deg" in bal: fx["max_tilt_deg"] = fact(round(bal["max_deg"], 1), f"{bal['max_deg']:.1f} deg", "report.json", "balance.max_deg")
    seen_out = {(round(r["x"] / 0.15), round(r["y"] / 0.15)) for r in rep.get("sweep") or []   # [OrO]: noted, not visited
                if isinstance(r, dict) and r.get("outside_area") and r.get("decision") == "confirmed"}
    seen_out |= {(round(o["x"] / 0.15), round(o["y"] / 0.15)) for o in rep.get("outside_area") or []}
    if seen_out: fx["outside_seen"] = fact(len(seen_out), words(len(seen_out)), "report.json", "confirmed spots outside the operating area, not visited")
    if rep.get("returns"): fx["area_returns"] = fact(len(rep["returns"]), words(len(rep["returns"])), "report.json", "returns: found outside the area, walked back in")
    if rep.get("stopped"):
        fx["stopped_lost"] = fact(rep["stopped"], "lost", "report.json", "stopped: why the run stopped") if "localization lost" in rep["stopped"] \
            else fact(rep["stopped"], "outside", "report.json", "stopped: why the run stopped")
    start = od[0, 1:3] if len(od) else None
    k = 0
    for f in F:
        if f["fate"] != "nothing": k += 1
        f["k"] = k if f["fate"] != "nothing" else None                       # the household's numbering: finds only
        f["clock"] = clock(f["t"]) if f["t"] else None
        f["from_start_m"] = round(float(np.hypot(f["x"] - start[0], f["y"] - start[1])), 1) if start is not None else None
    return {"night": {"dir": str(night.dir), "recording": rid, "kind": kind_of(rid), "date": dt.datetime.fromtimestamp(t_start).strftime("%A %-d %B %Y") if t_start else None},
            "facts": fx, "finds": F, "stage_s": {k: round(v, 1) for k, v in st.items()},
            "route": route, "start": start.round(2).tolist() if start is not None else None}


# ---- pictures ([hKP], [R1X]) ----

def no_people_in_sim(img) -> bool:
    """a sim night's screen: the sim houses hold no people, so every crop is clear"""
    return True


def owl_people_screen(img) -> bool:
    """a real night's screen: clear only if the footage screen's detector finds no person or body part in the crop"""
    sys.path.insert(0, str(HK / "tools")); import screen_people
    return not screen_people.people_in(img)


def screen_for(kind):
    return no_people_in_sim if kind == "sim" else owl_people_screen


def look_crop(night, n, flag, size=None):
    """the find's crop from his close look (looks/item<n>_look1.npz: the first view that has it in front), else from his
    standing look (standing_<k>.png at the flag's pixel): (RGB array, its source, box in the image) or None"""
    p = night.path(f"looks/item{n}_look1.npz")
    if p.exists():
        z = np.load(p)
        for v in range(8):
            if f"{v}_img" not in z.files: break
            img = z[f"{v}_img"]; H, W = img.shape[:2]
            xy = z[f"{v}_found_xy"] if f"{v}_found_xy" in z.files else z[f"{v}_aim"] if f"{v}_aim" in z.files else np.asarray(flag)
            pc = z[f"{v}_R"].T @ (np.array([xy[0], xy[1], 0.0]) - z[f"{v}_c"])
            if pc[2] >= 0: continue
            fpx = (H / 2) / math.tan(math.radians(float(z[f"{v}_fovy"])) / 2)
            u, w = W / 2 + fpx * pc[0] / -pc[2], H / 2 - fpx * pc[1] / -pc[2]
            if 0 <= u < W and 0 <= w < H: return crop_at(img, u, w, size or crop_side(W)) + (f"looks/item{n}_look1.npz", v)
    return None


def standing_crop(night, k, look, size=None):
    p = night.path(f"looks/standing_{k}.png")
    if not p.exists() or not look or not look.get("flag_px"): return None
    from PIL import Image
    img = np.asarray(Image.open(p).convert("RGB")); u, w = look["flag_px"]
    if not (0 <= u < img.shape[1] and 0 <= w < img.shape[0]): return None
    return crop_at(img, u, w, size or crop_side(img.shape[1])) + (f"looks/standing_{k}.png", None)


def view_crop(night, view):
    """a flagged place's crop from the recording itself, the light check's best view of it (a replayed night has no
    looks of its own: real_tidy.py's "view", {ts, box_full: [x0, y0, side] or [x0, y0, x1, y1]}): (RGB array, box, image size, source, None)"""
    if not view or view.get("ts") is None or not view.get("box_full"): return None
    img = night.frame(float(view["ts"]))
    if img is None: return None
    b = [int(v) for v in view["box_full"]]; H, W = img.shape[:2]       # [x0, y0, side] (the bench's verdicts) or
    x0, y0 = b[0], b[1]; x1, y1 = (b[2], b[3]) if len(b) == 4 else (x0 + b[2], y0 + b[2])   # corners (the robot's check)
    x0, y0 = max(0, min(x0, W - 1)), max(0, min(y0, H - 1)); x1, y1 = min(W, x1), min(H, y1)
    if x1 <= x0 or y1 <= y0: return None
    return img[y0:y1, x0:x1], [x0, y0, x1, y1], [W, H], "recording.txt", None


def crop_at(img, u, v, size):
    """a size x size crop centred on (u, v) as nearly as the image allows: always inside it"""
    H, W = img.shape[:2]; s = min(size, H, W)
    x0 = int(min(max(round(u - s / 2), 0), W - s)); y0 = int(min(max(round(v - s / 2), 0), H - s))
    return img[y0:y0 + s, x0:x0 + s], [x0, y0, x0 + s, y0 + s], [W, H]


def jpeg_b64(img, px=None, quality=85) -> str:
    """an RGB crop as base64 JPEG, scaled to px square if given"""
    from PIL import Image
    im = Image.fromarray(np.ascontiguousarray(img))
    if px and im.size[0] > px: im = im.resize((px, px), Image.BILINEAR)
    b = io.BytesIO(); im.save(b, "JPEG", quality=quality, optimize=True); return base64.b64encode(b.getvalue()).decode()


def pictures(night, F, kind, screen=None, px=None, quality=85) -> tuple:
    """one crop per find (not for a stop that found nothing), only from a recording cleared for its frames, and each one
    through the people screen ([hKP], [R1X]): ({find n: {jpg, source, box, image_wh}}, why any are missing)"""
    rid = night.recording_id()
    try:
        if not rid: raise PermissionError("the night names no recording")
        recordings.require_screened(rid)
    except (PermissionError, SystemExit) as e:
        return {}, f"Pictures withheld: {e}"
    screen = screen or screen_for(kind)
    rep = night.json("report.json", {}) or {}; items = rep.get("items", []); looks = rep.get("standing_looks", [])
    k_of, k = {}, 0
    for n, it in enumerate(items, 1):
        if "standing" in it: k += 1; k_of[n] = k
    out, withheld = {}, 0
    for f in F:
        if f["fate"] == "nothing": continue
        c = look_crop(night, f["n"], (f["x"], f["y"]))
        if c is None and f["n"] in k_of: c = standing_crop(night, k_of[f["n"]], looks[k_of[f["n"]] - 1] if k_of[f["n"]] <= len(looks) else None)
        if c is None and f["n"] <= len(items): c = view_crop(night, items[f["n"] - 1].get("view"))
        if c is None: continue
        img, box, wh, src, view = c
        if not screen(img): withheld += 1; continue                      # a crop that may show a person is never kept
        out[f["n"]] = {"jpg": jpeg_b64(img, px, quality), "source": src, "view": view, "box": box, "image_wh": wh}
    return out, (f"{withheld} picture(s) withheld by the people screen." if withheld else "")


# ---- his words ([W0w], [ZNG]) ----

def sentences(H) -> list:
    """the prose as templates: {key} is a fact's text, {key|cap} capitalised; a sentence appears only if its facts do"""
    fx, S = H["facts"], []
    def add(tpl, *need):
        if all(k in fx for k in need) and all(k in fx for k in re.findall(r"\{(\w+)", tpl)): S.append(tpl)
    add("Good morning. I set out at {started} and was done by {ended}, {minutes} minutes in all, and walked {walked_m} metres about the house.")
    if fx["stops"]["value"] == 0:
        add("Nothing on the floor caught my eye; the house was as you left it, and I found nothing that wanted putting away.")
    elif "watch_only" in fx:
        add("Something on the floor caught my eye at {stops} places; I walked my rounds without stopping, and have marked "
            "each for you to look at.")
    else:
        add("Something on the floor caught my eye {stops} times, and each time I went to see.")
        if fx["fate_nothing"]["value"] == fx["stops"]["value"]:
            add("Each time there proved to be nothing at all, and I beg your pardon for the fuss.")
        elif fx["fate_nothing"]["value"]:
            add("At {fate_nothing} of those places there proved to be nothing at all, and I beg your pardon for the fuss.")
    for _, name, one, many, _ in FATES:
        if name == "nothing" or f"fate_{name}" not in fx: continue
        if "watch_only" in fx and name == "other": continue          # the places marked: said once, above
        S.append((one if fx[f"fate_{name}"]["value"] == 1 else many).replace("{L", f"{{fate_{name}_list").replace("{c", f"{{fate_{name}"))
    if "trouble_what" in fx and fx["stops"]["value"] > fx["fate_nothing"]["value"] + 1 and fx["trouble_minutes"]["value"] >= 2:
        add("{trouble_what|cap} I went to at {trouble_time} gave me the most trouble: {trouble_minutes} minutes of it.")
    if "no_arm" in fx:
        add("I have no arm as yet, so I left undone all that needs one: {no_arm}.")
    if "outside_seen" in fx:
        S.append(("I noticed one thing" if fx["outside_seen"]["value"] == 1 else "I noticed {outside_seen} things")
                 + " beyond the bounds you set, and, as instructed, left them be.")
    if "area_returns" in fx:
        S.append("I found myself beyond the bounds you set" + (" once" if fx["area_returns"]["value"] == 1 else " {area_returns} times")
                 + ", and went straight back in before anything else.")
    if "stopped_lost" in fx:
        S.append("I lost my bearings and stopped where I stood, as I ought, until you can set me right." if fx["stopped_lost"]["text"] == "lost"
                 else "I found myself beyond the bounds you set and could not get back in, so I stopped where I stood.")
    if "battery_used_pct" in fx:
        add("The night cost the battery {battery_used_pct} per cent of its charge; {battery_left_pct} per cent remains.")
    else:
        S.append("I kept no account of the battery last night.")
    if fx["stops"]["value"] > fx["fate_nothing"]["value"]: S.append("A picture of each is below, with the hour and the place.")
    S.append("I remain, as ever, your servant.")
    return S


def fill(tpl, fx, span):
    def sub(m):
        k, cap = m[1], m[2] == "|cap"; t = fx[k]["text"]; t = t[:1].upper() + t[1:] if cap else t
        return span(k, t)
    return re.sub(r"\{(\w+)(\|cap)?\}", sub, tpl)


def prose_text(H) -> list:
    return [fill(s, H["facts"], lambda k, t: t) for s in sentences(H)]


# ---- the page ([Ndt]) ----

def house_png(night, box, res=0.05):
    """the house map he started from (house_map/house_map_upto*.npy), seen from above, as a PNG over box (x0, y0, x1, y1)"""
    ps = sorted(night.dir.glob("house_map/house_map_upto*.npy"))
    if not ps: return None, None
    P = np.load(ps[-1]); x0, y0, x1, y1 = box
    W, H = int((x1 - x0) / res), int((y1 - y0) / res)
    c = ((P[:, 0] - x0) / res).astype(int); r = ((y1 - P[:, 1]) / res).astype(int); ok = (c >= 0) & (c < W) & (r >= 0) & (r < H)
    g = np.zeros((H, W, 4), np.uint8); g[r[ok], c[ok]] = (120, 110, 95, 255)
    from PIL import Image
    b = io.BytesIO(); Image.fromarray(g, "RGBA").save(b, "PNG")
    return base64.b64encode(b.getvalue()).decode(), str(ps[-1].relative_to(night.dir))


def map_svg(night, H):
    pts = np.array(H["route"] + [[f["x"], f["y"]] for f in H["finds"]]) if H["route"] or H["finds"] else np.zeros((0, 2))
    if not len(pts): return "", None
    x0, y0 = pts.min(0) - 1.0; x1, y1 = pts.max(0) + 1.0
    png, src = house_png(night, (x0, y0, x1, y1))
    e = []
    if png: e.append(f'<image href="data:image/png;base64,{png}" x="{x0:.2f}" y="{-y1:.2f}" width="{x1 - x0:.2f}" height="{y1 - y0:.2f}" preserveAspectRatio="none"/>')
    if H["route"]: e.append('<polyline class="route" points="' + " ".join(f"{x:.2f},{-y:.2f}" for x, y in H["route"]) + '"/>')
    if H["start"]: e.append(f'<rect class="start" x="{H["start"][0] - 0.15:.2f}" y="{-H["start"][1] - 0.15:.2f}" width="0.3" height="0.3"/>')
    for f in H["finds"]:
        if f["fate"] == "nothing": e.append(f'<circle class="nothing" cx="{f["x"]:.2f}" cy="{-f["y"]:.2f}" r="0.07"/>')
    for f in H["finds"]:
        if f["fate"] != "nothing":
            e.append(f'<g class="find {f["fate"]}"><circle cx="{f["x"]:.2f}" cy="{-f["y"]:.2f}" r="0.2"/>'
                     f'<text x="{f["x"]:.2f}" y="{-f["y"] + 0.08:.2f}">{f["k"]}</text></g>')
    return (f'<svg class="map" viewBox="{x0:.2f} {-y1:.2f} {x1 - x0:.2f} {y1 - y0:.2f}" role="img" '
            f'aria-label="The house from above: his route, where he started, and each find">' + "".join(e) + "</svg>"), src


CSS = """
:root{--paper:#f6f1e7;--ink:#2b2620;--muted:#7a6f60;--rule:#d9cfbd;--accent:#6b2e2a;--route:#3d5a73;--card:#fffdf8;--nothing:#a99d8a}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--paper:#1c1a17;--ink:#ece4d6;--muted:#a89c89;--rule:#3a352d;--accent:#d29a8f;--route:#8fb3d1;--card:#252219;--nothing:#6e6556}}
:root[data-theme="dark"]{--paper:#1c1a17;--ink:#ece4d6;--muted:#a89c89;--rule:#3a352d;--accent:#d29a8f;--route:#8fb3d1;--card:#252219;--nothing:#6e6556}
*{box-sizing:border-box}
body{margin:0;background:var(--paper);color:var(--ink);font:17px/1.6 Georgia,"Iowan Old Style","Times New Roman",serif}
main{max-width:680px;margin:0 auto;padding:28px 16px 48px}
header{border-bottom:1px solid var(--rule);margin-bottom:22px;padding-bottom:10px}
.kicker{font-variant:small-caps;letter-spacing:.08em;color:var(--muted);font-size:14px}
h1{font-weight:normal;font-size:28px;margin:4px 0 0}
h2{font-weight:normal;font-variant:small-caps;letter-spacing:.06em;font-size:18px;color:var(--accent);margin:34px 0 10px;border-bottom:1px solid var(--rule)}
.prose p{margin:0 0 .7em}
.sign{font-style:italic;text-align:right;margin-top:6px}
.fact{font-variant-numeric:oldstyle-nums}
.finds{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:14px}
.card{background:var(--card);border:1px solid var(--rule);border-radius:6px;overflow:hidden}
.card img{display:block;width:100%;aspect-ratio:1;object-fit:cover}
.card .nopic{aspect-ratio:1;display:flex;align-items:center;justify-content:center;color:var(--muted);font-style:italic;padding:16px;text-align:center}
.card .cap{padding:8px 10px 10px;font-size:15px;line-height:1.4}
.card .when{color:var(--muted);font-size:13px}
.num{display:inline-block;min-width:1.6em;height:1.6em;line-height:1.6em;border-radius:50%;background:var(--accent);color:var(--paper);text-align:center;font-size:13px;margin-right:6px}
svg.map{width:100%;height:auto;background:var(--card);border:1px solid var(--rule);border-radius:6px}
svg.map image{opacity:.85}
svg.map .route{fill:none;stroke:var(--route);stroke-width:.05;stroke-linejoin:round;opacity:.8}
svg.map .start{fill:var(--ink)}
svg.map .nothing{fill:var(--nothing)}
svg.map .find circle{fill:var(--accent);stroke:var(--paper);stroke-width:.04}
svg.map .find text{fill:var(--paper);font:bold .22px Georgia,serif;text-anchor:middle}
.legend{color:var(--muted);font-size:14px;margin-top:6px}
details{margin-top:28px;border-top:1px solid var(--rule);padding-top:8px}
summary{cursor:pointer;font-variant:small-caps;letter-spacing:.06em;color:var(--accent);font-size:17px}
table{border-collapse:collapse;width:100%;font-size:14px;margin-top:10px}
td,th{text-align:left;padding:5px 6px;border-bottom:1px solid var(--rule);vertical-align:top}
th{font-weight:normal;color:var(--muted)}
td.v{font-variant-numeric:tabular-nums}
td{overflow-wrap:anywhere}
.note.how{display:block}
a{color:var(--route)}
.note{color:var(--muted);font-size:14px;font-style:italic}
"""

ENG = [("started", "Began"), ("ended", "Finished"), ("minutes", "Minutes"), ("waypoints_reached", "Route waypoints reached"),
       ("waypoints", "Route waypoints planned"), ("walked_m", "Walked (m, odometry)"), ("stops", "Stops"),
       ("fate_nothing", "Stops for nothing (his own judgement)"), ("finds_by_kind", "Finds by kind (his names)"),
       ("fate_put_away", "Put in the bin"), ("tries", "Pick attempts"), ("park_err_cm", "Parking error, median"),
       ("battery_used_pct", "Battery used (%)"), ("battery_used_wh", "Battery used (Wh)"), ("battery_left_pct", "Battery left (%)"),
       ("slowest_stage", "Slowest stage"), ("max_tilt_deg", "Greatest tilt"), ("route_planned_m", "Route planned (m)"),
       ("route_waypoints", "Route waypoints given"), ("night_seed", "House seed"), ("night_mode", "Mode"),
       ("night_item_seed", "Item seed"), ("night_round", "Round"), ("night_folder", "Night folder")]


def render(H, pics, why_no_pics, truth_html, out_dir, night) -> str:
    esc = html.escape; fx = H["facts"]; src = lambda s: esc(os.path.relpath(night.dir / s, out_dir))
    span = lambda k, t: f'<span class="fact" data-fact="{k}">{esc(t)}</span>'
    P = "".join(f"<p>{fill(s, fx, span)}</p>" for s in sentences(H))
    cards = []
    for f in H["finds"]:
        if f["fate"] == "nothing": continue
        pic = pics.get(f["n"])
        img = (f'<img alt="Find {f["k"]}: {esc(f["what"])}, from his look" src="data:image/jpeg;base64,{pic["jpg"]}">' if pic
               else f'<div class="nopic">{esc(why_no_pics or "No look at it was kept.")}</div>')
        where = f'{f["from_start_m"]} m from where I started' if f["from_start_m"] is not None else ""
        cards.append(f'<div class="card" id="find-{f["k"]}">{img}<div class="cap"><span class="num">{f["k"]}</span>'
                     f'{esc(f["what"][:1].upper() + f["what"][1:])}. {esc(fate(f["outcome"])[3])}'
                     f'<div class="when">{esc(f["clock"] or "")}{" · " if where and f["clock"] else ""}{esc(where)}</div></div></div>')
    svg, map_src = map_svg(night, H)
    rows = "".join(f'<tr><td>{esc(lab)}</td><td class="v">{esc(str(fx[k]["value"]) if isinstance(fx[k]["value"], int) and k not in ("started", "ended") else fx[k]["text"])}</td>'
                   f'<td><a href="{src(fx[k]["source"])}">{esc(fx[k]["source"])}</a> <span class="note how">{esc(fx[k]["how"])}</span></td></tr>' for k, lab in ENG if k in fx)
    stages = "".join(f'<tr><td>{esc(STAGES.get(k, k.lower()))}</td><td class="v">{v / 60:.1f} min</td><td><a href="{src("dimos.log")}">dimos.log</a></td></tr>'
                     for k, v in sorted(H["stage_s"].items(), key=lambda kv: -kv[1]))
    kn = {f["n"]: f["k"] for f in H["finds"]}
    pic_rows = "".join(f'<tr><td>Find {kn[n]} (stop {n})</td><td class="v">box {p["box"]} of {p["image_wh"]}</td><td><a href="{src(p["source"])}">{esc(p["source"])}</a></td></tr>'
                       for n, p in sorted(pics.items()))
    T = truth_html(src) if truth_html else ""                # the sim's score: morning_score.py's, never his own
    nothing_n = fx["fate_nothing"]["value"]
    legend = ("His route in blue from the black square where he started; numbered finds as on the cards"
              + (f"; the {nothing_n} stops for nothing are the small grey dots" if nothing_n else "") + ".")
    title = f'The night of {H["night"]["date"].rsplit(" ", 1)[0]}' if H["night"]["date"] else "The night"
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Barrymore's Morning Log</title><style>{CSS}</style></head><body><main>
<header><div class="kicker">Barrymore's morning log{" · a simulated night" if H["night"]["kind"] == "sim" else ""}</div><h1>{esc(title)}</h1></header>
<section class="prose">{P}<p class="sign">Barrymore</p></section>
{f'<h2>What I found</h2><div class="finds">{"".join(cards)}</div>' if cards else ""}
{f'<h2>The house, and my walk</h2>{svg}<div class="legend">{legend}</div>' if svg else ""}
<details><summary>The night's figures</summary><table><tr><th>Figure</th><th>Value</th><th>From</th></tr>{rows}</table>
<table><tr><th>Stage</th><th>Time</th><th>From</th></tr>{stages}</table>
{f'<table><tr><th>Picture</th><th>Crop</th><th>From</th></tr>{pic_rows}</table>' if pic_rows else ""}
{f'<p class="note">Map: <a href="{src(map_src)}">{esc(map_src)}</a> (the house map he started from), route from <a href="{src(fx["walked_m"]["source"])}">{esc(fx["walked_m"]["source"])}</a> (odometry).</p>' if map_src else ""}
<p class="note">Night: {esc(str(night.dir))}; recording {esc(str(H["night"]["recording"]))} ({esc(H["night"]["kind"])}). Facts: morning_log.json.</p>
</details>{T}
</main></body></html>"""


def write(night_dir, out=None, scored=None, screen=None) -> Path:
    """the night's log: morning_log.html and morning_log.json in out (default the night's own directory, [t0a]).
    scored(night, kind) -> (its figures, html(src) -> the page's section) or None: a section scored against the sim's
    truth, which morning_score.py passes for a sim night; the household's part never reads the truth ([9X9], [1B2])"""
    night = Night(night_dir); out = Path(out or night.dir); out.mkdir(parents=True, exist_ok=True)
    H = household(night); pics, why = pictures(night, H["finds"], H["night"]["kind"], screen, FILE_CROP_PX)
    S = scored(night, H["night"]["kind"]) if scored else None; T = S[0] if S else None
    for f in H["finds"]:
        p = pics.get(f["n"]); f["picture"] = {k: p[k] for k in ("source", "view", "box", "image_wh")} if p else None
    doc = dict(H, prose=prose_text(H), pictures_note=why, scored_against_truth=T)
    (out / "morning_log.json").write_text(json.dumps(doc, indent=1))
    (out / "morning_log.html").write_text(render(H, pics, why, S[1] if S else None, out, night))
    return out / "morning_log.html"


# ---- on the phone ([Ndt]) ----

def began(d: Path) -> float:
    """when a night began, for ordering nights ([q66]): a sim tidy run's folder stamp (YYYYmmdd_HHMMSS); a real night
    (<YYYYmmdd>/night, [oWO]) its night.json's start, else its evening's date"""
    for name, fmt in ((d.name, "%Y%m%d_%H%M%S"), (d.parent.name if d.name == "night" else "", "%Y%m%d")):
        try: t0 = dt.datetime.strptime(name, fmt).timestamp()
        except ValueError: continue
        try: return float(json.loads((d / "night.json").read_text()).get("started") or t0) if fmt == "%Y%m%d" else t0
        except (OSError, ValueError, TypeError): return t0
    return 0.0


def latest_night(root) -> Path | None:
    """the newest night with its log written under root, or any of several roots ([q66]): a sim tidy run's directory
    (root/<YYYYmmdd_HHMMSS>/) or a real night's (root/<YYYYmmdd>/night/, [oWO])"""
    roots = [root] if isinstance(root, (str, os.PathLike)) else list(root)
    ds = [p.parent for r in roots for g in ("*/morning_log.json", "*/night/morning_log.json") for p in Path(r).glob(g)]
    return max(ds, key=lambda d: (began(d), str(d))) if ds else None


def morning_message(root, screen=None, max_bytes=PHONE_MAX_BYTES) -> dict | None:
    """the latest night's log for the phone cockpit's Morning page: its morning_log.json (prose, finds, route, the
    figures and their sources) with each find's crop as JPEG base64, the crops made again here through pictures(), so
    only from a screened recording and through the people screen ([hKP], [R1X]). Never more than max_bytes of compact
    JSON: crops made smaller, then dropped from the last find back, then the route thinned. None: no night yet."""
    d = latest_night(root)
    if d is None: return None
    doc = json.loads((d / "morning_log.json").read_text()); night = Night(d)
    finds = [f for f in doc["finds"] if f["fate"] != "nothing"]
    pics, why = pictures(night, finds, doc["night"]["kind"], screen, PHONE_CROP_PX, PHONE_JPEG_Q)
    fx = doc["facts"]
    m = {"v": 1, "night": doc["night"], "title": f'The night of {doc["night"]["date"].rsplit(" ", 1)[0]}' if doc["night"].get("date") else "The night",
         "prose": doc["prose"], "pictures_note": why,
         "finds": [{"k": f["k"], "what": f["what"], "fate": f["fate"], "card": fate(f["outcome"])[3], "clock": f["clock"],
                    "from_start_m": f["from_start_m"], "x": f["x"], "y": f["y"], "jpg": (pics.get(f["n"]) or {}).get("jpg")} for f in finds],
         "nothing": [[f["x"], f["y"]] for f in doc["finds"] if f["fate"] == "nothing"],
         "route": doc["route"], "start": doc["start"],
         "figures": [{"label": lab, "text": fx[k]["text"], "source": fx[k]["source"]} for k, lab in ENG if k in fx]}
    size = lambda: len(json.dumps(m, separators=(",", ":")))
    for px, q in ((PHONE_CROP_PX, PHONE_JPEG_Q), (160, 60), (120, 50)):
        if size() <= max_bytes: break
        small, _ = pictures(night, finds, doc["night"]["kind"], screen, px, q)
        for f, g in zip(m["finds"], finds): f["jpg"] = (small.get(g["n"]) or {}).get("jpg") if f["jpg"] else None
    dropped = 0
    for f in reversed(m["finds"]):
        if size() <= max_bytes: break
        if f["jpg"]: f["jpg"] = None; dropped += 1
    if dropped: m["pictures_note"] = (m["pictures_note"] + " " if m["pictures_note"] else "") + f"{dropped} picture(s) left off to fit the phone."
    while size() > max_bytes and len(m["route"]) > 2: m["route"] = m["route"][::2]
    if size() > max_bytes: m["prose"] = m["prose"][:1]; m["figures"] = []
    return m if size() <= max_bytes else None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0]); ap.add_argument("night")
    ap.add_argument("--out", help="where to write (default: the night's directory)")
    a = ap.parse_args(argv); p = write(a.night, a.out)
    print(p); print("\n".join(json.loads((p.parent / "morning_log.json").read_text())["prose"]))


if __name__ == "__main__":
    main()
