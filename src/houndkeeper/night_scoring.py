"""A real night scored from the phone ([vO9], [71R]; decisions [hOy], [wTj], [Vd3], [oWO]): the Marks page's and the
Review page's side, on the project service (houndkeeper.project), so they work with the robot docked or off.

Marks (cockpit/extensions/hk-marks.js): before the night the user taps each laid-out toy's place on the house's floor
map (the Area page's map, in the house's frame), its name from TOYS (which gives its length, for the sim's reach, and
whether it is a 1 x 1 brick) and its floor kind; a confirmed save writes marks.json in the night's directory, the shape
real/tests/score_night.py reads. Refused once the night has run (its candidates.json exists).

Review (cockpit/extensions/hk-review.js): in the morning each flag (the candidates near no marked toy, merged as the
scorer merges them) with its crop (review_crops/, made by real/tests/flag_crops.py through the people screen) and its
place; the user's tap, false alarm or real change, is written to review.json for every candidate in the flag, the
night scored again (result.json) and final when none is left unjudged. The first two nights that ran are tuning nights
([wTj]): their results say so, and pooling keeps them apart.

Every save and judgement is a line in the night's score_actions.log.jsonl, with its time ([Dlx]). Files only, beside
the night ([oWO]); no image leaves the laptop except to the user's phone through the relay, as the morning log's do."""
from __future__ import annotations

import base64, datetime as dt, json, math, os, sys, threading
from pathlib import Path
from dataclasses import dataclass, field
from typing import Any, ClassVar

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.web.cockpit import Channel, Panel, _request_of

HK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HK / "real" / "tests"))
import score_night as sn  # noqa: E402  the scorer: marks matched by the sim's reach, judgements, pooling

ROOT = HK / "data" / "real_nights"        # HK_REAL_NIGHTS overrides ([oWO])
EVENING_STARTS_H = 6                      # before 6 am is still the evening before
MARKS, REVIEW, RESULT, CANDS, LOG, CROPS = "marks.json", "review.json", "result.json", "candidates.json", "score_actions.log.jsonl", "review_crops"
FRAME = "house"
MAX_BYTES = 384 * 1024                    # the Review page's message, compact JSON (the Morning page's bound)
PUBLISH_S = 10.0                          # published again unchanged this often, for a late viewer ([E2e])
BY = "the phone cockpit"

# the short list ([oWO]): name, half the toy's longest side (m: the sim's reach is 30 cm, or 15 cm past half a long
# toy), and whether it is a 1 x 1 brick (the smallest, reported apart, [vO9])
TOYS = [("1 x 1 brick", 0.004, True), ("2 x 2 brick", 0.008, False), ("toy car", 0.05, False), ("small figure", 0.04, False),
        ("ball", 0.04, False), ("building block", 0.03, False), ("crayon", 0.045, False), ("puzzle piece", 0.03, False),
        ("soft toy", 0.10, False), ("toy train", 0.08, False), ("doll", 0.15, False), ("book", 0.12, False),
        ("long toy (a sword, a wand)", 0.25, False), ("another toy", 0.05, False)]
FLOOR_NAMES = {"hard": "Hard floor", "carpet": "Carpet", "rug": "Rug"}


def root() -> Path:
    return Path(os.environ.get("HK_REAL_NIGHTS") or ROOT)


def night_name(now: dt.datetime | None = None) -> str:
    """a night's name: the date its evening began, YYYYmmdd ([oWO])"""
    now = now or dt.datetime.now()
    return (now - dt.timedelta(hours=EVENING_STARTS_H)).strftime("%Y%m%d")


def _load(p: Path) -> Any:
    try: return json.loads(p.read_text())
    except (OSError, ValueError): return None


def _now() -> str:
    return dt.datetime.now().isoformat(timespec="seconds")


def log(d: Path, what: str, **kw) -> None:
    """one action, with its time and where it came from ([Dlx])"""
    d.mkdir(parents=True, exist_ok=True)
    with open(d / LOG, "a") as f: f.write(json.dumps({"when": _now(), "what": what, "from": BY, **kw}) + "\n")


def _night_dir(name: str) -> Path:
    """a night's directory under the root by its name; nothing outside it"""
    if not isinstance(name, str) or not name.isdigit() or len(name) != 8: raise ValueError(f"no night {name!r}")
    return root() / name


def ran(d: Path) -> bool:
    return (Path(d) / CANDS).exists()


# ---- marks ([vO9]) ----

def validate_toys(toys: list) -> list[dict]:
    """the page's taps as the scorer's toys: a name from TOYS, a floor kind, a place in numbers"""
    kinds = {n: (h, b) for n, h, b in TOYS}; out = []
    if not isinstance(toys, list): raise ValueError("the toys must be a list")
    for i, t in enumerate(toys, 1):
        if not isinstance(t, dict) or t.get("name") not in kinds: raise ValueError(f"toy {i}: its name must come from the short list")
        if t.get("floor") not in sn.FLOORS: raise ValueError(f"toy {i}: its floor must be one of {', '.join(sn.FLOORS)}")
        try: x, y = float(t["x"]), float(t["y"])
        except (KeyError, TypeError, ValueError): raise ValueError(f"toy {i}: its place must be a number") from None
        if not (math.isfinite(x) and math.isfinite(y)): raise ValueError(f"toy {i}: its place must be a number")
        h, b = kinds[t["name"]]
        out.append({"id": f"t{i}", "name": t["name"], "x": round(x, 3), "y": round(y, 3), "half_long_m": h, "brick": b,
                    "floor": t["floor"]})
    return out


def save_marks(d: Path, toys: list, seen: str = "", by: str = BY) -> dict:
    """the night's marks.json from the page's taps; refused once the night has run, or when the marks changed since the
    page loaded them (seen: the saved stamp it had, "" for none)"""
    d = Path(d)
    if ran(d): raise ValueError("the night has already run: marks are made before it")
    cur = _load(d / MARKS)
    if ((cur or {}).get("saved") or "") != (seen or ""): raise ValueError("the marks changed since this page loaded them (look again, then save)")
    m = {"night": d.name, "frame": FRAME, "toys": validate_toys(toys), "saved": _now(), "by": by}
    d.mkdir(parents=True, exist_ok=True)
    tmp = d / (MARKS + ".tmp"); tmp.write_text(json.dumps(m, indent=1)); tmp.replace(d / MARKS)
    log(d, "marks saved", toys=len(m["toys"]), bricks=sum(t["brick"] for t in m["toys"]))
    return m


def house_map(md: Path | None = None):
    """the house's floor map as the Area page draws it (operating_area_module.house_map_image), or None"""
    from houndkeeper import operating_area as oa
    from houndkeeper.operating_area_module import house_map_image
    return house_map_image(Path(md) if md else oa.map_dir())


def marks_message(now: dt.datetime | None = None, md: Path | None = None, map_img=...) -> dict:
    """the Marks page's message: tonight's name, whether it has run, its marks so far, the map, the list and the floors"""
    name = night_name(now); d = root() / name
    return {"v": 1, "night": name, "ran": ran(d), "marks": _load(d / MARKS),
            "map": house_map(md) if map_img is ... else map_img, "frame": FRAME,
            "toys": [{"name": n, "brick": b, "long_m": round(2 * h, 3)} for n, h, b in TOYS],
            "floors": [{"key": k, "name": FLOOR_NAMES[k]} for k in sn.FLOORS]}


# ---- review ([71R], [wTj]) ----

def ran_nights() -> list[Path]:
    """the nights that ran, oldest first"""
    r = root()
    return sorted((p.parent for p in r.glob(f"*/{CANDS}") if p.parent.name.isdigit()), key=lambda p: p.name) if r.is_dir() else []


def tuning_names() -> list[str]:
    """the first sn.TUNING_NIGHTS nights that ran ([wTj])"""
    return [d.name for d in ran_nights()[:sn.TUNING_NIGHTS]]


def score(d: Path) -> dict:
    """the night scored from its files (marks, candidates, judgements) and written to result.json"""
    d = Path(d); C = _load(d / CANDS) or {}
    marks = _load(d / MARKS) or {"night": d.name, "toys": []}
    r = sn.score_night(dict(marks, night=d.name), C.get("candidates") or [], _load(d / REVIEW) or {}, d.name in tuning_names())
    r["checked"] = C.get("checked"); r["marked"] = (d / MARKS).exists(); r["marks_saved"] = marks.get("saved")
    tmp = d / (RESULT + ".tmp"); tmp.write_text(json.dumps(r, indent=1)); tmp.replace(d / RESULT)
    return r


def judge(d: Path, ids: list, verdict: str | None) -> dict:
    """the user's judgement on one flag (its candidates' ids): false_alarm, real_change, or None to take it back;
    written to review.json, logged, the night scored again"""
    d = Path(d)
    if verdict is not None and verdict not in sn.JUDGEMENTS: raise ValueError(f"a judgement is {' or '.join(sn.JUDGEMENTS)}")
    known = {c["id"] for c in (_load(d / CANDS) or {}).get("candidates") or []}
    if not ids or not set(ids) <= known: raise ValueError(f"no candidate {', '.join(sorted(set(ids or []) - known)) or '(none named)'} in this night")
    rv = _load(d / REVIEW) or {}
    for i in ids:
        if verdict is None: rv.pop(i, None)
        else: rv[i] = verdict
    tmp = d / (REVIEW + ".tmp"); tmp.write_text(json.dumps(rv, indent=1)); tmp.replace(d / REVIEW)
    log(d, "judgement taken back" if verdict is None else f"judged {verdict}", ids=list(ids))
    return score(d)


def crops(d: Path) -> tuple[dict, str]:
    """the flags' crops (review_crops/index.json): {candidate id: {"jpg": base64} or {"why": why there is none}}, and
    the index's note"""
    idx = _load(Path(d) / CROPS / "index.json")
    if not isinstance(idx, dict): return {}, "No pictures yet: real/tests/flag_crops.py makes them after the night."
    out = {}
    for cid, e in (idx.get("crops") or {}).items():
        f = Path(d) / CROPS / Path(str(e.get("file") or "")).name if e.get("file") else None
        if f is not None and f.is_file(): out[cid] = {"jpg": base64.b64encode(f.read_bytes()).decode()}
        else: out[cid] = {"why": e.get("withheld") or e.get("none") or "no picture"}
    return out, idx.get("note") or ""


def _size(m: dict) -> int:
    return len(json.dumps(m, separators=(",", ":")).encode())


def review_message(show: str | None = None, md: Path | None = None, max_bytes: int = MAX_BYTES, map_img=...) -> dict:
    """the Review page's message: every night that ran (newest first, each final or how many are left), and one night in
    full (show, else the newest not yet final, else the newest): its toys found or not, its flags with crop, place and
    judgement. At most max_bytes: judged flags' pictures dropped first, then the unjudged from the last back."""
    nights = ran_nights(); tune = set(tuning_names()); summ = []
    for d in reversed(nights):
        r = _load(d / RESULT) or score(d)
        summ.append({"name": d.name, "final": r["final"], "unjudged": r["unjudged"], "tuning": d.name in tune})
    m = {"v": 1, "nights": summ, "night": None, "flags": [], "toys": [], "pictures_note": "", "frame": FRAME,
         "map": house_map(md) if map_img is ... else map_img}
    pick = show if show in {s["name"] for s in summ} else next((s["name"] for s in summ if not s["final"]), summ[0]["name"] if summ else None)
    if pick is None: return m
    d = root() / pick; r = score(d); pics, note = crops(d)
    m["night"] = {"name": pick, "tuning": r["tuning"], "laid_out": r["laid_out"], "marked": r["marked"], "final": r["final"],
                  "found": [r["found"], r["n"]], "bricks": r["bricks"], "floor": r["floor"], "false_alarms": r["false_alarms"],
                  "real_changes": r["real_changes"], "unjudged": r["unjudged"], "candidates": r["candidates"], "checked": r["checked"]}
    m["toys"] = [{k: t[k] for k in ("id", "name", "x", "y", "floor", "brick", "found")} for t in r["toys"]]
    for k, f in enumerate(r["flags"], 1):
        p = next((pics[i] for i in f["ids"] if i in pics and "jpg" in pics[i]), None)
        why = next((pics[i]["why"] for i in f["ids"] if i in pics and "why" in pics[i]), "no picture yet")
        m["flags"].append({"k": k, "ids": f["ids"], "x": f["x"], "y": f["y"], "judged": f["judged"],
                           "jpg": p["jpg"] if p else None, "no_picture": None if p else why})
    m["pictures_note"] = note; dropped = 0
    for f in sorted(m["flags"], key=lambda f: (f["judged"] is None, -f["k"])):    # judged first, then the last back
        if _size(m) <= max_bytes: break
        if f["jpg"]: f["jpg"] = None; f["no_picture"] = "left off to fit the phone"; dropped += 1
    if dropped: m["pictures_note"] = (note + " " if note else "") + f"{dropped} picture(s) left off to fit the phone."
    return m


# ---- the DimOS side ----

@dataclass(frozen=True)
class MarksPage(Panel):
    """the toys laid out tonight, tapped on the house's floor map ([vO9]; cockpit/extensions/hk-marks.js). Sends only a
    confirmed save, which ScoreFeed validates, writes beside the night and logs."""
    kind: ClassVar[str] = "hk-marks"
    title: str = field(default="Marks", kw_only=True)

    def _channels(self) -> tuple[Channel, ...]:
        return (Channel("marks_state", dict, max_hz=1.0, delivery="latest", resend_on_subscribe=True),
                Channel("marks_cmd", dict, dir="tx", publish="shared", max_hz=2.0))

    def _channel_requests(self):
        return tuple(_request_of(c) for c in self._channels())


@dataclass(frozen=True)
class ReviewPage(Panel):
    """each flag near no mark, judged ([71R]; cockpit/extensions/hk-review.js). Sends a judgement or the night to show,
    which ScoreFeed writes beside the night and logs."""
    kind: ClassVar[str] = "hk-review"
    title: str = field(default="Review", kw_only=True)

    def _channels(self) -> tuple[Channel, ...]:
        return (Channel("review_state", dict, max_hz=1.0, delivery="latest", resend_on_subscribe=True),
                Channel("review_cmd", dict, dir="tx", publish="shared", max_hz=5.0))

    def _channel_requests(self):
        return tuple(_request_of(c) for c in self._channels())


class ScoreFeed(Module):
    """the Marks and Review pages' data: marks_state and review_state published every PUBLISH_S and after each command;
    marks_cmd (a confirmed save) and review_cmd (a judgement, or the night to show) acted on and logged"""
    marks_cmd: In[dict]
    marks_state: Out[dict]
    review_cmd: In[dict]
    review_state: Out[dict]

    @rpc
    def start(self) -> None:
        super().start()
        from reactivex.disposable import Disposable
        self._marks_msg = ""; self._review_msg = ""; self._show = None; self._map = None; self._map_stamp = None
        self._halt = threading.Event()
        self.register_disposable(Disposable(self.marks_cmd.subscribe(self.on_marks)))
        self.register_disposable(Disposable(self.review_cmd.subscribe(self.on_review)))
        threading.Thread(target=self._watch, name="score-feed", daemon=True).start()

    @rpc
    def stop(self) -> None:
        ev = getattr(self, "_halt", None)
        if ev is not None: ev.set()
        super().stop()

    def _house_map(self):
        from houndkeeper import operating_area as oa
        f = oa.map_dir() / "house_lidar.npy"; st = f.stat().st_mtime_ns if f.exists() else None
        if st != self._map_stamp: self._map, self._map_stamp = house_map(), st
        return self._map

    def publish(self) -> None:
        mp = self._house_map()
        self.marks_state.publish(dict(marks_message(map_img=mp), message=self._marks_msg))
        self.review_state.publish(dict(review_message(self._show, map_img=mp), message=self._review_msg))

    def _watch(self) -> None:
        while True:
            try: self.publish()
            except Exception as e: print(f"[score] {e!r}", file=sys.stderr)
            if self._halt.wait(PUBLISH_S): return

    def on_marks(self, cmd) -> None:
        try:
            cmd = cmd if isinstance(cmd, dict) else json.loads(cmd)
            if cmd.get("op") != "save": raise ValueError(f"unknown command {cmd.get('op')!r}")
            if cmd.get("confirmed") is not True: raise ValueError("the save was not confirmed")
            m = save_marks(_night_dir(cmd.get("night")), cmd.get("toys"), cmd.get("seen") or "")
            n = len(m["toys"]); self._marks_msg = f"saved {m['saved'].replace('T', ' ')}: {n} toy{'' if n == 1 else 's'} for the night of {m['night']}"
        except Exception as e: self._marks_msg = f"not saved: {e}"
        try: self.publish()
        except Exception as e: print(f"[score] {e!r}", file=sys.stderr)

    def on_review(self, cmd) -> None:
        try:
            cmd = cmd if isinstance(cmd, dict) else json.loads(cmd)
            d = _night_dir(cmd.get("night"))
            if not ran(d): raise ValueError(f"no night {cmd.get('night')!r} has run")
            if cmd.get("op") == "show": self._show = d.name; self._review_msg = ""
            elif cmd.get("op") == "judge":
                r = judge(d, cmd.get("ids") or [], cmd.get("verdict")); self._show = d.name
                self._review_msg = "every flag judged: the night is final" if r["final"] else f"{r['unjudged']} left to judge"
            else: raise ValueError(f"unknown command {cmd.get('op')!r}")
        except Exception as e: self._review_msg = f"not done: {e}"
        try: self.publish()
        except Exception as e: print(f"[score] {e!r}", file=sys.stderr)
