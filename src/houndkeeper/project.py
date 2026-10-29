"""The project service ([eTc]): the project's own pages on the phone cockpit with no robot or sim running ([C3p], [ehV]).
A small DimOS blueprint the laptop keeps up; it registers with the relay as a participant of its own (robot id
"project") beside the robots, and carries only the project's pages. First page: Last night ([RzM],
cockpit/extensions/hk-last-night.js): each recent night's morning log and numbers, what failed, and the newest findings.
Then Marks and Review, a real night's score made from the phone ([vO9], [71R]; houndkeeper.night_scoring, [oWO]).

Last night is read only ([Dlx]): its channel is rx. The scoring pages send only a confirmed save of the marks and the
user's judgements, which are written beside the real night and logged; nothing is sent to the robot or run.

    dimos --robot-id project --relay-url http://127.0.0.1:7780 --viewer none run houndkeeper.project

(the relay on loopback for tests; in production on the laptop's Tailscale address with its certificate and auth file,
[rhP]; real/bringup/project.service is the systemd user unit's template).

A night is a tidy run's directory under HK_PROJECT_RUNS (default data/runs/dimos_tidy). Only nights whose night.json
names their house are listed, and never a hold-out house ([jnG]): night.json is the first and, for a hold-out, the only
file opened. The message is compact JSON of at most MAX_BYTES ([WkQ])."""
from __future__ import annotations

import json, os, re, sys, threading, time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, ClassVar

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.web.cockpit import Channel, Panel, _request_of, cockpit

from houndkeeper.night_scoring import MarksPage, ReviewPage, ScoreFeed

HK = Path(__file__).resolve().parents[2]
RUNS = HK / "data" / "runs" / "dimos_tidy"
FINDINGS = HK / "docs" / "research"                # the write-ups shown as findings; README.md is its index
HOLDOUT = frozenset({80, 81, *range(100, 120)})   # sim/house/make_house.HOLDOUT; tests/test_project.py holds them equal
NIGHTS = 5                                        # the newest nights listed
FINDINGS_N = 4                                    # the newest findings documents
EXCERPT_CHARS = 2400                              # a finding's opening section, at most
DOC_CHARS = 32000                                 # a finding's whole text on the page, at most
MAX_BYTES = 256 * 1024                            # the whole message, compact JSON ([WkQ])
CHECK_S = 10.0                                    # how often it looks for a newer night or finding, and publishes ([E2e])


def _load(p: Path) -> Any:
    try: return json.loads(p.read_text())
    except (OSError, ValueError): return None


def house_of(d: Path) -> int | None:
    """the night's house from its night.json alone; None when it names none (such a night is never listed, [jnG])"""
    n = _load(d / "night.json")
    s = n.get("seed") if isinstance(n, dict) else None
    return s if isinstance(s, int) and not isinstance(s, bool) else None


def listed_nights(root: Path, n: int = NIGHTS) -> list[tuple[Path, int]]:
    """the newest n nights (directories named by their start, YYYYmmdd_HHMMSS) whose house is known and not held out"""
    out = []
    for d in sorted((p for p in Path(root).glob("*/night.json")), key=lambda p: p.parent.name, reverse=True):
        h = house_of(d.parent)
        if h is None or h in HOLDOUT: continue
        out.append((d.parent, h))
        if len(out) == n: break
    return out


def _num(label: str, value: Any, text: str, source: str, note: str = "") -> dict:
    return {"label": label, "value": value, "text": text, "note": note, "source": source}


def _speed(d: Path, score: dict | None) -> dict | None:
    sp = (score or {}).get("speed")
    if not isinstance(sp, dict):
        sys.path.insert(0, str(HK / "sim" / "dimos"))
        import run_speed
        sp = run_speed.of_run(d)
    return sp if sp and sp.get("sim_wall") is not None else None


def numbers(d: Path, night: dict) -> tuple[list[dict], str]:
    """the night's numbers, each with the file it came from: score.json where the sim scored it, else the robot's own
    report.json (found and false stops need the truth: not there). Returns (numbers, the source of the counts)."""
    score = _load(d / "score.json"); report = _load(d / "report.json"); energy = _load(d / "energy.json")
    out: list[dict] = []
    if isinstance(score, dict):
        src = "score.json"
        if score.get("items") is not None:
            out.append(_num("Found", score.get("found"), f'{score.get("found")} of {score["items"]}', src, "items in the house"))
            out.append(_num("Binned", score.get("binned"), f'{score.get("binned")} of {score.get("binnable", score["items"])}', src, "of the binnable"))
        out.append(_num("False stops", score.get("stops_for_nothing"), f'{score.get("stops_for_nothing")} of {score.get("stops")}', src, "stops for nothing"))
    elif isinstance(report, dict):
        src = "report.json"; items = report.get("items") or []
        binned = sum(1 for it in items if isinstance(it, dict) and it.get("outcome") == "binned")
        out.append(_num("Stops", len(items), str(len(items)), src))
        out.append(_num("Binned", binned, str(binned), src))
    else:
        src = ""
    if isinstance(energy, dict) and energy.get("air_left_pct") is not None:
        out.append(_num("Battery left", energy["air_left_pct"], f'{energy["air_left_pct"]:.0f} %', "energy.json",
                        f'{energy.get("air_used_Wh", 0):.0f} Wh used'))
    t = (score or {}).get("t_s") if isinstance(score, dict) else None
    if t is not None:
        out.append(_num("Time", t, f"{t / 60:.0f} min" if t >= 60 else f"{t:.1f} s", "score.json", "the robot's clock"))
    elif night.get("started") and night.get("ended"):
        m = (night["ended"] - night["started"]) / 60
        out.append(_num("Time", round(m * 60), f"{m:.0f} min", "night.json", "wall clock"))
    try: sp = _speed(d, score if isinstance(score, dict) else None)
    except Exception: sp = None
    if sp is not None:
        out.append(_num("Sim speed", sp["sim_wall"], f'{sp["sim_wall"]:.2f}x real time',
                        "score.json" if isinstance((score or {}).get("speed"), dict) else "dimos.log",
                        "latency-flattered" if sp.get("latency_flattered") else ""))
    return out, src


def failures(d: Path) -> list[str]:
    """what failed: the run's end state and error, waypoints not reached, the stops that came to nothing by reason, the
    items left by tier (score.json), or the robot's own outcomes (report.json)"""
    score = _load(d / "score.json"); out: list[str] = []
    if isinstance(score, dict):
        if score.get("state") not in (None, "done"): out.append(f'Ended in state "{score["state"]}"')
        if score.get("error"): out.append(f'Error: {score["error"]}')
        for w in score.get("waypoints_not_reached") or []:
            out.append(f'Waypoint {w.get("waypoint")} not reached: {w.get("not_reached")}')
        for why, k in sorted((score.get("outcomes") or {}).items(), key=lambda kv: -kv[1]):
            if why != "binned": out.append(f"{k} stop{'s' if k != 1 else ''}: {why.removeprefix('review: ')}")
        if score.get("stops") == 0:
            st = _load(d / "status.json"); log = st.get("log") if isinstance(st, dict) else None
            last = log[-1].strip() if isinstance(log, list) and log else ""
            out.append(f'No stops at all, over after {score.get("t_s")} s' + (f'; its last line: "{last[:160]}"' if last else ""))
        for tier, (got, of) in (score.get("tiers") or {}).items():
            if got < of: out.append(f"{tier.replace('_', ' ')} items: {got} of {of} binned")
        return out
    report = _load(d / "report.json")
    if isinstance(report, dict):
        if report.get("state") not in (None, "done"): out.append(f'Ended in state "{report["state"]}"')
        counts: dict[str, int] = {}
        for it in report.get("items") or []:
            o = it.get("outcome") if isinstance(it, dict) else None
            if o and o != "binned": counts[o] = counts.get(o, 0) + 1
        for why, k in sorted(counts.items(), key=lambda kv: -kv[1]):
            out.append(f"{k} stop{'s' if k != 1 else ''}: {why.removeprefix('review: ')}")
        for w in report.get("sweep") or []:
            if isinstance(w, dict) and "not_reached" in w: out.append(f'Waypoint {w.get("waypoint")} not reached: {w["not_reached"]}')
    return out


def morning(d: Path) -> dict | None:
    """the night's morning log ([uoW]): its prose (the text), the finds in a line each, and its figures with sources"""
    doc = _load(d / "morning_log.json")
    if not isinstance(doc, dict): return None
    from houndkeeper.morning_log import ENG
    fx = doc.get("facts") or {}
    return {"prose": list(doc.get("prose") or []),
            "finds": [{"k": f.get("k"), "what": f.get("what"), "fate": f.get("fate"), "clock": f.get("clock")}
                      for f in doc.get("finds") or [] if f.get("fate") != "nothing"],
            "figures": [{"label": lab, "text": fx[k]["text"], "source": fx[k]["source"]} for k, lab in ENG if k in fx],
            "pictures_note": doc.get("pictures_note") or "", "source": "morning_log.json"}


def night_summary(d: Path, house: int) -> dict:
    night = _load(d / "night.json") or {}
    nums, src = numbers(d, night)
    stamp = d.name
    try: when = time.strftime("%a %-d %b %Y, %-I:%M %p", time.strptime(stamp, "%Y%m%d_%H%M%S")).replace("AM", "am").replace("PM", "pm")
    except ValueError: when = stamp
    return {"id": stamp, "when": when, "house": house, "mode": night.get("mode"), "item_seed": night.get("item_seed"),
            "dir": str(d.relative_to(HK)) if d.is_relative_to(HK) else str(d), "counts_from": src,
            "scored_against_truth": src == "score.json", "numbers": nums, "failed": failures(d), "morning": morning(d)}


# ---- findings ----

_PLACEHOLDER = re.compile(r"^@@[A-Z_]+@@\s*$", re.M)          # a finding's slot still to be filled


def finding(p: Path) -> dict:
    """a findings document: its title, its Answer section if it has one, else what comes before its first section,
    else its first section; and its whole text, each bounded"""
    text = _PLACEHOLDER.sub("", p.read_text(errors="replace"))
    lines = text.splitlines()
    title = next((l[2:].strip() for l in lines if l.startswith("# ")), p.stem)
    secs: list[tuple[str, list[str]]] = [("", [])]
    for l in lines:
        if l.startswith("# ") and not secs[0][1] and len(secs) == 1: continue
        if l.startswith("## "): secs.append((l[3:].strip(), []))
        else: secs[-1][1].append(l)
    answer = next((s for s in secs[1:] if s[0].lower().startswith("answer")), None)
    lead = "\n".join(secs[0][1]).strip()
    head, body = (answer[0], "\n".join(answer[1])) if answer else (("", lead) if lead else ((secs[1][0], "\n".join(secs[1][1])) if len(secs) > 1 else ("", "")))
    body = body.strip(); cut = len(body) > EXCERPT_CHARS
    full = "\n".join(l for l in lines if not l.startswith("# ")).strip(); full_cut = len(full) > DOC_CHARS   # the title shown above
    return {"name": p.name, "title": title, "date": time.strftime("%-d %b %Y", time.localtime(p.stat().st_mtime)),
            "mtime": p.stat().st_mtime, "path": str(p.relative_to(HK)) if p.is_relative_to(HK) else p.name,
            "section": head, "excerpt": body[:EXCERPT_CHARS].rsplit("\n", 1)[0] if cut else body, "excerpt_cut": cut,
            "full": full[:DOC_CHARS] if full_cut else full, "full_cut": full_cut}


def newest_findings(root: Path, n: int = FINDINGS_N) -> list[dict]:
    docs = sorted((p for p in Path(root).glob("*.md") if p.name != "README.md"), key=lambda p: (p.stat().st_mtime, p.name), reverse=True)[:n]
    return [finding(p) for p in docs]


def _size(m: dict) -> int:
    return len(json.dumps(m, separators=(",", ":")).encode())


def last_night_message(runs: Path | None = None, findings_dir: Path | None = None, max_bytes: int = MAX_BYTES) -> dict:
    """the Last night page's message ([RzM]): v, the newest nights and the newest findings, at most max_bytes ([WkQ]):
    findings' whole texts dropped from the oldest back, then the older nights' morning logs, then older nights, then
    excerpts shortened. Always a message, even with no night (an empty list) or no findings."""
    runs = Path(runs or RUNS); findings_dir = Path(findings_dir or FINDINGS)
    m = {"v": 1, "made": time.strftime("%Y-%m-%dT%H:%M:%S"), "nights": [night_summary(d, h) for d, h in listed_nights(runs)],
         "findings": newest_findings(findings_dir) if findings_dir.is_dir() else [], "trimmed": []}
    for f in reversed(m["findings"]):
        if _size(m) <= max_bytes: break
        if f["full"]: f["full"] = ""; f["full_cut"] = True; m["trimmed"].append(f"whole text of {f['name']}")
    for nt in reversed(m["nights"][1:]):
        if _size(m) <= max_bytes: break
        if nt["morning"]: nt["morning"] = None; m["trimmed"].append(f"morning log of {nt['id']}")
    while _size(m) > max_bytes and len(m["nights"]) > 1:
        m["trimmed"].append(f"night {m['nights'].pop()['id']}")
    for f in m["findings"]:
        if _size(m) <= max_bytes: break
        f["excerpt"] = f["excerpt"][:400]; f["excerpt_cut"] = True
    while _size(m) > max_bytes and m["findings"]: m["findings"].pop()
    if _size(m) > max_bytes and m["nights"] and m["nights"][0]["morning"]: m["nights"][0]["morning"]["prose"] = m["nights"][0]["morning"]["prose"][:2]
    if _size(m) > max_bytes:
        m.update(nights=[], findings=[], trimmed=["everything: too large for the phone"])
    return m


def _stamp(runs: Path, findings_dir: Path) -> tuple:
    """what changes when the message would: the listed nights' files and the findings' times"""
    ks = []
    for p in sorted(runs.glob("*/night.json"))[-3 * NIGHTS:]:
        for f in ("night.json", "score.json", "report.json", "morning_log.json", "energy.json"):
            try: ks.append((str(p.parent / f), (p.parent / f).stat().st_mtime))
            except OSError: pass
    ks += [(str(p), p.stat().st_mtime) for p in findings_dir.glob("*.md")] if findings_dir.is_dir() else []
    return tuple(ks)


# ---- the DimOS side: a module and the blueprint ----

@dataclass(frozen=True)
class LastNight(Panel):
    """the Last night page ([RzM]; cockpit/extensions/hk-last-night.js). Reads only: its one channel is rx ([Dlx])."""
    kind: ClassVar[str] = "hk-last-night"
    title: str = field(default="Last night", kw_only=True)

    def _channels(self) -> tuple[Channel, ...]:
        return (Channel("last_night", dict, max_hz=0.2, delivery="latest", resend_on_subscribe=True),)

    def _channel_requests(self):
        return tuple(_request_of(c) for c in self._channels())



class ProjectFeed(Module):
    """the Last night message onto the last_night channel every CHECK_S, made again only when a night or a finding
    changed: published again unchanged, so a page opened while another viewer already watches gets it within CHECK_S
    ([E2e]). Roots: HK_PROJECT_RUNS, HK_PROJECT_FINDINGS, else RUNS and FINDINGS."""
    last_night: Out[dict]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event(); self._seen: tuple | None = None; self._msg: dict | None = None

    def roots(self) -> tuple[Path, Path]:
        return Path(os.environ.get("HK_PROJECT_RUNS") or RUNS), Path(os.environ.get("HK_PROJECT_FINDINGS") or FINDINGS)

    @rpc
    def start(self) -> None:
        super().start(); self._stop.clear()
        threading.Thread(target=self._watch, name="project-feed", daemon=True).start()

    @rpc
    def stop(self) -> None:
        self._stop.set(); super().stop()

    def check(self) -> dict:
        """the message, made again only if anything it reads changed since the last look; published each time"""
        runs, fd = self.roots(); key = _stamp(runs, fd)
        if key != self._seen or self._msg is None:
            self._msg = last_night_message(runs, fd); self._seen = key
        self.last_night.publish(self._msg)
        return self._msg

    def _watch(self) -> None:
        while True:
            try: self.check()
            except Exception as e: print(f"[project] {e}", file=sys.stderr)
            if self._stop.wait(CHECK_S): return


def project_layout() -> dict:
    return dict(layout=LastNight(), pages=[MarksPage(), ReviewPage()])


project = autoconnect(cockpit(**project_layout()), ProjectFeed.blueprint(), ScoreFeed.blueprint())
