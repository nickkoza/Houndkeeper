#!/usr/bin/env python3
"""The morning log of a sim night with a section scored against the sim's truth ([9X9]): morning_log.write's log, the
household's part unchanged, plus the simulator's score of the night (score_tidy.py's score.json), marked as such and
for engineering only. A scorer: it reads the truth the robot's side never does ([1B2]); a real night gets no section.
Usage: morning_score.py <night dir> [--out DIR]   (dimos venv python)"""
from __future__ import annotations

import argparse, html, json, sys
from pathlib import Path

HK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HK / "src"))
from houndkeeper import morning_log as ml   # noqa: E402

CSS = (".truth{border:1px dashed var(--accent);border-radius:6px;padding:4px 12px 12px;margin-top:28px}"
       ".truth summary{color:var(--ink)}")


def scored_against_truth(night, kind) -> dict | None:
    """a sim night's score against the sim's truth (score_tidy.py's score.json): for engineering, never in his prose"""
    if kind != "sim": return None
    p = night.dir / "score.json"
    if not p.exists(): return None
    s = json.loads(p.read_text())
    return {"source": "score.json", "items_placed": s.get("items"), "found": s.get("found"), "binned": s.get("binned"),
            "grippable": s.get("binnable"), "too_big": s.get("too_big"), "too_big_reported": s.get("too_big_reported"),
            "stops_for_nothing": s.get("stops_for_nothing"),
            "per_item": [{k: it.get(k) for k in ("item", "tier", "binned", "said_too_wide", "handled")} for it in s.get("per_item", [])]}


def section(truth, src) -> str:
    """the page's section for the score, marked as the simulator's; src(name) -> a link to the night's file"""
    esc = html.escape
    per = "".join(f'<tr><td>{esc(str(i["item"]))}</td><td>{esc(str(i["tier"]))}</td><td>{"binned" if i["binned"] else ("reported too big" if i["said_too_wide"] else "left")}</td></tr>'
                  for i in truth["per_item"])
    return (f'<style>{CSS}</style><details class="truth"><summary>Scored against the simulation\'s truth (sim only)</summary>'
            f'<p class="note">Barrymore knew none of this: it is the simulator\'s record of where everything really was, '
            f'for judging the night. Source: <a href="{src(truth["source"])}">{esc(truth["source"])}</a>.</p><table>'
            f'<tr><td>Items placed tonight</td><td class="v">{truth["items_placed"]}</td></tr>'
            f'<tr><td>Found (a stop within reach of it)</td><td class="v">{truth["found"]}</td></tr>'
            f'<tr><td>Binned, of those that fit his grip</td><td class="v">{truth["binned"]} of {truth["grippable"]}</td></tr>'
            f'<tr><td>Too big for the grip; reported so</td><td class="v">{truth["too_big"]}; {truth["too_big_reported"]}</td></tr>'
            f'<tr><td>Stops near no item</td><td class="v">{truth["stops_for_nothing"]}</td></tr></table>'
            f'<table><tr><th>Item</th><th>Tier</th><th>Became</th></tr>{per}</table></details>')


def scored(night, kind):
    """morning_log.write's scored hook: (the score's figures, its section) for a sim night with a score, else None"""
    T = scored_against_truth(night, kind)
    return (T, lambda src: section(T, src)) if T else None


def write(night_dir, out=None, screen=None) -> Path:
    return ml.write(night_dir, out, scored=scored, screen=screen)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0]); ap.add_argument("night")
    ap.add_argument("--out", help="where to write (default: the night's directory)")
    a = ap.parse_args(argv); p = write(a.night, a.out)
    print(p); print("\n".join(json.loads((p.parent / "morning_log.json").read_text())["prose"]))


if __name__ == "__main__":
    main()
