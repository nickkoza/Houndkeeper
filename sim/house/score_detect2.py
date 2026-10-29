#!/usr/bin/env python3
"""The locked detector (detect_changes2.py) run over a run's nights and scored against the sim's truth: each item of a
night's truth found, missed or unobserved, the false alarms, the stains, the rug, an overlay picture per night, and
detect_results.json for the run. A scorer: the truth is read here, after each night's blobs are found, never by the
detector ([1B2]); the blobs it writes (detect_nN.json) carry the scorer's "matched" beside the detector's own.
Usage: score_detect2.py --seed 7 --nights 4 [--run-dir DIR] [detect_changes2 options]   (dimos venv python)"""
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

import detect_changes2 as dc2
from detect_changes2 import RES, HX, HY, w2rc


class Scorer:
    """the detector's nights matched with the truth: night() before the blobs are written (it sets their "matched"),
    report() after, finish() once the run is done"""

    def __init__(self, run, seed):
        self.run, self.seed, self.results = Path(run), seed, {}

    def night(self, c):
        run, n, lab, klab, pres, region, blobs = self.run, c["n"], c["lab"], c["klab"], c["pres"], c["region"], c["blobs"]
        t = json.loads((run / f"nights_s{self.seed}_n{n}.truth.json").read_text())
        tprev = json.loads((run / f"nights_s{self.seed}_n{n-1}.truth.json").read_text())

        def disk(x, y, r):
            r0, c0 = w2rc(x, y); rp = int(r / RES)
            yy, xx = np.ogrid[-rp:rp + 1, -rp:rp + 1]; d = (yy ** 2 + xx ** 2) <= rp ** 2
            sl = (slice(max(r0 - rp, 0), r0 + rp + 1), slice(max(c0 - rp, 0), c0 + rp + 1))
            return sl, d[: pres[sl].shape[0], : pres[sl].shape[1]] if r0 - rp >= 0 and c0 - rp >= 0 else d[-pres[sl].shape[0]:, -pres[sl].shape[1]:]

        per, matched = {}, set()
        for it in t["items"]:
            r = it.get("length_m", 0.30) + 0.05 if it["kind"] == "string" else max(it["half"]) + 0.06
            sl, d = disk(it["x"], it["y"], r)
            if region[sl][d].mean() < 0.3: status = "unobserved"
            else:
                hit = np.unique(lab[sl][d & pres[sl]]); hit = hit[hit > 0]; matched |= set(hit.tolist())
                status = "found" if len(hit) else "missed"
            key = it["kind"]; per.setdefault(key, {"found": 0, "missed": 0, "unobserved": 0})[status] += 1
            it["_status"] = status
        stain_flags = 0
        for s_ in t["stains"]:
            sl, d = disk(s_["x"], s_["y"], s_["r"] + 0.05)
            hit = np.unique(lab[sl][d & pres[sl]]); hit = hit[hit > 0]
            stain_flags += int(len(hit) > 0); matched |= set(hit.tolist())
        now_ids = {it["id"] for it in t["items"]}
        gone_as_present = 0
        for it in tprev["items"]:
            if it["id"] in now_ids: continue
            sl, d = disk(it["x"], it["y"], max(it["half"]) + 0.06)
            hit = np.unique(lab[sl][d & pres[sl]]); hit = hit[hit > 0]
            gone_as_present += int(len(hit) > 0); matched |= set(hit.tolist())
        false_alarms = len(c["present_ids"] - matched)
        for b in blobs: b["matched"] = b["id"] in matched
        rug_flag = None
        if t.get("rug_new_tonight"):
            r = t["rug"]; rr0, cc0 = w2rc(r["x"] - r["w"] / 2, r["y"] + r["h"] / 2); rr1, cc1 = w2rc(r["x"] + r["w"] / 2, r["y"] - r["h"] / 2)
            ids = np.unique(lab[rr0:rr1, cc0:cc1]); ids = ids[ids > 0]
            kinds = [b["kind"] for b in blobs if b["id"] in ids]
            rug_flag = {k: kinds.count(k) for k in set(kinds)}
        stains_known = 0
        for s_ in t["stains"]:
            sl, d = disk(s_["x"], s_["y"], s_["r"] + 0.05); stains_known += int((klab[sl][d] > 0).any())
        marks_off_stain = 0
        for m_ in c["marks"]:
            rr, cc = np.nonzero(klab == m_["id"]); x = -HX + cc.mean() * RES; y = HY - rr.mean() * RES
            near = any(np.hypot(x - s_["x"], y - s_["y"]) < s_["r"] + 0.15 for s_ in t["stains"])
            marks_off_stain += int(not near)
        self.results[n] = {"stains_recognized": f"{stains_known}/{len(t['stains'])}", "marks_not_on_stains": marks_off_stain,
                           "baseline_nights": c["prev"], "per_kind": per, "false_alarms": false_alarms,
                           "stains_flagged": stain_flags, "gone_flagged_as_present": gone_as_present, "rug_blobs": rug_flag,
                           "compared_area_m2": round(region.sum() * RES * RES, 1)}
        c["truth"] = t

    def report(self, c):
        run, n, lab, klab, pres, blobs, t = self.run, c["n"], c["lab"], c["klab"], c["pres"], c["blobs"], c["truth"]
        (run / f"items_n{n}.json").write_text(json.dumps([{"id": it["id"], "kind": it["kind"], "status": it["_status"]} for it in t["items"]], indent=0))
        img = Image.fromarray((c["rgb_n"] * 0.55).astype(np.uint8)); ov = np.array(img)      # the overlay
        ov[pres] = (40, 200, 180); ov[np.isin(lab, [b["id"] for b in blobs if b["kind"] == "gone"])] = (150, 150, 255)
        ov[np.isin(lab, [b["id"] for b in blobs if b["kind"] == "floor_change"])] = (230, 170, 40)
        ov[klab > 0] = (60, 110, 255)
        img = Image.fromarray(ov); dr = ImageDraw.Draw(img)
        for it in t["items"]:
            r0, c0 = w2rc(it["x"], it["y"]); col = {"found": (40, 220, 120), "missed": (240, 60, 60), "unobserved": (130, 130, 130)}[it["_status"]]
            dr.ellipse([c0 - 22, r0 - 22, c0 + 22, r0 + 22], outline=col, width=4)
        img.save(run / f"detect_n{n}.png")
        R = self.results[n]; per = R["per_kind"]
        tot = {k: sum(v[k] for v in per.values()) for k in ("found", "missed", "unobserved")}
        print(f"night {n} (baseline {c['prev']}): found {tot['found']}/{tot['found']+tot['missed']} observed items "
              f"({tot['unobserved']} unobserved); by kind {per}; false alarms {R['false_alarms']}; "
              f"stains recognized as known {R['stains_recognized']} (other marks {R['marks_not_on_stains']}); stains flagged as new {R['stains_flagged']}; gone-as-present {R['gone_flagged_as_present']}; rug {R['rug_blobs']}; compared {R['compared_area_m2']} m2")

    def finish(self):
        (self.run / "detect_results.json").write_text(json.dumps(self.results, indent=1))


def main(argv=None):
    dc2.main(argv, scorer=Scorer)


if __name__ == "__main__":
    main()
