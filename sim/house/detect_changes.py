#!/usr/bin/env python3
"""Floor change detection over a multi-night timeline, scored against ground truth.

Per night n >= 2: baseline = per-cell median of up to the 3 previous nights' mosaics (so anything
that stays 2+ nights is absorbed: the promotion rule). Candidates = cells whose colour (chromaticity)
shifts or that get brighter relative to their surroundings; darkening (shadows) is ignored. Floor near
structures DimOS maps on every night is skipped. Blobs are split into "something is here" vs
"something left" by comparing to their surroundings. Ground truth is only used for scoring.
Usage: detect_changes.py --seed 7 --nights 4   (dimos venv python)"""
import argparse, json
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
from PIL import Image, ImageDraw

RES, HX, HY = 0.01, 8.4, 4.9
TWO_VIEW = False   # frozen for held-out: maximise catches; the arm's look-closer step handles false alarms
TC, TB, MIN_PX, BIG_PX = 0.06, 0.30, 4, 5000     # chroma shift, brightening, min blob, "floor-sized" blob (0.5 m^2)


def w2rc(x, y):
    return int((HY - y) / RES), int((x + HX) / RES)


def feats(rgb, valid):
    """Colour and brightness relative to the surrounding floor, so the tint and level of the light cancel out."""
    f = rgb.astype(np.float32); I = f.mean(-1)
    V = valid.astype(np.float32); den = np.maximum(ndi.gaussian_filter(V, 25), 1e-3)
    L = ndi.gaussian_filter(I * V, 25) / den
    c = f / (f.sum(-1, keepdims=True) + 1.0)
    cl = np.stack([ndi.gaussian_filter(c[..., k] * V, 25) / den for k in range(3)], -1)
    return c - cl, I / np.maximum(L, 1.0)


def tolerant_diff(cn, rn, cb, rb, slack=2):
    """Per-pixel change vs the baseline, allowing up to `slack` px (cm) of misalignment between nights."""
    dC = np.full(rn.shape, np.inf, np.float32); br = np.full(rn.shape, np.inf, np.float32)
    for dy in range(-slack, slack + 1):
        for dx in range(-slack, slack + 1):
            cbs = np.roll(cb, (dy, dx), (0, 1)); rbs = np.roll(rb, (dy, dx), (0, 1))
            dC = np.minimum(dC, np.linalg.norm(cn - cbs, axis=-1)); br = np.minimum(br, np.abs(rn - rbs) + 0 * (rn - rbs))
            br = np.where(rn - rbs < br, rn - rbs, br)
    return dC, br


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--seed", type=int, default=7); ap.add_argument("--nights", type=int, default=4)
    ap.add_argument("--run-dir", default=None, help="read mosaics / write results here (IO only)")
    ap.add_argument("--model-dir", default=None, help="baseline = composite floor model after the previous night (floor_model_after_nightN.png), with tonight aligned to it")
    a = ap.parse_args()
    run = Path(a.run_dir) if a.run_dir else Path(__file__).resolve().parent / "runs" / f"nights_s{a.seed}"
    M = {n: np.load(run / f"mosaic_n{n}.npz") for n in range(1, a.nights + 1) if (run / f"mosaic_n{n}.npz").exists()}
    occ_count = np.sum([M[n]["occ"] for n in M], axis=0)
    static = occ_count >= 2                                               # mapped as occupied on 2+ nights
    band = ndi.distance_transform_edt(~static) * RES < 0.40
    results = {}
    for n in sorted(M)[1:]:
        prev = [k for k in range(max(1, n - 3), n) if k in M]
        rgb_n = M[n]["rgb"]; Vn = np.isfinite(M[n]["dist"].astype(np.float32))
        stack = np.ma.masked_array(np.stack([M[k]["rgb"] for k in prev]).astype(np.float32),
                                   mask=~np.stack([np.isfinite(M[k]["dist"].astype(np.float32)) for k in prev])[..., None].repeat(3, -1))
        base = np.ma.median(stack, axis=0); Vb = ~np.ma.getmaskarray(base)[..., 0]; base = base.filled(0).astype(np.uint8)
        if a.model_dir and (Path(a.model_dir) / f"floor_model_after_night{n-1}.png").exists():
            from PIL import Image as _I; import floor_model as fm
            mdl = np.asarray(_I.open(Path(a.model_dir) / f"floor_model_after_night{n-1}.png").convert("RGB"))[: rgb_n.shape[0], : rgb_n.shape[1]]
            pad = np.zeros_like(rgb_n); pad[: mdl.shape[0], : mdl.shape[1]] = mdl; base = pad; Vb = base.sum(-1) > 0
            al, Va, _ = fm.align(rgb_n.astype(np.float32), Vn, base.astype(np.float32), Vb)   # tonight aligned to the model
            rgb_n = np.clip(al, 0, 255).astype(np.uint8); Vn = Va
        region = Vn & Vb & ~band
        cn, rn = feats(rgb_n, Vn); cb, rb = feats(base, Vb)
        dC, bright = tolerant_diff(cn, rn, cb, rb)
        if TWO_VIEW and "rgb2" in M[n].files:                            # confirm with the second view (off: max-catch)
            V2 = np.isfinite(M[n]["dist2"].astype(np.float32))
            cn2, rn2 = feats(M[n]["rgb2"], V2)
            dC2, bright2 = tolerant_diff(cn2, rn2, cb, rb)
            both = ((dC > TC) | (bright > TB)) & ((dC2 > TC) | (bright2 > TB)) & V2
            dC = np.where(both, dC, 0); bright = np.where(both, bright, 0)
        core = ndi.binary_erosion(region, iterations=6)              # comparisons are unreliable at the edge of seen floor
        cand = core & ((dC > TC) | (bright > TB))
        cand = ndi.binary_closing(cand, iterations=2) & core
        rim = region & ~core
        lab, nb = ndi.label(cand); blobs = []
        for i, sl in enumerate(ndi.find_objects(lab), 1):
            m = lab[sl] == i; area = int(m.sum())
            if area < MIN_PX: continue
            if (ndi.binary_dilation(m, iterations=2) & rim[sl]).any(): continue      # touches the edge of seen floor
            pad = tuple(slice(max(s.start - 8, 0), s.stop + 8) for s in sl)
            mm = lab[pad] == i; ring = ndi.binary_dilation(mm, iterations=6) & ~mm & region[pad]
            if ring.sum() < 5: continue
            here = np.linalg.norm(cn[pad][mm].mean(0) - cn[pad][ring].mean(0)) + max(0, rn[pad][mm].mean() - rn[pad][ring].mean())
            left = np.linalg.norm(cb[pad][mm].mean(0) - cb[pad][ring].mean(0)) + abs(rb[pad][mm].mean() - rb[pad][ring].mean())
            kind = "floor_change" if area > BIG_PX else ("present" if here >= 0.8 * left else "gone")
            peak = float(max(dC[sl][m].max(), bright[sl][m].max()))
            blobs.append({"id": i, "area_cm2": area, "kind": kind, "sl": sl, "here": round(float(here), 4), "peak": round(peak, 4)})
        # Known marks: spots that stand out from the floor around them tonight AND in the baseline (stains).
        def odd(c, r):
            V = region.astype(np.float32)
            cs = np.stack([ndi.gaussian_filter(c[..., k] * V, 25) / np.maximum(ndi.gaussian_filter(V, 25), 1e-3) for k in range(3)], -1)
            return (r < 0.8) | (np.linalg.norm(c - cs, axis=-1) > TC)
        known = region & ~cand & odd(cn, rn) & odd(cb, rb)
        known = ndi.binary_opening(known, iterations=2)            # drops thin grout lines
        klab, _ = ndi.label(known); marks = []
        for i, sl in enumerate(ndi.find_objects(klab), 1):
            area = int((klab[sl] == i).sum())
            if 50 <= area <= BIG_PX: marks.append({"id": i, "area_cm2": area})
            else: klab[klab == i] = 0
        # Lidar side: obstacles DimOS mapped tonight that aren't there on other nights (things that stand up)
        others = np.sum([M[k]["occ"] for k in M if k != n], axis=0)
        tall = M[n]["occ"] & (others == 0) & core
        tall = ndi.binary_opening(tall, iterations=1)
        tlab, _ = ndi.label(tall); off = int(lab.max())
        for i, sl in enumerate(ndi.find_objects(tlab), 1):
            m = tlab[sl] == i
            if m.sum() < 20: continue
            if (lab[sl][m] > 0).any(): continue                   # the camera already flagged it
            lab[sl][m] = off + i
            blobs.append({"id": off + i, "area_cm2": int(m.sum()), "kind": "present", "sl": sl, "here": None, "peak": None, "source": "lidar"})
        present_ids = {b["id"] for b in blobs if b["kind"] == "present"}
        pres = np.isin(lab, list(present_ids))
        t = json.loads((run / f"nights_s{a.seed}_n{n}.truth.json").read_text())
        tprev = json.loads((run / f"nights_s{a.seed}_n{n-1}.truth.json").read_text())

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
        false_alarms = len(present_ids - matched)
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
        for m_ in marks:
            rr, cc = np.nonzero(klab == m_["id"]); x = -HX + cc.mean() * RES; y = HY - rr.mean() * RES
            m_["x"], m_["y"] = round(float(x), 3), round(float(y), 3)
            near = any(np.hypot(x - s_["x"], y - s_["y"]) < s_["r"] + 0.15 for s_ in t["stains"])
            marks_off_stain += int(not near)
        for b in blobs:
            rr, cc = np.nonzero(lab[b["sl"]] == b["id"]); r0, c0 = b["sl"][0].start, b["sl"][1].start
            b["x"] = round(float(-HX + (cc.mean() + c0) * RES), 3); b["y"] = round(float(HY - (rr.mean() + r0) * RES), 3)
            b["bbox_rc"] = [b["sl"][0].start, b["sl"][1].start, b["sl"][0].stop, b["sl"][1].stop]; del b["sl"]
        np.savez_compressed(run / f"detect_n{n}.npz", lab=lab.astype(np.int32), klab=klab.astype(np.int32))
        (run / f"detect_n{n}.json").write_text(json.dumps({"blobs": blobs, "marks": marks}, indent=1))
        (run / f"items_n{n}.json").write_text(json.dumps([{"id": it["id"], "kind": it["kind"], "status": it["_status"]} for it in t["items"]], indent=0))
        results[n] = {"stains_recognized": f"{stains_known}/{len(t['stains'])}", "marks_not_on_stains": marks_off_stain,
                      "baseline_nights": prev, "per_kind": per, "false_alarms": false_alarms,
                      "stains_flagged": stain_flags, "gone_flagged_as_present": gone_as_present, "rug_blobs": rug_flag,
                      "compared_area_m2": round(region.sum() * RES * RES, 1)}
        # overlay
        img = Image.fromarray((rgb_n * 0.55).astype(np.uint8)); ov = np.array(img)
        ov[pres] = (40, 200, 180); ov[np.isin(lab, [b["id"] for b in blobs if b["kind"] == "gone"])] = (150, 150, 255)
        ov[np.isin(lab, [b["id"] for b in blobs if b["kind"] == "floor_change"])] = (230, 170, 40)
        ov[klab > 0] = (60, 110, 255)
        img = Image.fromarray(ov); dr = ImageDraw.Draw(img)
        for it in t["items"]:
            r0, c0 = w2rc(it["x"], it["y"]); col = {"found": (40, 220, 120), "missed": (240, 60, 60), "unobserved": (130, 130, 130)}[it["_status"]]
            dr.ellipse([c0 - 22, r0 - 22, c0 + 22, r0 + 22], outline=col, width=4)
        img.save(run / f"detect_n{n}.png")
        tot = {k: sum(v[k] for v in per.values()) for k in ("found", "missed", "unobserved")}
        print(f"night {n} (baseline {prev}): found {tot['found']}/{tot['found']+tot['missed']} observed items "
              f"({tot['unobserved']} unobserved); by kind {per}; false alarms {false_alarms}; "
              f"stains recognized as known {stains_known}/{len(t['stains'])} (other marks {marks_off_stain}); stains flagged as new {stain_flags}; gone-as-present {gone_as_present}; rug {rug_flag}; compared {results[n]['compared_area_m2']} m2")
    (run / "detect_results.json").write_text(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
