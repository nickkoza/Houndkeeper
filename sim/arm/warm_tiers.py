#!/usr/bin/env python3
"""Fill the grasp-tier cache (items.TIER_CACHE, [QyZ]) for every one-body item in the library, in parallel, and clear the
entries no current key names (an edit to the measuring code, a rescanned toy, other jaws). Each tier is measured
afresh, as without the cache, and kept; a run after it reads them back.
Usage: warm_tiers.py [--jobs N] [--rebuild]   (--rebuild: measure every item again, even one already kept)"""
import argparse, os, sys
from concurrent.futures import ProcessPoolExecutor
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import items


def measure(name):
    """the item's drop-test tier, measured (not read back) and kept"""
    os.environ["HK_TIER_CACHE"] = "0"; t = items.grasp_tier(name)
    os.environ["HK_TIER_CACHE"] = "1"; items._tier_keep(name, t)
    return name, t


def names():
    """every item one rigid body would settle as: the library's scans and fixed shapes (not cloth or rope)"""
    return [n for n, e in items.ITEMS.items() if e[0] in ("gso", "box", "cylinder", "sphere")]


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--rebuild", action="store_true"); a = ap.parse_args()
    keys = {n: items.tier_key(n) for n in names()}
    todo = [n for n in keys if a.rebuild or items._tier_kept(n) is None]
    with ProcessPoolExecutor(a.jobs) as ex:
        for n, t in ex.map(measure, todo): print(f"{t:8s} {n}", flush=True)
    live = {f"{k}.json" for k in keys.values()}
    stale = [f for f in items.TIER_CACHE.glob("*.json") if f.name not in live] if items.TIER_CACHE.exists() else []
    for f in stale: f.unlink()
    print(f"{len(todo)} measured, {len(keys) - len(todo)} already kept, {len(stale)} stale cleared ({items.TIER_CACHE})")
