#!/usr/bin/env python3
"""Evidence-based persistent house map with smart clearing.

Per 4 cm voxel, per night: OCCUPIED if it's in DimOS's own cleaned map of that night (global_map);
EMPTY if enough raw lidar rays passed through it to something beyond and DimOS didn't map it. Rules:
  add:   in the map once occupied on >= 2 nights
  clear: removed once seen EMPTY on enough separate nights since it was last occupied, where
         enough = 2 + (nights it was ever occupied) // 2. Tenure: walls effectively never clear, a sofa
         seen for weeks needs many empty nights, a chair seen for two nights clears after three.
Biased to keep: a ghost for a few nights is cheaper than losing real structure.
Writes the seed for the next night plus a per-night evidence log. Rays are subsampled for speed.
Usage: house_map.py <recordings.txt> --upto N --out runs/nights_s7/house_map"""
import argparse, json
from pathlib import Path
import numpy as np
from dimos.memory.store.sqlite import SqliteStore

VOX, STEP_FRAMES, STEP_PTS, MIN_THRU = 0.04, 8, 6, 2


def keys(P): return np.floor(P / VOX).astype(np.int64)


def pack(K): return (K[:, 0] + 1000) * 10 ** 8 + (K[:, 1] + 1000) * 10 ** 4 + (K[:, 2] + 1000)   # each field < 10^4


def unpack(Q): Q = np.asarray(Q, np.int64); return np.stack([Q // 10 ** 8 - 1000, (Q // 10 ** 4) % 10 ** 4 - 1000, Q % 10 ** 4 - 1000], 1)


def night_evidence(db):
    st = SqliteStore(path=db); st.start()
    tf = [(getattr(t, "ts", None) or o.ts, t.to_matrix()) for o in st.stream("tf") for t in getattr(o.data, "transforms", [o.data]) if (t.frame_id, t.child_frame_id) == ("world", "base_link")]
    ts = np.array([t for t, _ in tf]); Ms = [m for _, m in tf]; order = np.argsort(ts); ts = ts[order]; Ms = [Ms[i] for i in order]
    occ, thru = {}, {}
    for k, o in enumerate(st.stream("lidar")):
        if k % STEP_FRAMES: continue
        i = min(np.searchsorted(ts, o.ts), len(ts) - 1); O = Ms[i][:3, 3].copy(); O[2] = 0.35     # sensor ~ head height
        p = o.data.as_numpy(); P = np.asarray(p[0] if isinstance(p, tuple) else p, np.float64)[::STEP_PTS]
        P = P[(P[:, 2] > 0.10) & (np.linalg.norm(P[:, :2] - O[:2], axis=1) < 6.0)]
        if not len(P): continue
        for q in np.unique(pack(keys(P))): occ[q] = occ.get(q, 0) + 1
        d = P - O; L = np.linalg.norm(d, axis=1); n = np.maximum((L / VOX).astype(int) - 2, 0)   # stop short of the hit
        steps = np.arange(n.max() + 1) if n.max() > 0 else np.array([0])
        f = (steps[None, :] * VOX) / np.maximum(L[:, None], 1e-6); valid = steps[None, :] < n[:, None]
        S = O[None, None, :] + d[:, None, :] * f[..., None]; S = S[valid]; S = S[S[:, 2] > 0.10]
        if len(S):
            u, c = np.unique(pack(keys(S)), return_counts=True)
            for q, cc in zip(u, c): thru[q] = thru.get(q, 0) + int(cc)
    st.stop()
    occ_set = {q for q, c in occ.items() if c >= 1}
    empty_set = {q for q, c in thru.items() if c >= MIN_THRU and q not in occ_set}
    return occ_set, empty_set


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("recordings"); ap.add_argument("--upto", type=int, default=99)
    ap.add_argument("--out", default="runs/nights_s7/house_map"); a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    occ_nights, empty_since, in_map, log = {}, {}, set(), []
    for line in open(a.recordings):
        parts = line.split()
        if len(parts) != 2 or int(parts[0]) > a.upto: continue
        n = int(parts[0]); cache = out / f"evidence_n{n}.npz"
        if cache.exists(): z = np.load(cache); empty_set = set(z["empty"].tolist())
        else:
            _, empty_set = night_evidence(parts[1]); np.savez_compressed(cache, empty=np.array(sorted(empty_set), np.int64))
        st = SqliteStore(path=parts[1]); st.start(); gp = list(st.stream("global_map"))[-1].data.as_numpy(); st.stop()
        G = np.asarray(gp[0] if isinstance(gp, tuple) else gp, np.float64); G = G[G[:, 2] > 0.10]
        occ_set = set(np.unique(pack(keys(G))).tolist())
        if in_map:                                            # consistency gate: a mislocalized night is quarantined
            Hk = unpack(np.array(sorted(in_map), np.int64)); from scipy.spatial import cKDTree
            agree = float((cKDTree(Hk).query(unpack(np.array(sorted(occ_set), np.int64)), distance_upper_bound=2.1)[0] <= 2).mean())
            if agree < 0.60:
                print(f"night {n}: QUARANTINED ({agree:.0%} agreement); kept on disk, not merged", flush=True)
                log.append({"night": n, "quarantined": True, "agreement": agree}); continue
        # a cell next to anything seen occupied tonight can't count as empty (grazing rays along walls)
        O = unpack(np.array(sorted(occ_set), np.int64)); off = np.array([[i, j, k] for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)])
        near = set(pack((O[:, None, :] + off[None]).reshape(-1, 3)).tolist()); empty_set -= near
        for q in occ_set: occ_nights[q] = occ_nights.get(q, 0) + 1; empty_since[q] = 0
        for q in empty_set: empty_since[q] = empty_since.get(q, 0) + 1
        added = {q for q, c in occ_nights.items() if c >= 2 and q not in in_map}
        cleared = {q for q in in_map if empty_since.get(q, 0) >= 2 + occ_nights.get(q, 0) // 2}
        in_map = (in_map | added) - cleared
        np.save(out / f"cleared_n{n}.npy", np.array(sorted(cleared), np.int64)); np.save(out / f"map_n{n}.npy", np.array(sorted(in_map), np.int64))
        log.append({"night": n, "occupied": len(occ_set), "seen_empty": len(empty_set), "added": len(added), "cleared": len(cleared), "map": len(in_map)})
        print(f"night {n}: occupied {len(occ_set)}, seen empty {len(empty_set)}; +{len(added)} added, -{len(cleared)} cleared -> house map {len(in_map)} voxels", flush=True)
    K = np.array(sorted(in_map), np.int64)
    xyz = unpack(K).astype(np.float32) * VOX + VOX / 2 if len(K) else np.zeros((0, 3), np.float32)
    np.save(out / f"house_map_upto{a.upto}.npy", xyz); (out / "log.json").write_text(json.dumps(log, indent=1))


if __name__ == "__main__":
    main()
