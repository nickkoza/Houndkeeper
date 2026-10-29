#!/usr/bin/env python3
"""Live floor overlay for the DimOS Viewer (source "dimos-floor"): tonight's floor picture as a textured
floor in 3D and a top-down panel, the robot and its trail, points of interest from the frozen detector's
own functions vs previous nights' floor pictures, and the pipeline state. Read-only on the robot.
Usage: overlay.py --night N --baseline runs/nights_s7_r2 [--grpc rerun+http://127.0.0.1:9877/proxy]"""
import argparse, json, threading, time
from pathlib import Path
import numpy as np
import rerun as rr, rerun.blueprint as rrb
from scipy import ndimage as ndi
from dimos.core.transport_factory import transport_topic
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
import detect_changes2 as dc   # the locked detector's own test (its feats, tolerant_diff and thresholds are v1's)

HX, HY, RES, HRES = 8.4, 4.9, 0.01, 0.02
W, H = int(2 * HX / RES), int(2 * HY / RES)
SIM = Path(__file__).resolve().parent
PHASES = ["loading house map", "mapping", "sweeping", "processing", "done"]
FUTURE = ["inspecting (needs the arm)", "picking up (needs the arm)"]


def arr(img):
    try:                                                          # live Image messages: decode to RGB pixels
        rgb = img.to_rgb() if hasattr(img, "to_rgb") else img
        a = rgb.as_numpy() if hasattr(rgb, "as_numpy") else np.asarray(rgb)
        if a.ndim == 3: return a
    except Exception: pass
    for c in (getattr(img, "data", None), img):
        try:
            a = np.asarray(c)
            if a.ndim == 3: return a
        except Exception: pass
    return img.to_numpy()


class Live:
    def __init__(self):
        self.lock = threading.Lock(); self.img = None; self.K = None; self.T_bc = self.T_co = None
        self.poses = []; self.hmap = np.zeros((int(2 * HY / HRES), int(2 * HX / HRES)), np.float32); self.n_img = 0

    def on_img(self, m, _t):
        with self.lock: self.img = (time.time(), m); self.n_img += 1

    def on_info(self, m, _t):
        if self.K is None: self.K = (m.K[0], m.K[4], m.K[2], m.K[5], m.width, m.height)

    @staticmethod
    def _tf(t):
        """Works for recorded Transform objects and live TransformStamped messages."""
        frame = getattr(t, "frame_id", None) or getattr(getattr(t, "header", None), "frame_id", None)
        if hasattr(t, "to_matrix"): M = t.to_matrix()
        else:
            tr = t.transform; q = tr.rotation; v = tr.translation
            from scipy.spatial.transform import Rotation
            M = np.eye(4); M[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(); M[:3, 3] = [v.x, v.y, v.z]
        return (frame, t.child_frame_id), M

    def on_tf(self, m, _t):
        for t in getattr(m, "transforms", [m]):
            try: p, M = self._tf(t)
            except Exception: continue
            t = type("T", (), {"to_matrix": lambda self, M=M: M})()
            if p == ("world", "base_link"):
                with self.lock: self.poses.append((time.time(), t.to_matrix())); self.poses = self.poses[-400:]
            elif p == ("base_link", "camera_link") and self.T_bc is None: self.T_bc = t.to_matrix()
            elif p == ("camera_link", "camera_optical") and self.T_co is None: self.T_co = t.to_matrix()

    def on_map(self, m, _t):
        p = m.as_numpy(); P = np.asarray(p[0] if isinstance(p, tuple) else p); P = P[P[:, 2] > 0.06]
        hi = ((HY - P[:, 1]) / HRES).astype(int); hj = ((P[:, 0] + HX) / HRES).astype(int)
        ok = (hi >= 0) & (hi < self.hmap.shape[0]) & (hj >= 0) & (hj < self.hmap.shape[1])
        h = np.zeros_like(self.hmap); np.maximum.at(h, (hi[ok], hj[ok]), P[ok, 2].astype(np.float32))
        c = np.zeros(h.shape, np.int32); np.add.at(c, (hi[ok], hj[ok]), 1); h[c < 3] = 0
        self.hmap = ndi.maximum_filter(h, size=3)


def project(L, rgb, best, img, Twb, rmax=2.5):
    fx, fy, cx, cy, iw, ih = L.K
    T = Twb @ L.T_bc @ L.T_co; R, C = T[:3, :3], T[:3, 3]
    gx = -HX + (np.arange(W) + .5) * RES; gy = HY - (np.arange(H) + .5) * RES
    c0, c1 = np.searchsorted(gx, C[0] - rmax), np.searchsorted(gx, C[0] + rmax)
    r0, r1 = np.searchsorted(-gy, -(C[1] + rmax)), np.searchsorted(-gy, -(C[1] - rmax))
    if c1 <= c0 or r1 <= r0: return
    X, Y = np.meshgrid(gx[c0:c1], gy[r0:r1]); P = np.stack([X - C[0], Y - C[1], np.full_like(X, -C[2])], -1)
    pc = P @ R; z = pc[..., 2]; ok = z > 0.1
    u = fx * pc[..., 0] / np.where(ok, z, 1) + cx; v = fy * pc[..., 1] / np.where(ok, z, 1) + cy; d = np.hypot(P[..., 0], P[..., 1])
    ok &= (u >= 0) & (u < iw - 1) & (v >= 0) & (v < ih - 1) & (d < rmax) & (d < best[r0:r1, c0:c1])
    if not ok.any(): return
    rr_, cc = np.nonzero(ok); tx, ty, dd = X[rr_, cc], Y[rr_, cc], d[rr_, cc]; vis = np.ones(len(rr_), bool)
    for f in np.linspace(0.03, 0.97, 16):
        sx, sy = tx + (C[0] - tx) * f, ty + (C[1] - ty) * f
        hi = ((HY - sy) / HRES).astype(int); hj = ((sx + HX) / HRES).astype(int)
        inb = (hi >= 0) & (hi < L.hmap.shape[0]) & (hj >= 0) & (hj < L.hmap.shape[1])
        hg = np.zeros(len(rr_), np.float32); hg[inb] = L.hmap[hi[inb], hj[inb]]
        vis &= ~((hg > C[2] * f + 0.01) & ~(dd * f < 0.04))
    rr_, cc = rr_[vis], cc[vis]
    rgb[r0 + rr_, c0 + cc] = img[np.rint(v[rr_, cc]).astype(int), np.rint(u[rr_, cc]).astype(int), :3]; best[r0 + rr_, c0 + cc] = d[rr_, cc]


def flags(rgb, best, base, Vb):
    """Points of interest with the frozen detector's own functions (camera side only, live)."""
    Vn = np.isfinite(best); region = Vn & Vb
    if region.sum() < 1000: return [], []
    cn, rn = dc.feats(rgb, Vn); cb, rb = dc.feats(base, Vb); dC, br = dc.tolerant_diff(cn, rn, cb, rb)
    core = ndi.binary_erosion(region, iterations=6); cand = ndi.binary_closing(core & ((dC > dc.TC) | (br > dc.TB)), iterations=2) & core
    lab, _ = ndi.label(cand); new = []
    for i, sl in enumerate(ndi.find_objects(lab), 1):
        m = lab[sl] == i
        if dc.MIN_PX <= m.sum() <= dc.BIG_PX:
            r, c = np.nonzero(m); new.append((-HX + (c.mean() + sl[1].start) * RES, HY - (r.mean() + sl[0].start) * RES, max(np.ptp(r), np.ptp(c)) * RES / 2 + .03))
    known = region & ~cand & (rn < 0.8) & (rb < 0.8); known = ndi.binary_opening(known, iterations=2)
    klab, _ = ndi.label(known); marks = []
    for i, sl in enumerate(ndi.find_objects(klab), 1):
        m = klab[sl] == i
        if 50 <= m.sum() <= dc.BIG_PX:
            r, c = np.nonzero(m); marks.append((-HX + (c.mean() + sl[1].start) * RES, HY - (r.mean() + sl[0].start) * RES, max(np.ptp(r), np.ptp(c)) * RES / 2 + .03))
    return new, marks


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--night", type=int, required=True); ap.add_argument("--baseline", required=True)
    ap.add_argument("--grpc", default="rerun+http://127.0.0.1:9877/proxy"); ap.add_argument("--seconds", type=float, default=1e9)
    ap.add_argument("--model-dir", default=str(SIM.parent / "media" / "floor")); run(ap.parse_args())


def _layout(inside):
    ours = rrb.Vertical(rrb.Horizontal(rrb.Spatial2DView(origin="/topdown", name="Live, tonight"), rrb.Spatial2DView(origin="/composite", name="Composite, house floor model")),
                        rrb.TextDocumentView(origin="/state", name="State"), row_shares=[3, 2])
    if inside:                                                     # DimOS's own panels on the left, ours on the right
        try:
            from dimos.visualization.rerun.bridge import _default_blueprint
            return rrb.Blueprint(rrb.Horizontal(_default_blueprint().root_container, ours, column_shares=[3, 2]))
        except Exception:
            pass
    return rrb.Blueprint(rrb.Horizontal(rrb.Spatial3DView(origin="/world", name="House: composite floor, live painted over"), ours, column_shares=[3, 2]))


def run(a, stop=None, inside=False):
    prev = [k for k in range(max(1, a.night - 3), a.night) if (Path(a.baseline) / f"mosaic_n{k}.npz").exists()]
    if prev:
        M = [np.load(Path(a.baseline) / f"mosaic_n{k}.npz") for k in prev]
        st = np.ma.masked_array(np.stack([m["rgb"] for m in M]).astype(np.float32), mask=~np.stack([np.isfinite(m["dist"].astype(np.float32)) for m in M])[..., None].repeat(3, -1))
        med = np.ma.median(st, axis=0); Vb = ~np.ma.getmaskarray(med)[..., 0]; base = med.filled(0).astype(np.uint8)
    else: base, Vb = None, None
    comp = None; mp = Path(a.model_dir) / f"floor_model_after_night{a.night - 1}.png"
    if mp.exists():                                             # composite house floor model: baseline for flags
        from PIL import Image as _I
        m_ = np.asarray(_I.open(mp).convert("RGB"))[:H, :W]; comp = np.zeros((H, W, 3), np.uint8); comp[: m_.shape[0], : m_.shape[1]] = m_
        base, Vb = comp, comp.sum(-1) > 0
    if inside:
        from dimos.visualization.rerun.init import rerun_init
        import socket
        rerun_init()                                               # same recording as DimOS's own views (shared default id)
        for _ in range(120):                                       # wait for the bridge's gRPC server, then connect to it
            with socket.socket() as so:
                if so.connect_ex(("127.0.0.1", 9877)) == 0: break
            time.sleep(1)
        rr.connect_grpc(url=a.grpc or "rerun+http://127.0.0.1:9877/proxy")
    else:
        rr.init("dimos-floor"); rr.connect_grpc(url=a.grpc)
    rr.send_blueprint(_layout(inside)); layout_resent = False
    if comp is not None: rr.log("composite/floor", rr.Image(comp[::2, ::2]), static=True)
    L = Live(); bus = Zenoh(); bus.start()
    for topic, cb in (("/color_image", L.on_img), ("/camera_info", L.on_info), ("/tf", L.on_tf), ("/global_map", L.on_map)):
        bus.subscribe(Topic(f"{transport_topic(topic)}/**"), cb)
    rgb = np.zeros((H, W, 3), np.uint8); best = np.full((H, W), np.inf, np.float32)
    trail, last_img_n, last_log, last_flag, new, marks, t0 = [], 0, 0, 0, [], [], time.time()
    uv = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float32)
    quad = np.array([[-HX, HY, .002], [HX, HY, .002], [HX, -HY, .002], [-HX, -HY, .002]], np.float32)
    while time.time() - t0 < a.seconds and not (stop is not None and stop.is_set()):
        time.sleep(0.05)
        if inside and not layout_resent and time.time() - t0 > 15: rr.send_blueprint(_layout(inside)); layout_resent = True   # after DimOS's own layout
        with L.lock: item, n_img, poses = L.img, L.n_img, list(L.poses)
        if item is None or L.K is None or L.T_bc is None or not poses: continue
        if n_img - last_img_n >= 4:                                     # every 4th frame, like the pipeline
            last_img_n = n_img; Twb = poses[-1][1]; project(L, rgb, best, arr(item[1]), Twb)
            trail.append([Twb[0, 3], Twb[1, 3], 0.05]); trail = trail[-3000:]
        now = time.time()
        if base is not None and now - last_flag > 15: new, marks = flags(rgb, best, base, Vb); last_flag = now
        if now - last_log > 2:
            last_log = now
            shown = np.where(np.isfinite(best)[..., None], rgb, np.array([222, 228, 231], np.uint8))[::2, ::2]
            layered = np.where(np.isfinite(best)[..., None], rgb, comp if comp is not None else np.array([222, 228, 231], np.uint8))[::2, ::2]
            rr.log("world/floor", rr.Mesh3D(vertex_positions=quad, triangle_indices=[[0, 1, 2], [0, 2, 3]], vertex_texcoords=uv, albedo_texture=layered))
            if trail: rr.log("world/robot/trail", rr.LineStrips3D([trail], colors=[[28, 124, 140]])); rr.log("world/robot/now", rr.Points3D([trail[-1]], radii=0.12, colors=[[22, 34, 46]]))
            for name, pts, col, lbl in (("flagged", new, [28, 170, 150], "flagged"), ("known", marks, [40, 102, 255], "known stain")):
                if pts: rr.log(f"world/poi/{name}", rr.Boxes3D(centers=[[x, y, .05] for x, y, _ in pts], half_sizes=[[r, r, .05] for _, _, r in pts], colors=[col], labels=[lbl] * len(pts)))
                else: rr.log(f"world/poi/{name}", rr.Clear(recursive=True))
            rr.log("topdown/floor", rr.Image(shown))
            if new: rr.log("topdown/flagged", rr.Boxes2D(centers=[[(x + HX) / RES / 2, (HY - y) / RES / 2] for x, y, _ in new], half_sizes=[[r / RES / 2] * 2 for *_, r in new], colors=[[28, 170, 150]]))
            try: s = json.loads((SIM / "runs" / "state.json").read_text())
            except Exception: s = {"phase": "starting", "night": a.night, "wp": 0, "of": 0}
            lines = [f"## Night {s.get('night', a.night)}"]
            for p in PHASES:
                mark = "**▶ " + p + "**" if p == s["phase"] else ("✓ " + p if PHASES.index(p) < PHASES.index(s["phase"]) else "· " + p) if s["phase"] in PHASES else "· " + p
                if p == "sweeping" and s["phase"] == "sweeping": mark += f" — waypoint {s['wp']} of {s['of']}"
                lines.append(f"- {mark}")
            lines += [f"- ~~{p}~~" for p in FUTURE]
            lines += ["", f"**Baseline:** {'composite floor model (nights 1–' + str(a.night - 1) + ')' if comp is not None else ('last nights' if base is not None else 'none yet')}  ",
                      f"**Floor seen tonight:** {np.isfinite(best).mean():.0%}  ", f"**Flagged (camera):** {len(new)}  ", f"**Known marks:** {len(marks)}  ",
                      "_Flags include false alarms; the arm's look-closer step would check them._" if base is not None else "_First night: learning the floor, no flags yet._"]
            rr.log("state", rr.TextDocument("\n".join(lines), media_type=rr.MediaType.MARKDOWN))
    bus.stop()


if __name__ == "__main__":
    main()
