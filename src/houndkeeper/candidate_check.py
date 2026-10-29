"""The light check behind the floor comparison, on the robot ([Zaf]; the cascade of [bFx], [3hK], launch bar [tTD]).

Each floor-comparison candidate (a confirmed or unsure spot, never a floor-sized change) is looked at again on crops of
the head camera's frames that saw its floor point, the VIEWS frames nearest in time to the candidate's first decision
(0.6-2.0 m off, at least SEP_S apart, inside the image), each crop about SPAN_M of floor around the point, raised a
little and resized to CROP_PX; YOLO-E 11l prompt-free scores each crop (a detection counts when its box, grown by
CENTRE_GROW of the crop, holds the point; labels naming the house dropped), the candidate's score is its best crop's,
and it is kept at THRESHOLD or above. A candidate no frame saw is kept: the robot cannot check it, so it stops for it.
The crops are the bench's (sim/house/baselines/recheck.py, cascade.py crops --by-time), so a night replayed here keeps
the candidates the bench kept (tests/test_candidate_check.py holds the two to each other).

Online: frames arrive in time order (frame), decisions as the confirmer makes them (candidate). The views of a
candidate are chosen greedily in order of |frame time - decision time|; a frame after the decision is always the
farthest so far, so once VIEWS views are chosen and the newest frame is later than the decision by more than the
farthest chosen view, nothing can change the choice: the candidate is settled and scored (settle). Each crop is cut when
its frame is chosen (frames before the decision from `images`, the robot's own record of its frames, or the recent
frames kept here), so no frame is kept longer than KEEP_S. Nothing here reads the sim's truth.

The check is off by default: confirm_stream.py --check, and the live tidy's HK_CANDIDATE_CHECK=1."""
import math
import os

import numpy as np

VIEWS = 3                # crops a candidate gets
RANGE_M = (0.6, 2.0)     # views from this near to this far along the floor
SEP_S = 1.0              # the views at least this far apart in time
MARGIN_PX = 16           # the point at least this far inside the image
SPAN_M = 0.4             # a crop covers about this much floor across at the point's range
CROP_MIN, CROP_MAX = 96, 280   # crop side (px of the full frame) at its smallest and largest
RAISE = 0.15             # the crop's centre this share of its side above the point (a toy stands up from its point)
EVERY = 2                # the frames looked at: every 2nd of the head camera's stream
GRID_M = 0.05            # decisions within this grid cell are one candidate (its first decision's time and point)
CROP_PX = 384            # crops are resized to this square before the recognizer sees them
CENTRE_GROW = 0.10       # a detection counts if its box, grown by this share of the crop's side, holds the point
FLOOR_SCORE = 0.05       # the lowest detection score the recognizer reports
MODEL = "yoloe-11l-seg-pf.pt"   # YOLO-E 11l prompt-free, DimOS's data (models_yoloe)
THRESHOLD = 0.2436       # the lead's threshold, chosen on the six tuning nights by recall first ([Btx]; cascade round,
                         # 12 Oct 2026, detection-baselines [80w]: 45/48 at 7.8 a night there)
KEEP_S = 60.0            # live: frames kept this long for candidates decided after them (the bench's views lay up to
                         # 58 s before the decision on the fresh nights)
WAIT_S = 60.0            # live: a candidate is scored with the views it has this long after its decision (on the fresh
                         # nights 90% settled within 31 s; a few are seen again only minutes later, or never 3 times)
CPU_THREADS = int(os.environ.get("HK_CHECK_CPU_THREADS", "2"))   # on the CPU, the check's threads: the confirmer shares
                         # the CPU (ultralytics takes up to 8 at its first prediction otherwise)
KINDS = ("confirmed", "unsure")
# labels a prompt-free detector gives the house itself, dropped (the bench's prompts.STRUCTURE, [En1]; held equal by test)
STRUCTURE = ["floor", "flooring", "hardwood", "parquet", "tile", "carpet", "rug", "mat", "doormat", "wall", "ceiling",
             "door", "doorway", "window", "curtain", "blind", "baseboard", "skirting board", "molding", "stairs",
             "room", "living room", "bedroom", "kitchen", "hallway", "interior", "house", "home", "furniture", "table",
             "desk", "chair", "couch", "sofa", "bed", "cabinet", "cupboard", "drawer", "shelf", "bookshelf", "wardrobe",
             "dresser", "counter", "lamp", "light", "radiator", "fireplace", "television", "plant", "houseplant",
             "shadow", "corner"]


def is_candidate(rec):
    """a decision the check looks at: confirmed or unsure, never a floor-sized change"""
    return rec.get("decision") in KINDS and rec.get("kind") != "floor_change"


def key(x, y):
    """a candidate's key: its floor point on GRID_M"""
    return f"{round(x / GRID_M)}:{round(y / GRID_M)}"


def project(xy, T_wc, K, D=None, model="equidistant", floor_z=0.0):
    """a floor point (x, y) to its pixel (u, v) in a frame taken from T_wc (world <- optical) through the lens (K 3x3,
    the fisheye's D), or None behind the camera"""
    P = np.linalg.inv(np.asarray(T_wc, float)) @ np.array([xy[0], xy[1], floor_z, 1.0])
    if P[2] <= 0.05: return None
    K = np.asarray(K, float).reshape(3, 3)
    if D is not None and len(D) == 4 and model == "equidistant" and np.abs(np.asarray(D, float)).max() > 0:
        import cv2
        uv = cv2.fisheye.distortPoints(np.array([[[P[0] / P[2], P[1] / P[2]]]], np.float64), K, np.asarray(D, float)).reshape(2)
        return float(uv[0]), float(uv[1])
    return float(K[0, 0] * P[0] / P[2] + K[0, 2]), float(K[1, 1] * P[1] / P[2] + K[1, 2])


def crop_box(u, v, rng, fx, w, h):
    """the crop around pixel (u, v) of a point rng m off: (x0, y0, side), inside a w x h image"""
    s = int(np.clip(fx * SPAN_M / max(rng, 0.1), CROP_MIN, min(CROP_MAX, h)))
    x0 = int(np.clip(round(u - s / 2), 0, w - s)); y0 = int(np.clip(round(v - s / 2 - RAISE * s), 0, h - s))
    return x0, y0, s


def cut(img, u, v, rng, fx):
    """the crop of a frame (RGB) for a point at pixel (u, v), rng m off: (CROP_PX square RGB, the point's pixel in it,
    the box in the frame [x0, y0, x1, y1])"""
    import cv2
    h, w = img.shape[:2]; x0, y0, s = crop_box(u, v, rng, fx, w, h)
    c = cv2.resize(np.ascontiguousarray(img[y0:y0 + s, x0:x0 + s]), (CROP_PX, CROP_PX), interpolation=cv2.INTER_LINEAR)
    sc = CROP_PX / s
    return c, [round((u - x0) * sc, 1), round((v - y0) * sc, 1)], [x0, y0, x0 + s, y0 + s]


def sees(xy, T, K, D, model, wh):
    """(range m, (u, v)) when a frame from T sees floor point xy within RANGE_M and inside the image by MARGIN_PX"""
    r = math.hypot(xy[0] - T[0, 3], xy[1] - T[1, 3])
    if not RANGE_M[0] <= r <= RANGE_M[1]: return None
    uv = project(xy, T, K, D, model)
    if uv is None or not (MARGIN_PX <= uv[0] < wh[0] - MARGIN_PX and MARGIN_PX <= uv[1] < wh[1] - MARGIN_PX): return None
    return r, uv


def choose(seen):
    """the views of a candidate from the frames that see it, seen: [(|dt|, range, frame index, uv, ts)]: greedily in
    that order, each at least SEP_S from those chosen, up to VIEWS"""
    chosen = []
    for s in sorted(seen, key=lambda s: s[:3]):
        if all(abs(s[4] - c[4]) >= SEP_S for c in chosen): chosen.append(s)
        if len(chosen) == VIEWS: break
    return chosen


def structural(label):
    """whether a prompt-free detector's label names the house rather than a thing on its floor"""
    words = set(str(label).lower().replace("-", " ").replace("_", " ").split())
    return any(set(s.split()) <= words for s in STRUCTURE)


def holds(box, uv, side, grow=CENTRE_GROW):
    """whether a box (x0, y0, x1, y1), grown by grow * side each way, holds pixel uv"""
    g = grow * side; x0, y0, x1, y1 = box
    return x0 - g <= uv[0] <= x1 + g and y0 - g <= uv[1] <= y1 + g


def device():
    """where the recognizer runs: HK_CHECK_DEVICE when set (cuda or cpu), else the GPU when there is one"""
    d = os.environ.get("HK_CHECK_DEVICE")
    if d: return d
    try:
        import torch
        return "cuda" if torch.cuda.is_available() else "cpu"
    except Exception:
        return "cpu"


class Yoloe:
    """YOLO-E 11l prompt-free through DimOS's wrapper, on the GPU when there is one; on a GPU failure (no memory left
    beside the sim, a driver fault) it falls back to the CPU, at load or mid-night, and says so"""
    name = "yoloe_pf_l"

    def __init__(self, dev=None, make=None, say=print):
        self.device = dev or device(); self._make = make or self._wrapper; self.say = say; self.det = None

    @staticmethod
    def _wrapper(dev):
        from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
        return Yoloe2DDetector(prompt_mode=YoloePromptMode.LRPC, model_name=MODEL, conf=FLOOR_SCORE, device=dev)

    def load(self):
        try:
            self.det = self._make(self.device)
        except Exception as e:
            if self.device == "cpu": raise
            self.say(f"check: the recognizer could not load on {self.device} ({type(e).__name__}: {e}); on the CPU")
            self.device = "cpu"; self.det = self._make("cpu")
        if self.device == "cpu": self._cpu_threads()
        return self

    def _cpu_threads(self):
        """the CPU's threads held to CPU_THREADS, after one prediction (ultralytics sets its own at its first)"""
        try:
            import torch
            self.detect(np.zeros((CROP_PX, CROP_PX, 3), np.uint8)); torch.set_num_threads(CPU_THREADS)
        except Exception:
            pass

    def detect(self, rgb):
        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
        r = self.det.predict_image(Image.from_numpy(np.ascontiguousarray(rgb), format=ImageFormat.RGB, ts=0.0))
        return [{"label": d.name, "box": [round(float(v), 1) for v in d.bbox], "score": round(float(d.confidence), 4)}
                for d in r.detections]

    def score(self, crop, uv):
        """the strongest detection whose grown box holds uv (house labels dropped); 0 if none"""
        if self.det is None: self.load()
        try:
            ds = self.detect(crop)
        except Exception as e:
            if self.device == "cpu": raise
            self.say(f"check: the recognizer failed on {self.device} ({type(e).__name__}: {e}); on the CPU from now")
            self.device = "cpu"; self.det = self._make("cpu"); self._cpu_threads(); ds = self.detect(crop)
        return max((d["score"] for d in ds if not structural(d["label"]) and holds(d["box"], uv, crop.shape[1])), default=0.0)


class CandidateCheck:
    """the check over one night, online. K (3x3), D, model: the head camera's lens (the raw frame's); wh: its size.
    images(ts) -> the frame taken at ts (RGB) or None: the robot's own record of its frames; without it the frames are
    kept here. keep_s: frames older than this before the newest are not used (None: every frame, as the bench). recognizer: anything with score(crop, uv) (Yoloe by default, loaded on first use).
    max_wait_s: a candidate not settled this long after its decision is scored with the views it has (None: wait for
    settle or finish)."""

    def __init__(self, K, D, model, wh, recognizer=None, threshold=THRESHOLD, images=None, keep_s=KEEP_S, max_wait_s=None):
        self.K = np.asarray(K, float).reshape(3, 3); self.D = None if D is None else [float(v) for v in D]
        self.model = model or ""; self.wh = (int(wh[0]), int(wh[1])); self.fx = float(self.K[0, 0])
        self.rec = recognizer if recognizer is not None else Yoloe()
        self.threshold = float(threshold); self.images = images; self.keep_s = keep_s; self.max_wait_s = max_wait_s
        self.poses = []          # [(ts, T world <- optical, stream index)] of every frame looked at
        self.ring = {}           # pose index -> image, the recent frames (when images is None)
        self.pending = {}        # key -> candidate state
        self.verdicts = {}       # key -> verdict, once settled
        self.now = -math.inf

    def _image(self, i):
        if i in self.ring: return self.ring[i]
        if self.images is None or (self.keep_s is not None and self.poses[i][0] < self.now - self.keep_s): return None
        return self.images(self.poses[i][0])

    def _crop(self, c, s, img):
        crop, uv, box = cut(img, s[3][0], s[3][1], s[1], self.fx)
        ts, T, idx = self.poses[s[2]]
        c["crops"][s[2]] = (crop, uv, {"ts": float(ts), "index": idx, "range_m": round(float(s[1]), 2), "uv": uv, "box_full": box})

    def frame(self, ts, T, image, index=None):
        """a head-camera frame (its stamp, world <- optical then, the raw RGB frame): chosen as a view by any pending
        candidate it sees; returns the verdicts that settle with it"""
        T = np.asarray(T, float); i = len(self.poses); self.poses.append((float(ts), T, index)); self.now = float(ts)
        if self.images is None:
            self.ring[i] = image
            for j in [j for j in self.ring if self.poses[j][0] < self.now - self.keep_s]: del self.ring[j]
        for c in self.pending.values():
            v = sees(c["xy"], T, self.K, self.D, self.model, self.wh)
            if v is None: continue
            c["seen"].append((abs(ts - c["t0"]), v[0], i, v[1], float(ts))); self._fill(c, i, image)
        return self.settle()

    def _fill(self, c, i_now=None, image=None):
        """a crop for each chosen view that has none yet (a newer view can unblock an older one within SEP_S)"""
        for s in choose(c["seen"]):
            if s[2] in c["crops"]: continue
            img = image if s[2] == i_now and image is not None else self._image(s[2])
            if img is None: c["seen"].remove(s); return self._fill(c, i_now, image)   # gone from memory: as if unseen
            self._crop(c, s, img)

    def candidate(self, rec, fields=("x", "y")):
        """a decision (its point, fields, in the frame the frames' poses are in; ts, the stamp of the frame it was made
        on): a new candidate, its views among the frames so far chosen now; returns its verdict if settled, else None"""
        if not is_candidate(rec): return None
        x, y = float(rec[fields[0]]), float(rec[fields[1]]); k = key(x, y)
        if k in self.verdicts: return self.verdicts[k]
        if k in self.pending: return None
        c = {"key": k, "xy": (x, y), "t0": float(rec["ts"]), "seen": [], "crops": {}}
        for i, (ts, T, _) in enumerate(self.poses):
            v = sees(c["xy"], T, self.K, self.D, self.model, self.wh)
            if v is not None and self._image(i) is not None: c["seen"].append((abs(ts - c["t0"]), v[0], i, v[1], ts))
        self._fill(c); self.pending[k] = c
        return self.settle().get(k)

    def _score(self, c, why):
        chosen = choose(c["seen"]); views = [c["crops"][s[2]] for s in chosen]
        scores = [round(float(self.rec.score(cr, uv)), 4) for cr, uv, _ in views]
        best = max(scores) if scores else None
        v = {"score": best, "kept": best is None or best >= self.threshold, "threshold": self.threshold,
             "views": [dict(m, score=sc) for (_, _, m), sc in zip(views, scores)], "decided_ts": c["t0"],
             "settled_ts": self.now, "settled": why, "device": getattr(self.rec, "device", None)}
        self.verdicts[c["key"]] = v
        return v

    def settle(self, now=None, final=False):
        """the candidates whose views can no longer change (or final: every pending one): {key: verdict}"""
        if now is not None: self.now = max(self.now, float(now))
        out = {}
        for k, c in list(self.pending.items()):
            chosen = choose(c["seen"]); why = None
            if len(chosen) == VIEWS and self.now >= c["t0"] + max(s[0] for s in chosen): why = "settled"
            elif final: why = "night over"
            elif self.max_wait_s is not None and self.now >= c["t0"] + self.max_wait_s: why = "waited"
            if why: out[k] = self._score(c, why); del self.pending[k]
        return out

    def finish(self):
        return self.settle(final=True)


def annotate(recs, verdicts, fields=("x", "y")):
    """each candidate decision marked with its candidate's verdict (check: score, kept, the views' frame stamps)"""
    for r in recs:
        if is_candidate(r):
            v = verdicts.get(key(r[fields[0]], r[fields[1]]))
            if v is not None: r["check"] = v
    return recs
