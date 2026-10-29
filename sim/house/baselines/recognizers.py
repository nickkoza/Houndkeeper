"""The local recognizers ([En1]), each through DimOS's own wrapper (principle [eM8]: no new dependency), local, one at
a time on the GPU. Each is used two ways on the same frames, placement and scorer as the first arms ([o0E]):
  detect(rgb)          as a detector on a whole head-camera frame: [{label, box, score}] (recog_arm.py);
  verify(crop, uv)     as the recognizer of [3hK] on a crop around a floor-comparison candidate: whether something is
                       there, {score, label} (a detection whose box, grown by CENTRE_GROW of the crop, holds the
                       candidate's pixel uv; for Moondream a yes/no question then a label). score 0: nothing there.
Prompts, phrases and variants are fixed here (4 Oct 2026) before any result; where a recognizer has variants, the one
used is chosen on night s70_t1 alone ([XEi]'s rule), recorded in the findings.

RECOGNIZERS: name -> (constructor, the exact model, the variants). Nothing here reads the truth."""
import time

import numpy as np

from baselines import prompts as pr

CENTRE_GROW = 0.10       # a detection counts for a crop if its box, grown by this share of the crop's side, holds uv
CROP_PX = 384            # crops are resized to this square before they are shown to a recognizer
FLOOR_SCORE = 0.05       # the lowest detection score kept (thresholds are chosen afterwards, on s70_t1 alone)

# Moondream 2: what its detector is asked for (one phrase a call; the frontier prompt's intent, "objects on the floor
# that don't belong", in the few words its detect interface takes), and its yes/no question on a crop
MOONDREAM_PHRASES = ["toy", "toy or small object on the floor", "object lying on the floor"]
MOONDREAM_YESNO = ("Is there a toy or another small loose object lying on the floor in the middle of this picture? "
                   "Furniture, rugs, floor patterns, stains and shadows do not count. Answer yes or no.")
MOONDREAM_LABEL = "What is the small object lying on the floor in the middle of this picture? Answer in a few words."
# Florence-2: its prompt-free object detection (<OD>, labels outside STRUCTURE kept) or open-vocabulary detection of
# one phrase (<OPEN_VOCABULARY_DETECTION>)
FLORENCE_VARIANTS = {"od": ("<OD>", None), "ovd_toy": ("<OPEN_VOCABULARY_DETECTION>", "toy")}


def structural(label):
    """whether a prompt-free detector's label names the house rather than a thing on its floor (prompts.STRUCTURE)"""
    words = set(str(label).lower().replace("-", " ").replace("_", " ").split())
    return any(set(s.split()) <= words for s in pr.STRUCTURE)


def _image(rgb, ts=0.0):
    from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
    return Image.from_numpy(np.ascontiguousarray(rgb), format=ImageFormat.RGB, ts=ts)


def holds(box, uv, side, grow=CENTRE_GROW):
    """whether a box (x0, y0, x1, y1), grown by grow * side each way, holds pixel uv"""
    g = grow * side; x0, y0, x1, y1 = box
    return x0 - g <= uv[0] <= x1 + g and y0 - g <= uv[1] <= y1 + g


class Recognizer:
    """a DimOS model with a detect (whole frame) and a verify (crop) side; model: its exact name and weights"""
    name = ""; model = ""; prompt_free = False

    def load(self): raise NotImplementedError

    def detect(self, rgb, ts=0.0): raise NotImplementedError

    def verify(self, crop, uv):
        """the strongest detection in the crop whose box (grown) holds uv: {score, label}; nothing: score 0"""
        best = {"score": 0.0, "label": ""}
        for d in self.detect(crop):
            if d["score"] > best["score"] and holds(d["box"], uv, crop.shape[1]): best = {"score": d["score"], "label": d["label"]}
        return best


class Owl(Recognizer):
    """OWLv2 through Owlv2Detector, the twelve prompts of [XEi] and any extra (the generic "small object" the misses'
    diagnosis supports, docs/research/detection-misses-2026-10-05.md); the base weights (the first arm's) or the large"""
    def __init__(self, model="google/owlv2-base-patch16-ensemble", extra=()):
        self.model = model; self.prompts = pr.PROMPTS + list(extra)
        self.name = ("owlv2_large" if "large" in model else "owlv2") + ("_so" if "small object" in extra else "")

    def load(self):
        from dimos.perception.detection.detectors.owlv2 import Owlv2Detector
        self.det = Owlv2Detector(model_name=self.model); self.det._model; self.det._processor

    def detect(self, rgb, ts=0.0):
        r = self.det.query_detections(_image(rgb, ts), self.prompts, threshold=FLOOR_SCORE)
        return [{"label": d.name, "box": [round(float(v), 1) for v in d.bbox], "score": round(float(d.confidence), 4)} for d in r.detections]


class Yoloe(Recognizer):
    """YOLO-E through Yoloe2DDetector: prompt-free (its built-in vocabulary, LRPC; labels naming the house dropped) or
    prompted with the twelve prompts of [XEi]; the wrapper's default small weights (11s) and the large (11l), both in
    DimOS's data (models_yoloe). The wrapper's own box-size cap (30% of the image) kept"""
    def __init__(self, prompt_free=True, size="s"):
        self.prompt_free = prompt_free; self.size = size
        self.name = ("yoloe_pf" if prompt_free else "yoloe_prompt") + ("_l" if size == "l" else "")

    def load(self):
        from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
        mode = YoloePromptMode.LRPC if self.prompt_free else YoloePromptMode.PROMPT
        self.model = f"yoloe-11{self.size}-seg" + ("-pf" if self.prompt_free else "") + ".pt"
        self.det = Yoloe2DDetector(prompt_mode=mode, model_name=self.model, conf=FLOOR_SCORE)
        if not self.prompt_free: self.det.set_prompts(text=pr.PROMPTS)

    def detect(self, rgb, ts=0.0):
        r = self.det.predict_image(_image(rgb, ts))
        out = [{"label": d.name, "box": [round(float(v), 1) for v in d.bbox], "score": round(float(d.confidence), 4)} for d in r.detections]
        return [d for d in out if not (self.prompt_free and structural(d["label"]))]


class Florence(Recognizer):
    """Florence-2 through Florence2Model (DimOS's captioner): its loaded model and processor asked for detection
    (FLORENCE_VARIANTS), as the wrapper's own caption() asks for a caption. No scores (every box 1.0)"""
    prompt_free = True

    def __init__(self, model="microsoft/Florence-2-base", variant="od"):
        self.model = model; self.variant = variant
        self.name = ("florence2_large" if "large" in model else "florence2") + f"_{variant}"

    def load(self):
        from dimos.models.vl.florence import Florence2Model
        self.m = Florence2Model(model_name=self.model); self.m._model; self.m._processor

    def detect(self, rgb, ts=0.0):
        import torch
        from PIL import Image as PILImage
        task, text = FLORENCE_VARIANTS[self.variant]
        pil = PILImage.fromarray(np.ascontiguousarray(rgb)); P = self.m._processor
        inputs = self.m._move_inputs_to_device(P(text=task + (text or ""), images=pil, return_tensors="pt"))
        with torch.inference_mode():
            ids = self.m._model.generate(**inputs, max_new_tokens=512, num_beams=3, do_sample=False)
        parsed = P.post_process_generation(P.batch_decode(ids, skip_special_tokens=False)[0], task=task, image_size=pil.size)[task]
        labels = parsed.get("labels") or parsed.get("bboxes_labels") or []
        out = [{"label": str(l) or (text or ""), "box": [round(float(v), 1) for v in b], "score": 1.0}
               for b, l in zip(parsed.get("bboxes", []), labels)]
        return [d for d in out if not structural(d["label"])]


class Moondream(Recognizer):
    """Moondream 2 through MoondreamVlModel: as a detector, its detect interface (query_detections) with one phrase
    (MOONDREAM_PHRASES, chosen on s70_t1); as the recognizer, a yes/no question about the crop's middle and, on a yes,
    a label. No scores (a detection or a yes is 1.0)"""
    prompt_free = True

    def __init__(self, phrase=MOONDREAM_PHRASES[0]):
        self.phrase = phrase; self.model = "vikhyatk/moondream2"; self.name = "moondream2"

    def load(self):
        from dimos.models.vl.moondream import MoondreamVlModel
        self.m = MoondreamVlModel(); self.m._model

    def detect(self, rgb, ts=0.0):
        r = self.m.query_detections(_image(rgb, ts), self.phrase, max_objects=10)
        return [{"label": self.phrase, "box": [round(float(v), 1) for v in d.bbox], "score": 1.0} for d in r.detections]

    def verify(self, crop, uv):
        img = _image(crop)
        ans = self.m.query(img, MOONDREAM_YESNO).strip().lower()
        if not ans.startswith("yes"): return {"score": 0.0, "label": "", "answer": ans[:60]}
        return {"score": 1.0, "label": self.m.query(img, MOONDREAM_LABEL).strip()[:60], "answer": ans[:60]}


RECOGNIZERS = {
    "owlv2": lambda **k: Owl(),
    "owlv2_large": lambda **k: Owl("google/owlv2-large-patch14-ensemble"),
    "owlv2_so": lambda **k: Owl(extra=["small object"]),
    "owlv2_large_so": lambda **k: Owl("google/owlv2-large-patch14-ensemble", ["small object"]),
    "yoloe_pf": lambda **k: Yoloe(True),
    "yoloe_prompt": lambda **k: Yoloe(False),
    "yoloe_pf_l": lambda **k: Yoloe(True, "l"),
    "yoloe_prompt_l": lambda **k: Yoloe(False, "l"),
    "florence2": lambda variant="od", **k: Florence("microsoft/Florence-2-base", variant),
    "florence2_large": lambda variant="od", **k: Florence("microsoft/Florence-2-large", variant),
    "moondream2": lambda variant=MOONDREAM_PHRASES[0], **k: Moondream(variant),
}
VARIANTS = {"florence2": list(FLORENCE_VARIANTS), "florence2_large": list(FLORENCE_VARIANTS), "moondream2": MOONDREAM_PHRASES}


def make(name, variant=None):
    """a recognizer by name (RECOGNIZERS), at a variant where it has them"""
    return RECOGNIZERS[name](**({"variant": variant} if variant is not None else {}))


def gpu_stats():
    """peak GPU memory since the last reset (MB allocated, reserved) and the device's name"""
    import torch
    if not torch.cuda.is_available(): return {"gpu": "none"}
    return {"gpu": torch.cuda.get_device_name(0), "gpu_peak_alloc_mb": round(torch.cuda.max_memory_allocated() / 2 ** 20),
            "gpu_peak_reserved_mb": round(torch.cuda.max_memory_reserved() / 2 ** 20)}


def timed(f, *a):
    """f(*a) and its wall time (s), the GPU synchronised either side"""
    import torch
    if torch.cuda.is_available(): torch.cuda.synchronize()
    t0 = time.time(); r = f(*a)
    if torch.cuda.is_available(): torch.cuda.synchronize()
    return r, time.time() - t0
