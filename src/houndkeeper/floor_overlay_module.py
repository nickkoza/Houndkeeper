"""DimOS module wrapping the floor overlay so it runs inside the DimOS run (same Rerun recording as DimOS).
Configured through environment variables set when launching `dimos run`:
HK_NIGHT, HK_BASELINE (dir of previous nights' floor pictures), HK_MODEL_DIR (composite floor models)."""
import os, sys, threading
from types import SimpleNamespace
from dimos.core.core import rpc
from dimos.core.module import Module

HK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # this repository
SIM = os.environ.get("HK_SIM", os.path.join(HK, "sim", "house"))   # overlay code lives with the night pipeline for now


class FloorOverlay(Module):
    def __init__(self, **kwargs):
        super().__init__(**kwargs); self._stop = threading.Event(); self._thread = None

    @rpc
    def start(self) -> None:
        super().start()
        if SIM not in sys.path: sys.path.insert(0, SIM)
        import overlay
        args = SimpleNamespace(night=int(os.environ.get("HK_NIGHT", "1")), baseline=os.environ.get("HK_BASELINE", os.path.join(SIM, "runs/nights_s7_r2")),
                               model_dir=os.environ.get("HK_MODEL_DIR", os.path.join(HK, "data", "media", "floor")), seconds=1e9, grpc=None)
        self._thread = threading.Thread(target=overlay.run, args=(args, self._stop, True), name="floor-overlay", daemon=True); self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set(); super().stop()
