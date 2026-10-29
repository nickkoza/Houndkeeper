"""The operating area published to DimOS's costmap ([JVr]; decision [u4W]): the area file, re-read when it changes,
placed in the odometry's frame and sent as the costmap layer's keep-out zones, again every few seconds so a late
subscriber has it. The tidy publishes it the same way, with its localization's correction; this module is for a run
without the tidy (an exploration night, sim/house/sweep_night.sh), where the house's frame and the odometry's are taken
to agree. Never run both in one blueprint: two publishers of one boundary.

    dimos run unitree-go2-agentic houndkeeper.operating-area     (HK_AREA: the area file, or the map directory)"""
from __future__ import annotations

import json, os, threading
from collections.abc import Callable
from pathlib import Path

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.std_msgs.String import String
from dimos.utils.logging_config import setup_logger

from houndkeeper import operating_area as oa

logger = setup_logger()


class AreaPublisher:
    """the area at `path` as the costmap layer's JSON, through `send`, whenever the file or the correction changes, and
    every REPEAT_S anyway"""
    REPEAT_S = 5.0
    MOVED_M, TURNED_RAD = 0.02, 0.005   # a correction that moved the area more than this republishes it

    def __init__(self, path, send: Callable[[str], None], correction: Callable[[], np.ndarray | None] = lambda: None):
        self.path, self.send, self.correction = path, send, correction
        self.area: oa.Area | None = None; self.error = ""
        self._stamp = None; self._sent_C = None; self._sent_t = -1e9

    def _file(self):
        p = Path(self.path); return p / oa.FILE if p.is_dir() else p

    def reload(self) -> bool:
        """re-read the file if it changed; True if the area changed"""
        f = self._file()
        st = f.stat().st_mtime_ns if f.exists() else None
        if st == self._stamp: return False
        self._stamp = st
        try: self.area = oa.load(f); self.error = ""
        except (ValueError, OSError) as e:            # a broken file: keep the last good area, and say so
            self.error = f"operating area not read ({e}); keeping the last one"; logger.error(self.error)
            return False
        logger.info(f"operating area: {'none' if self.area is None else f'{len(self.area.allowed)} polygon(s), {len(self.area.walls)} wall(s)'} from {f}")
        return True

    def tick(self, now: float) -> bool:
        """publish if anything changed or REPEAT_S passed; True if it published"""
        changed = self.reload()
        if self.area is None: return False
        C = self.correction(); C = np.eye(3) if C is None else np.asarray(C, float)
        moved = self._sent_C is None or np.abs(C[:2, 2] - self._sent_C[:2, 2]).max() > self.MOVED_M \
            or abs(np.arctan2(C[1, 0], C[0, 0]) - np.arctan2(self._sent_C[1, 0], self._sent_C[0, 0])) > self.TURNED_RAD
        if not (changed or moved or now - self._sent_t >= self.REPEAT_S): return False
        self.send(oa.zones_json(self.area, C)); self._sent_C, self._sent_t = C, now
        return True


class OperatingArea(Module):
    """the area file published as keep-out zones, frames taken to agree ([rLQ], [OrO]: exploration sees no frontier
    outside it)"""
    keepout_zones: Out[String]

    @rpc
    def start(self) -> None:
        super().start()
        from dimos.utils import clock
        path = os.environ.get("HK_AREA") or str(oa.map_dir())
        self._pub = AreaPublisher(path, lambda j: self.keepout_zones.publish(String(j)))
        self._halt = threading.Event()
        def loop():
            while not self._halt.wait(1.0):
                try: self._pub.tick(clock.now())
                except Exception as e: logger.error(f"operating area not published: {e!r}")
        threading.Thread(target=loop, name="operating-area", daemon=True).start()

    @rpc
    def stop(self) -> None:
        ev = getattr(self, "_halt", None)
        if ev is not None: ev.set()
        super().stop()


# ---- the Area page's robot side ([daF], [HZO]) ----

MAP_RES = 0.05                # m per pixel of the map the page draws on
MAP_MAX_PX = 640              # its longest side, at most (a coarser pixel past it)
FLOOR_Z, TOP_Z = 0.10, 2.0    # house map points below FLOOR_Z are floor; between, something stands


def house_map_image(md: Path) -> dict | None:
    """the house's lidar map (<map dir>/house_lidar.npy: points in the house's frame, x y [z]; 2D points are walls) as a small PNG for the page to draw
    on: {"png": base64, "x0", "y0" (the image's bottom-left corner, m), "res" (m per pixel), "w", "h"}; None without one"""
    import base64, io
    from PIL import Image as PILImage
    f = md / "house_lidar.npy"
    if not f.exists(): return None
    P = np.asarray(np.load(f), float)
    if P.ndim != 2 or len(P) == 0: return None
    lo, hi = P[:, :2].min(0) - 0.5, P[:, :2].max(0) + 0.5
    res = max(MAP_RES, float((hi - lo).max()) / MAP_MAX_PX)
    w, h = (np.ceil((hi - lo) / res)).astype(int)
    c = ((P[:, 0] - lo[0]) / res).astype(int).clip(0, w - 1); r = (h - 1 - ((P[:, 1] - lo[1]) / res).astype(int)).clip(0, h - 1)
    img = np.full((h, w), 0, np.uint8)                                 # unseen: black
    if P.shape[1] < 3: low = np.zeros(len(P), bool); up = ~low          # a 2D map: every point is a wall's
    else: low = P[:, 2] < FLOOR_Z; up = ~low & (P[:, 2] < TOP_Z)
    img[r[low], c[low]] = 70                                           # floor: dim
    img[r[up], c[up]] = 200                                            # walls and furniture: light
    buf = io.BytesIO(); PILImage.fromarray(img, "L").save(buf, "PNG", optimize=True)
    return {"png": base64.b64encode(buf.getvalue()).decode(), "x0": round(float(lo[0]), 3), "y0": round(float(lo[1]), 3),
            "res": round(res, 4), "w": int(w), "h": int(h)}


def dock_of(md: Path) -> tuple[float, float] | None:
    """the dock in the house's frame, if the map directory records it (dock.json {"x", "y"})"""
    try: d = json.loads((md / "dock.json").read_text()); return float(d["x"]), float(d["y"])
    except (OSError, ValueError, KeyError, TypeError): return None


def history(md: Path, n: int = 5) -> list[dict]:
    """the last n changes from the log, newest first: when, by whom, how many polygons and walls"""
    try: lines = (md / oa.LOG).read_text().splitlines()[-n:]
    except OSError: return []
    out = []
    for ln in reversed(lines):
        try: d = json.loads(ln); out.append({"changed": d.get("changed", ""), "by": d.get("by", ""),
                                             "polygons": len(d.get("allowed", [])), "walls": len(d.get("walls", []))})
        except ValueError: continue
    return out


class AreaFeed(Module):
    """the Area page's data (cockpit/extensions/hk-area.js): the house map, the stored area, its last changes and the
    warning, on area_state; a confirmed edit from the page on area_cmd is validated and stored with the map, and logged
    with its time ([daF]). The page names the change it replaced ("seen"), so a stale tab cannot overwrite a newer area.
    Its map directory: HK_MAP_DIR, else the real house's (operating_area.REAL_MAP_DIR)."""
    area_cmd: In[dict]
    area_state: Out[dict]
    PUBLISH_S = 2.0

    @rpc
    def start(self) -> None:
        super().start()
        from reactivex.disposable import Disposable
        self._md = oa.map_dir(); self._message = ""; self._map = None; self._map_stamp = None
        self._halt = threading.Event()
        self.register_disposable(Disposable(self.area_cmd.subscribe(self._on_cmd)))
        def loop():
            while True:
                try: self.area_state.publish(self.state())
                except Exception as e: logger.error(f"area page: {e!r}")
                if self._halt.wait(self.PUBLISH_S): return
        threading.Thread(target=loop, name="area-feed", daemon=True).start()

    @rpc
    def stop(self) -> None:
        ev = getattr(self, "_halt", None)
        if ev is not None: ev.set()
        super().stop()

    def state(self) -> dict:
        f = self._md / "house_lidar.npy"; st = f.stat().st_mtime_ns if f.exists() else None
        if st != self._map_stamp: self._map, self._map_stamp = house_map_image(self._md), st
        area, error = None, ""
        try: a = oa.load(self._md); area = a.to_dict() if a else None
        except (ValueError, OSError) as e: error = f"the stored area could not be read: {e}"
        dock = dock_of(self._md)
        return {"map": self._map, "area": area, "dock": list(dock) if dock else None, "history": history(self._md),
                "warning": oa.WARNING, "message": error or self._message, "frame": oa.FRAME}

    def _on_cmd(self, cmd) -> None:
        try: self.handle(cmd if isinstance(cmd, dict) else json.loads(cmd))
        except Exception as e: self._message = f"not saved: {e}"

    def handle(self, cmd: dict) -> None:
        if cmd.get("op") != "save": self._message = f"unknown command {cmd.get('op')!r}"; return
        if cmd.get("confirmed") is not True: self._message = "not saved: the change was not confirmed"; return
        try: cur = oa.load(self._md)
        except ValueError: cur = None
        if (cur.changed if cur else "") != (cmd.get("seen") or ""):
            self._message = "not saved: the area changed since this page loaded it (look again, then save)"; return
        try: a = oa.save(self._md, cmd.get("area") or {}, by="the phone cockpit", dock=dock_of(self._md))
        except ValueError as e: self._message = f"not saved: {e}"; return
        n, w = len(a.allowed), len(a.walls)
        self._message = f"saved {a.changed}: {n} area{'' if n == 1 else 's'}, {w} virtual wall{'' if w == 1 else 's'}"
        self.area_state.publish(self.state())
