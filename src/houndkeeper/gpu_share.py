"""One 8 GB GPU shared by the sim's jobs ([jlr]): a DimOS night (its sim, Filament cameras and the arm's close-look model)
holds it alone; standalone sims (demo recording, policy evaluation) share it with each other. Overcommitting it crashes
whichever job allocates last (Filament "Unable to allocate image memory", CUDA out of memory), 25 Sep. A night announces
itself before it waits, so a stream of shared holders can't starve it. Shell side: sim/dimos/gpu_lock.sh."""
from __future__ import annotations

import fcntl
import os
import time
from pathlib import Path

LOCK = Path.home() / ".cache" / "houndkeeper" / "gpu.lock"
NIGHT_WAITING = LOCK.with_suffix(".night-waiting")
_held = None


def _waiting_night_gone() -> bool:
    """the announcing night's process (gpu_lock.sh writes its pid) has ended: a night killed while it waited never
    removes its announcement, and every sim after it would wait on it for good. Clears the stale announcement."""
    try: pid = int(NIGHT_WAITING.read_text().split()[0])
    except (OSError, ValueError, IndexError): return False      # gone already, or no pid in it: treated as live
    try: os.kill(pid, 0); return False
    except ProcessLookupError: pass
    except PermissionError: return False                        # running, as another user
    try: NIGHT_WAITING.unlink()
    except FileNotFoundError: pass
    return True


def share(say=print) -> None:
    """hold the GPU shared with other standalone sims until this process exits (a no-op inside a job already holding it)"""
    global _held
    if _held is not None or os.environ.get("HK_GPU_LOCKED"): return
    LOCK.parent.mkdir(parents=True, exist_ok=True)
    told = False
    while True:
        while NIGHT_WAITING.exists():                           # a night is next: let it have the GPU first
            if _waiting_night_gone(): continue                  # unless it died waiting: its announcement is stale
            if not told: say("GPU: waiting for a DimOS night to finish"); told = True
            time.sleep(10)
        f = open(LOCK, "a")
        try:
            fcntl.flock(f, fcntl.LOCK_SH | fcntl.LOCK_NB); _held = f; return
        except BlockingIOError:
            f.close()
            if not told: say("GPU: waiting for a DimOS night to finish"); told = True
            time.sleep(10)
