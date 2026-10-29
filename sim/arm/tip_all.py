#!/usr/bin/env python3
"""The tip-safety scenario ([BPL]): CoM margin and safe arm speed (tip_bench.py), then the measured tipping point per
posture, lean direction and arm load (tip_recovery.py). Same flags pass through (e.g. --headless)."""
import subprocess, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
for script in ("tip_bench.py", "tip_recovery.py"):
    r = subprocess.run([sys.executable, str(HERE / script)] + sys.argv[1:], cwd=HERE)
    if r.returncode != 0: sys.exit(r.returncode)
