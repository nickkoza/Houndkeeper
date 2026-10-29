#!/usr/bin/env python3
"""Sim suite: named scenarios, run the same way every time, results appended to history.jsonl.
  ./run list                     scenarios and their last result
  ./run grasp [--trials 40]      grasp bench (robot lying, items spawn in reach; look, grip, carry, bin)
  ./run slice                    full vertical slice, several items, clustered stops
  ./run walk                     walking with the arm stowed (stability, speed tracking)
  ./run reach                    reach map while lying (feeds the stop planner)
  ./run power                    arm energy per pick across item masses
  ./run tip                      tip safety: CoM margin over the arm's reach, safe arm speed, and the measured tipping point
  ./run mount                    arm mount placement sweep: reach, tipping, walking, camera view, bin room per position
  ./run history grasp            past results for a scenario
Viewer on by default (--headless to turn off). Every run records: scenario, time, args, code fingerprint, metrics."""
import hashlib, json, subprocess, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent; ARM = HERE.parent / "arm"; HIST = HERE / "history.jsonl"
PY = str(HERE.parents[2] / "dimos" / ".venv" / "bin" / "python")   # the DimOS venv, in the checkout beside this repo
SCEN = {  # name: (script in sim/arm, default args, how to pull metrics from its output)
    "grasp": ("grasp_bench.py", ["--trials", "20", "--seed", "0"], ("OUTCOMES", "SUMMARY")),
    "slice": ("vslice_multi.py", [], "{"),
    "walk":  ("payload_test.py", [], "=="),
    "reach": ("make_reach_map.py", [], "reachable"),
    "power": ("power_budget.py", [], "{"),
    "tip":   ("tip_all.py", [], ("SAFE", "REACH", "TIPPING")),
    "mount": ("mount_sweep.py", [], "MOUNT"),
}
def fingerprint():
    h = hashlib.sha1()
    for f in sorted(ARM.glob("*.py")): h.update(f.read_bytes())
    return h.hexdigest()[:10]
def run(name, extra):
    script, args, key = SCEN[name]; headless = "--headless" in extra
    env = {"DISPLAY": ":1", "XAUTHORITY": "/run/user/1000/gdm/Xauthority", "MUJOCO_GL": "egl" if headless else "glfw", "HF_HUB_OFFLINE": "1",
           "PATH": "/usr/bin:/bin", "HOME": str(Path.home())}
    cmd = [PY, script] + args + extra; t0 = time.time(); lines = []
    p = subprocess.Popen(cmd, cwd=ARM, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in p.stdout:
        if any(s in line.lower() for s in ("warn", "onnxruntime", "provider")): continue
        print(line, end="", flush=True); lines.append(line.rstrip())
    p.wait()
    metrics = [l for l in lines if l.startswith(key if isinstance(key, tuple) else (key,))]
    rec = {"scenario": name, "time": time.strftime("%Y-%m-%d %H:%M:%S"), "args": args + extra, "code": fingerprint(),
           "exit": p.returncode, "wall_s": round(time.time() - t0), "result": metrics[-16:]}
    with open(HIST, "a") as fh: fh.write(json.dumps(rec) + "\n")
    print(f"\n[suite] {name}: exit {p.returncode}, {rec['wall_s']} s, logged to {HIST.name} (code {rec['code']})")
def hist(name=None):
    if not HIST.exists(): print("no history yet"); return
    for l in HIST.read_text().splitlines():
        r = json.loads(l)
        if name is None or r["scenario"] == name: print(f"{r['time']}  {r['scenario']:6s} code {r['code']}  exit {r['exit']}  {r['wall_s']:5d}s  {(r['result'][-1] if r['result'] else '')[:150]}")
if __name__ == "__main__":
    a = sys.argv[1:]
    if not a or a[0] == "list":
        for k, (s_, args, _) in SCEN.items(): print(f"{k:6s} sim/arm/{s_} {' '.join(args)}")
        print(); hist()
    elif a[0] == "history": hist(a[1] if len(a) > 1 else None)
    elif a[0] in SCEN: run(a[0], a[1:])
    else: print(__doc__)
