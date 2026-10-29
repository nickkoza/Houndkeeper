"""[UvE]: our DimOS settings come from houndkeeper (sim/dimos/env.py), and the config DimOS resolves for our tidy and
sweep command lines equals what the fork once hard-wired, so our runs behave as before. Static: DimOS's own config
parser on our environment and flags, no sim."""
import json
import os
import re
import subprocess
import sys
from pathlib import Path

from houndkeeper import workspace

import pytest

HK = Path(__file__).resolve().parents[1]
DIMOS = workspace.dimos()
PY = DIMOS / ".venv" / "bin" / "python"
ITEMS = '[["block","block",[-1.9,0.3,0.1]]]'

# what the fork hard-wired before [UvE], in DimOS's terms
OLD = {"person": False, "geomgroup": [1, 1, 1, 0, 1, 0], "arm_rest": [0.0] * 6, "arm_idle_s": 1.0, "arm_tol": 0.15,
       "truth_names": ["trunk", "tray"], "policy_gpu": False, "steps_per_frame": 28,
       "costmap": {"resolution": 0.05, "frame_id": None, "ignore_above": 2.0, "can_pass_under": 3.0, "can_climb": 0.15,
                   "ignore_noise": 0.05, "smoothing": 1.0}}

PROBE = r'''
import json, os, sys
sys.path.insert(0, sys.argv[1] + "/sim/dimos"); import env, effective_config as ec
mode = sys.argv[2]
if mode != "stock":
    env.apply(sys.argv[3] if mode == "tidy" else None)
    if mode == "tidy": os.environ.update(json.loads(sys.argv[4]))       # what tidy_run.sh exports beside env.py
    if mode == "sweep": os.environ["DIMOS_SIM_ARM_CAMERA"] = ""          # as sweep_night.sh
if mode == "tidy":
    bps, toks = ["unitree-go2", "mcp-server", "houndkeeper.tidy"], ["--simulation", "mujoco", "--mujoco-room", "nights_s70_n3",
                                                                     "--mujoco-start-pos", "-1.0, 1.0", "--record", "sqlite"]
else:
    bps, toks = ["unitree-go2-agentic"], ["--simulation", "mujoco", "--mujoco-room", "nights_s70_n1", "--record", "sqlite",
                                          "--robot-width", "0.5", "--voxelgridmapper.carve-columns", "false"]
g, cm = ec.effective(bps, toks)
from dataclasses import asdict
from dimos.simulation.mujoco.mujoco_process import ArmCameraThrottle, frame_steps, sensor_scene_option
from dimos.simulation.mujoco.model import default_timestep
th = ArmCameraThrottle.from_config(None, range(6), g) if g.mujoco_arm_camera_rest_pose else None
print("PROBE " + json.dumps({
    "person": g.mujoco_person, "geomgroup": [int(v) for v in sensor_scene_option(g.mujoco_sensor_geomgroups).geomgroup],
    "arm_rest": th.rest_q.tolist() if th else None, "arm_idle_s": g.mujoco_arm_camera_idle_s, "arm_tol": g.mujoco_arm_camera_rest_tol,
    "truth_names": [n for n in g.mujoco_truth_bodies.split(",") if n], "policy_gpu": g.mujoco_policy_gpu,
    "steps_per_frame": frame_steps(g.mujoco_steps_per_frame, "unitree_go1", g.mujoco_dt or default_timestep("unitree_go1")),
    "costmap": asdict(cm), "problem": ec.costmap_problem(cm),
    "fields": g.model_dump(mode="json"), "environ": {k: v for k, v in os.environ.items() if k.startswith("DIMOS_SIM_")}}))
'''


def _probe(mode: str) -> dict:
    if not PY.exists():
        pytest.skip("no DimOS venv")
    env = {k: v for k, v in os.environ.items() if not k.startswith(("DIMOS_SIM_", "COSTMAPPER__", "HK_"))}
    tidy_exports = dict(re.findall(r'^export (DIMOS_SIM_TRUTH_BODIES)="([^"]*)"', (HK / "sim/dimos/tidy_run.sh").read_text(), re.M))
    assert tidy_exports, "tidy_run.sh no longer names the truth bodies"
    out = subprocess.run([str(PY), "-c", PROBE, str(HK), mode, ITEMS, json.dumps(tidy_exports)], cwd=DIMOS, env=env,
                         capture_output=True, text=True,
                         timeout=600)
    line = next((ln for ln in out.stdout.splitlines() if ln.startswith("PROBE ")), None)
    assert line, out.stdout[-2000:] + out.stderr[-3000:]
    return json.loads(line[6:])


@pytest.fixture(scope="module", params=["tidy", "sweep"])
def ours(request):
    return request.param, _probe(request.param)


def test_our_runs_get_what_the_fork_hard_wired(ours):
    mode, got = ours
    for k, v in OLD.items():
        if k == "truth_names" and mode == "sweep":
            continue                             # the truth file is written by the tidy's runs only (tidy_run.sh)
        assert got[k] == v, f"{mode}: {k} {got[k]} != {v}"
    assert got["problem"] is None


def test_every_dimos_sim_variable_reaches_its_typed_option(ours):
    """each DIMOS_SIM_* our scripts export (read once by the fork's own os.environ) arrives as its GlobalConfig field"""
    from_names = {"SPEED": "sim_speed"}
    mode, got = ours
    unread = {"DIMOS_SIM_CLOCK", "DIMOS_SIM_CAMERA_FOVY", "DIMOS_SIM_CAMERA_INFO_YAML",   # still environment (import time,
              "DIMOS_SIM_VIDEO_SIZE"}                     # or every process's clock; the frame size, [7cU], exported by default since [bFx])
    for name, raw in got["environ"].items():
        if name in unread:
            continue
        short = name.removeprefix("DIMOS_SIM_")
        field = "mujoco_" + from_names.get(short, short.lower())
        assert field in got["fields"], f"{mode}: {name} has no option"
        val = got["fields"][field]
        if isinstance(val, bool):
            assert val == (raw == "1"), (name, raw, val)
        elif isinstance(val, float):
            assert val == float(raw), (name, raw, val)
        else:
            assert val == raw, (name, raw, val)


def test_stock_dimos_keeps_its_own_defaults():
    got = _probe("stock")
    assert got["person"] is True and got["geomgroup"] == [1, 1, 1, 0, 0, 0] and got["truth_names"] == []
    assert got["policy_gpu"] is True and got["arm_idle_s"] == 0.0
    assert got["costmap"]["can_pass_under"] == 0.6 and got["costmap"]["ignore_above"] is None
    assert got["problem"]                        # sweep_night.sh's check refuses stock settings
