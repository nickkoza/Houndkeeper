"""Two cameras through one renderer don't re-make the shadow atlas every frame (waymark [U8i]; 18 Oct 2026: 3 of 5 sim
arm nights aborted). The head and wrist cameras share one Filament renderer; a shadow-casting lamp seen by one and not
the other gives their shadow atlases a layer more or fewer, and with Filament's default texture cache (released
textures kept 1 frame) the 512-554 MiB atlas was made anew each frame, until beside the tidy's recognizers an allocation
failed ("VulkanTexture: Unable to allocate image memory", an uncaught utils::PostconditionPanic on Filament's driver
thread). mujofil keeps released textures 60 frames (tools/mujofil_texture_cache.patch), so each atlas is made once.

The night's own model renders a close look (the arm through poses, head and wrist each frame) under a Vulkan layer that logs every image it creates (tools/vk_imglog); the
shadow atlas is the one 2D array image. The planted control runs the same with the cache at Filament's default
(MUJOFIL_TEXTURE_CACHE_FRAMES=1) and must churn, or the test proves nothing. Runs on the mujofil build HK_MUJOFIL_DIR
chooses, else the default one."""
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
SRC = HK / "src" / "houndkeeper"
LAYER = HK / "tools" / "vk_imglog"
FRAMES = 200

CHILD = r"""
import os, shlex, subprocess, sys
import numpy as np
EXP = subprocess.run([sys.executable, "-c", "import env; print(env.shell_exports('[]'))"], cwd=sys.argv[3],
                     capture_output=True, text=True, check=True).stdout
for tok in shlex.split(EXP):
    if "=" in tok: k, v = tok.split("=", 1); os.environ.setdefault(k, v)
sys.path.insert(0, sys.argv[1])
import mujoco
from dimos.core.global_config import GlobalConfig
from dimos.simulation.mujoco.model import load_model, load_scene_xml
cfg = GlobalConfig(); cfg.mujoco_room = "nights_s70_n3"          # a night's own model: the Go2, the arm, house 70
class Ctl:
    def get_command(self): return np.zeros(3)
m, d = load_model(Ctl(), robot="unitree_go1", scene_xml=load_scene_xml(cfg), config=cfg); mujoco.set_mjcb_control(None)
d.qpos[0:3] = [-5.0, -1.0, 0.3]
import filament_cameras as fc
cams = fc.make(m, d)
head = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera"); wrist = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "arm_wrist")
joints = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in os.environ["DIMOS_SIM_ARM_JOINTS"].split(",")]
qadr = [m.jnt_qposadr[j] for j in joints]; rng = [m.jnt_range[j] if m.jnt_limited[j] else (-1.5, 1.5) for j in joints]
R = np.random.default_rng(0); a = np.zeros(len(qadr)); b = a.copy()
for i in range(int(sys.argv[2])):                     # a close look: the arm through poses, head and wrist each frame
    if i % 40 == 0: a, b = b, np.array([R.uniform(*r) for r in rng]) * 0.8
    d.qpos[qadr] = a + (b - a) * (i % 40) / 40.0
    mujoco.mj_kinematics(m, d); mujoco.mj_camlight(m, d)
    cams.render(d, head); cams.render(d, wrist)
print("RENDERED", flush=True)
"""


def _atlases(tmp_path, cache_frames):
    """how many shadow atlases (square 2D array images) the renderer made over FRAMES frames of a close look"""
    env = dict(os.environ, VK_LAYER_PATH=str(tmp_path), VK_INSTANCE_LAYERS="VK_LAYER_HK_imglog")
    if cache_frames is not None: env["MUJOFIL_TEXTURE_CACHE_FRAMES"] = str(cache_frames)
    p = subprocess.run([sys.executable, "-c", CHILD, str(SRC), str(FRAMES), str(HK / "sim" / "dimos")], capture_output=True, text=True, env=env,
                       timeout=600)
    assert p.returncode == 0 and "RENDERED" in p.stdout, f"renderer child failed ({p.returncode}): {p.stderr[-2000:]}"
    images = re.findall(r"^VKIMG (\d+)x(\d+)x\d+ layers=(\d+)", p.stderr, re.M)
    assert images, "the image-logging layer didn't load: nothing to count"
    return sum(1 for w, h, layers in images if int(layers) > 1 and w == h)


def test_alternating_cameras_keep_one_shadow_atlas_each(tmp_path):
    pytest.importorskip("mujoco")
    if not shutil.which("gcc") or not Path("/usr/include/vulkan/vk_layer.h").exists():
        pytest.skip("the image-logging layer needs gcc and the Vulkan headers")
    sys.path.insert(0, str(SRC)); import filament_cameras as fc
    fc.use_chosen_mujofil(); pytest.importorskip("mujofil")
    subprocess.run(["gcc", "-shared", "-fPIC", "-O1", "-o", str(tmp_path / "libimglog.so"), str(LAYER / "imglog.c")], check=True)
    shutil.copy(LAYER / "imglog.json", tmp_path)
    control = _atlases(tmp_path, 1)                           # planted: Filament's default, the churn
    assert control >= FRAMES // 10, f"control made {control} atlases in {FRAMES} frames: the scene no longer splits it"
    fixed = _atlases(tmp_path, None)                          # the build's own setting
    assert fixed <= 15 and 3 * fixed <= control, f"{fixed} shadow atlases made in {FRAMES} frames of a look (control {control}): re-made frame after frame"
