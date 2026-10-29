# [UQK] — The robot's cameras render through Filament; the lidar and the viewer stay classic

**Status:** Proposed · **Date:** 2026-09-25 · **Source:** the classic renderer's images falling well short of games';
the aim to render with Filament and retrain for it, without downgrading the sim ·
**Serves:** [CaM] (principle), [QMt] (house lighting), [5AP] (viewer insets), the training pipeline, sim-to-real ·
**ID:** [UQK]

## Context

MuJoCo's classic renderer is built for speed and debugging: no bounce light, no ambient occlusion, no physically based
materials, one hard shadow map per light, at most 8 lights. The perception the robot runs (the floor comparison,
confirm-on-approach, the arm's close look, the learned pick) learns from and is tuned on these images, so a flat sim
hides what real night footage will do to it. MuJoCo 3.12 has a Filament renderer, but its drop-in mode breaks the
interactive viewer (it removes MuJoCo's UI library) and its Python module isn't in MuJoCo's normal Python build or in
Google's own wheels (checked 2026-09-25). `mujofil` (Tau Intelligence, Apache-2.0) renders MuJoCo models with Filament
from ordinary MjModel/MjData, but its wheels are built against MuJoCo 3.9, and DimOS runs 3.10.

## Decisions

1. [rMJ] **Forced: mujofil rebuilt against the MuJoCo DimOS runs, not MuJoCo downgraded.** mujofil reads MuJoCo's
   structs at compile-time offsets from vendored headers, so a MuJoCo minor needs a rebuild with its headers ("re-vendor
   the headers and rebuild", per mujofil's pyproject); the rebuild itself changes nothing in its code (the rendering fixes
   on top are [akm]). `tools/build_mujofil.sh` does it
   with a conda-forge toolchain installed by micromamba (no sudo): clang + libc++, CUDA 12 headers and static runtime,
   lld, zlib. Checked 2026-09-25, mujofil 0.2.6 @ 339a0fa, MuJoCo 3.10.0: pixel-identical to the official 3.9 wheel.
   Hard-won along the way, each now in the script: newer clang rejects Filament 1.56.3's std trait specialisations
   (`-Wno-invalid-specialization`); CUDA's `crt/` headers come with `cuda-nvcc`; conda's ld can't read Ubuntu 24.04's
   glibc (lld); Ubuntu's Python header needs `/usr/include` (`-idirafter`); and libc++ is linked statically with its
   symbols hidden, because open3d (imported first in DimOS's sim) ships an older `libc++.so.1` that a shared link
   resolves to ("undefined symbol ...__hash_memory").
2. [uyU] **Forced: the Vulkan backend.** DimOS's sim renders its viewer and depth cameras through GLFW, whose OpenGL
   context is current on the sim thread; mujofil's headless EGL then fails ("eglMakeCurrent failed"). Vulkan runs
   alongside it: 5-6 ms a 640x360 frame on the RTX 2070 (`MUJOFIL_BACKEND=vulkan`, set in `filament_cameras.py`).
3. [Hsz] **The head and wrist frames the robot's perception gets come from Filament; lidar and viewer stay classic.**
   DimOS's sim takes camera frames from `DIMOS_SIM_CAMERA_RENDERER` (`houndkeeper.filament_cameras:make`, set by
   `sim/dimos/env.py` unless `HK_RENDERER=classic`); the standalone arm sims' `capture` and `small_image` use the same
   module. The viewer's insets show those same frames ([5AP]). The lidar needs only geometry, and the viewer is for
   people. Lights are re-added in the renderer's units by what they are (house lights by name): mujofil adds a small
   fixed fill light, and lights set well above it keep night exposures low enough that the fill doesn't haze the
   frame. Exposure: every frame is metered and the exposure eases toward mid-grey for the next (15% of the way, at most
   a quarter stop, a frame: ~0.3 s at 20 frames/s; a stop a frame looked like lights popping on, 26 Sep), as a camera's
   auto-exposure does (mujofil's `set_exposure`, added by our patch [akm]): metered once, a camera walking
   out of the lamp's pool went black (53% of a night's frames, 26 Sep). By night a weak ambient stands in for the light
   bounced off walls and ceiling, which Filament doesn't trace; without it anything a lamp doesn't reach directly is
   black at any exposure. Filament traces no bounces, so the ambient is tinted with the room's own colour (the floor's
   and walls' mean, at unit brightness; our mujofil patch [akm] adds `set_ambient_tint`): a beige room's shade is warm,
   not the sky's blue (26 Sep). Exposure meters to mid-grey (0.45, an 18% card in
   sRGB), as cameras do; 0.30 ran a stop dark. It stops short where the brightest 2% of the frame
   would pass 92%, as a camera's metering protects highlights (30 Sep: a dark slate floor metered to mid-grey blew a
   beige carpet and a pale table to white, the scene massively over-lit). A light that states
   its size (MJCF `bulbradius`) keeps it for its soft shadows' width; the rest take their kind's size. The cameras share one renderer per regime, each keeping its own exposure: a renderer per
   camera held a copy of every texture per camera, and with the photographed house floor that ran a night out of GPU
   memory (26 Sep). Everything Filament offers is on, to be as photographic as it goes: 8x MSAA,
   bloom, reflections, SSAO, and the lamps' soft shadows (our mujofil patch [akm] lets lamps cast shadows, which
   upstream's `add_spot_light` never did). Hard-won, each measured 26 Sep: (1) SSAO's cone-traced contact shadows
   (SSCT) are off. They banded across large surfaces seen the length of the house, and the bands moved as the camera
   crept (it looked like z-fighting: 0.4-7% of pixels jumped between frames 1 mm apart; 0.00% with SSCT off and
   SSAO on; SSAO's resolution and bias didn't help), which the floor comparison would read as change
   (`tests/test_filament_flicker.py`). (2) Soft shadows are blurred variance shadow maps (VSM, 32-bit, blur from each
   lamp's size), not PCSS or DPCF: those draw the penumbra as a dithered speckle that Filament's temporal
   anti-aliasing smooths, and TAA can't be used when cameras share a renderer (it would blend their views). 16-bit
   VSM was grainy. The lamps' reach
   (Filament's falloff, beyond which a light gives nothing) is 30 m, past the house's far corner: it was 10 m. All values are estimates, versioned by
   `filament_cameras.CALIBRATION` in the render stamp ([Xl1]), so changing them re-renders the baselines.
   Hard-won: mujofil hands frames back bottom row first (OpenGL's order; mujofil 0.2.6 @ 339a0fa, 25 Sep 2026), so they
   are flipped to the classic renderer's top-down order in one place (`_shot`), held by
   `tests/test_filament_orientation.py`. Until then every Filament frame was upside down, including the first Filament
   demos (flipped back afterwards) and the first Filament DimOS night (re-run). A check that compares mujofil only
   with mujofil can't see it.
4. [eIH] **Every standalone episode draws its own lighting** (night: warm ceiling light, often a lamp; day: a window's
   glow, a third of days the sun), as the house nights do ([QMt]): a policy trained under one fixed light breaks when
   the light changes (v3 lifted 0/2 in DimOS's differently lit house). `Sim.new_floor` relights with the floor, so no
   scenario can forget it.
5. [jlr] **Hard-won: a DimOS night holds the GPU alone; standalone sims share it.** On the 8 GB RTX 2070 a DimOS night
   (sim, Filament cameras, the arm's close-look model) and two demo recorders (~2.4 GB each: the classic sim with its
   item library ~0.9 GB, each Filament renderer ~0.5 GB, one per regime) don't fit together, and whichever allocates last
   crashes: Filament aborts ("Unable to allocate image memory") or the close look fails ("CUDA out of memory"), and the
   night is scored as a failed tidy though nothing in the tidy failed (25 Sep). A lock file orders them
   (`sim/dimos/gpu_lock.sh`, `src/houndkeeper/gpu_share.py`): the night holds it exclusively and announces itself
   before it waits, so recorders starting one after another can't starve it.
6. [akm] **Hard-won: mujofil is patched to draw MuJoCo's textured planes as MuJoCo does, and to use its normal and ORM
   layers** (`tools/mujofil_pbr_maps.patch`, `tools/mujofil_materials/textured_pbr.mat`, built by
   `tools/build_mujofil.sh`; mujofil 0.2.6 @ 339a0fa, observed 25 Sep 2026). Upstream drew every plane 20 m square
   whatever its size, scaled plane textures by a fixed 0.13 per metre instead of MuJoCo's rule, never re-read a texture a
   program rewrites (the arm sim's floor changes every trial), and read only the colour layer. Until the patch, every
   Filament frame of the arm sim showed one frozen floor per renderer, magnified 6-8x, on 49 overlapping planes: the
   first 2,595 Filament demonstrations (25 Sep) have almost no floor variety and were not trained on. MuJoCo's rule,
   measured on its classic renderer (3.10): the texture is centred on the plane; `texuniform` means `texrepeat` repeats
   per 2 m, otherwise `texrepeat` repeats across the plane. Held by `tests/test_filament_orientation.py` (Filament
   against classic) and `tests/test_floor_photos.py` (relief lit from the side it faces). The patch also samples
   textures with 8x anisotropic filtering, so floors at a grazing angle don't fall to coarse mip levels. The material is
   compiled with Filament 1.56.3's own matc (material version 56); another version's matc builds materials mujofil
   can't load. It adds `set_exposure`, changing a built renderer's exposure without rebuilding it ([Hsz]). It draws MuJoCo flex bodies
   (cloth, soft bodies), which upstream skipped: a mesh rebuilt from the flex's vertices every frame, each side drawn its
   contact radius out as MuJoCo draws it (`tests/test_filament_flex.py`). Its material has specular anti-aliasing and
   draws normal-map relief at 60%: a normal-mapped floor otherwise sparkled from pixel to pixel as the camera moved
   ("marching ants", 26 Sep), there being no temporal anti-aliasing to smooth it.
7. [3my] **Hard-won: only the nearest lamp is drawn, fixed by Filament 1.77.1.** Found 28 Sep 2026 (Filament 1.56.3 as mujofil links it,
   Vulkan backend, the stock `materials/` set). On a bare floor with two spot lamps 3 m apart, each lighting its own
   half of the frame (about 25 against 17 unlit), both on renders exactly as the nearer lamp alone, whichever it is,
   even when their reach doesn't overlap (2.5 m). So a house is lit by one lamp at a time, and the light pops when
   the camera crosses the point where the lamps are equally far (seen on both cameras; on night 70/6's
   path at 165.75 s a 2 cm move doubled a frame's brightness at fixed exposure, the lamps 4.8368 and 4.8515 m away).
   Ruled out: auto-exposure (under 0.10 stop a frame over 4,000 replayed frames), shadows (off, PCF, PCSS, VSM),
   SSAO, SSR, bloom, MSAA, a lamp's reach and cone, the order lamps are added, spot against point lamps, mujofil's
   light code (a new entity per lamp), and a mismatch between the material and the engine (version 56 both; the light
   lists' packing and decoding agree in 1.56.3's source). Threatens: every night has been lit wrongly, and a pop
   inside a baseline picture is a false floor change. Google's stock 1.56.3 does the same, so it isn't the fork's patch;
   its OpenGL backend needs an X display (GLX), so it isn't an alternative. Forced, observed 28 Sep 2026: Filament
   1.77.1 on Vulkan draws both (`tests/test_filament_lamps.py`, which fails on 1.56.3), and on night 70/6's path the
   frame rises smoothly (158.4 to 160.7) where 1.56.3 jumped from 78.9 to 159.9; the other Filament tests pass on it,
   and a frame costs the same (11.0 and 11.5 ms, measured with a night on the GPU). Moving mujofil to it took
   (`tools/mujofil_filament_new.patch`, applied by `tools/build_mujofil.sh` with `FILAMENT_DIR` and `FILAMENT_TAG`):
   every material recompiled with 1.77.1's matc; Filament's source headers mujofil keeps, taken from the same tag;
   cubemaps uploaded as 6 layers (`FaceOffsets` is gone); a mipmapped texture declared `GEN_MIPMAPPABLE`; the
   off-screen Vulkan platform answering two new questions (no surface extensions, no surface); the Vulkan renderer
   compiled without RTTI (the release has no typeinfo for `VulkanPlatform`); and only the libraries the release ships
   linked (`vkshaders` is gone). The new build sits in its own folder, chosen per night by `HK_MUJOFIL_DIR`, which
   also enters the render stamp, so its baselines are rendered anew.
   Since 30 Sep 2026 it is the default and the old build is gone, broken on purpose so it can't be hit again: opt-in had
   let everything run without the variable (the showcase's stills and video, `night_tidy.sh`, the arm benches) fall back
   to the installed build, found again when the showcase's fan lamp and its light kit drew as one.
   `filament_cameras.mujofil_dir` picks `build/mujofil-f177-site` when nothing else is chosen and the render stamp
   always names the build; `check_lamps` renders two lamps on a bare floor before a process's first frame and stops with
   an error if either half stays dark, whatever build was imported; the 1.56.3 build was moved out of DimOS's venv to
   `build/attic/` (`tests/test_mujofil_default.py`).
8. [U8i] **Hard-won: Filament keeps released frame-graph textures 60 frames, not its default 1, or two cameras
   through one renderer re-make the shadow atlas every frame.** Found 18 Oct 2026 (Filament 1.77.1, Vulkan, RTX 2070
   8 GB): 3 of 5 sim arm nights on house 70 ended in `libc++abi: ... utils::PostconditionPanic` and nothing else. With
   the panic's reason logged (`tools/mujofil_panic_log.patch`: a panic on Filament's own threads otherwise reaches no
   handler), it was `VulkanTexture: Unable to allocate image memory. error=-2` (out of device memory) on the thread
   `FEngine::loop`, twice in two nights, both at a close look. The head and wrist cameras share one renderer ([Hsz]);
   the Go2 scene's own shadow-casting `spotlight` (45 degree cone from (3, 0, 4)) is in the head camera's view and
   not in the wrist's, so the shadow atlas, a frame-graph texture sized by the shadow lamps in view, alternated
   between 13 and 12 layers of 2048 x 2048 RG32F (554 and 512 MiB). Filament's texture cache keeps a released
   texture one frame, so each frame made a new atlas: a Vulkan layer logging `vkCreateImage` counted 159 atlases in
   600 frames of a look on the night's own model, and the sim process swung between 1.5 and 4.1 GB; in the nights,
   beside the tidy's recognizers (2.9 GB), the GPU's 8 GB ran out. Walk-only nights render the wrist once a second and
   load no close-look models, so they had the headroom. Kept 60 frames (`tools/mujofil_texture_cache.patch`,
   `MUJOFIL_TEXTURE_CACHE_FRAMES` overrides), each atlas is made once: 6 atlases in the same 600 frames, the process
   steady at 1.5 to 2.0 GB, and the frames pixel-identical to the old build's (120 head and wrist frames compared), so
   the render stamp and the baselines stand (`tests/test_filament_atlas_churn.py`, against a planted control with the
   cache at 1 frame). Threatens, if lost: any second camera or view through the renderer brings the churn back.

## Consequences

- Filament frames lack sensor effects: noise at high ISO, motion blur while walking, the Go2's equidistant lens
  distortion, H.264 compression. These belong in a camera model after the renderer, calibrated from real footage.
- The view-dependent carpet shading (`floors.view_from`) re-uploads textures to classic renderers only; under Filament
  the carpet's sheen doesn't change with the viewing angle (Filament's own materials could, later).
- Every result from before 25 Sep evening was on classic frames; results say which renderer made them.
- MuJoCo's own Filament (3.12+) replaces mujofil once its Python module ships in MuJoCo's wheels.

## Sources

- mujofil: <https://github.com/tau-intelligence/mujofil> (Apache-2.0), pyproject's note on vendored headers.
- MuJoCo 3.12.0 release and visualization docs (Filament integration, `MUJOCO_USE_FILAMENT`, `MUJOCO_USE_FILAMENT_MJR_COMPAT`):
  <https://github.com/google-deepmind/mujoco/releases/tag/3.12.0>, <https://mujoco.readthedocs.io/en/stable/programming/visualization.html>
- MuJoCo 3.10.0 changelog: <https://mujoco.readthedocs.io/en/3.10.0/changelog.html>
