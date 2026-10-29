#!/usr/bin/env bash
# Build mujofil (Filament rendering for MuJoCo, Apache-2.0) against the MuJoCo version DimOS's venv runs, and install it
# there. Upstream's wheels are built against MuJoCo 3.9 and read MuJoCo's model structs at compile-time offsets, so a
# different MuJoCo needs a rebuild with that version's headers ("re-vendor the headers", per their pyproject); nothing
# in mujofil's code changes. Checked 2026-09-25: mujofil 0.2.6 @ 339a0fa against MuJoCo 3.10.0 renders pixel-identical
# to the official 3.9 wheel.
# The toolchain (clang + libc++, CUDA headers and static runtime, lld, zlib) comes from conda-forge via micromamba into
# BUILD/mamba/toolchain: no sudo, nothing installed system-wide.
# Our one change to mujofil's rendering (tools/mujofil_pbr_maps.patch, tools/mujofil_materials/textured_pbr.mat): MuJoCo's
# normal and ORM material layers reach Filament (upstream reads only the colour map), and textures are sampled with 8x
# anisotropic filtering (floors seen at a grazing angle otherwise smear). The material is compiled with Filament 1.56.3's
# own matc: mujofil links that Filament, and a material from another version's matc won't load (material version 56).
# Usage: tools/build_mujofil.sh   (env: BUILD=<workspace>/build, DIMOS_PY=<workspace>/dimos/.venv/bin/python,
#        INSTALL=0 builds the wheel into $BUILD/$SRC/dist without installing it; SRC=mujofil the source folder under
#        $BUILD; FILAMENT_DIR a Filament to link, e.g. $BUILD/filament-1.56.3/filament, Google's own release (unset:
#        mujofil's CMake downloads its authors' forked 1.56.3); TARGET a folder to install into instead of DimOS's venv,
#        chosen at run time by HK_MUJOFIL_DIR (filament_cameras.py), so a new build sits beside the one nights use;
#        FILAMENT_TAG the git tag of FILAMENT_DIR's release, whose source headers the Vulkan renderer needs)
set -euo pipefail
HK=$(cd "$(dirname "$0")/.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
BUILD=${BUILD:-$WS/build}; DIMOS_PY=${DIMOS_PY:-$DIMOS/.venv/bin/python}
COMMIT=339a0fa10617e6d3af17305fab0efca75dd11fac          # mujofil 0.2.6
mkdir -p "$BUILD"; cd "$BUILD"
# 1. toolchain
MM=$BUILD/mamba/bin/micromamba; T=$BUILD/mamba/toolchain; export MAMBA_ROOT_PREFIX=$BUILD/mamba/root
[ -x "$MM" ] || (mkdir -p mamba && cd mamba && curl -sL https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj bin/micromamba)
[ -x "$T/bin/clang++" ] || "$MM" create -y -q -p "$T" -c conda-forge clang clangxx libcxx libcxx-devel lld zlib \
    "cuda-nvcc=12.*" "cuda-cudart-dev=12.*" "cuda-cudart-static=12.*" "cuda-version=12.*" libgl-devel libegl-devel cmake ninja
# 2. source at the pinned commit, with the MuJoCo headers of the version DimOS runs
SRC=${SRC:-mujofil}
[ -d "$SRC/.git" ] || git clone -q https://github.com/tau-intelligence/mujofil.git "$SRC"
cd "$SRC" && git fetch -q origin && git checkout -q "$COMMIT" && git checkout -q -- . && git clean -qfd third_party
MJ_INC=$("$DIMOS_PY" -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))")/include
MJ_VER=$("$DIMOS_PY" -c "import mujoco; print(mujoco.__version__)")
rm -rf third_party/mujoco/include && cp -r "$MJ_INC" third_party/mujoco/include
git apply "$HK/tools/mujofil_mujoco310.patch"                     # the pins: mujoco 3.10.x, mujoco-warp dropped (GPU physics only)
git apply "$HK/tools/mujofil_pbr_maps.patch"                      # normal + ORM layers, anisotropic filtering, lamp shadows, SSAO/shadow options
git apply "$HK/tools/mujofil_gl_exposure.patch"                   # set_exposure on the OpenGL renderer too (its module wouldn't load)
git apply "$HK/tools/mujofil_panic_log.patch"                     # a Filament panic's reason on stderr, from any thread
git apply "$HK/tools/mujofil_texture_cache.patch"                 # frame-graph textures kept 60 frames: no per-frame atlas churn
if [ -z "${FILAMENT_DIR:-}" ]; then                                # mujofil's own Filament (its authors' fork of 1.56.3):
    FIL=$BUILD/filament-1.56.3; MATC_SHA=47be12cb30346378d17df47111d956eab9282d915d9a606552326d73b09e19c8   # Google's matc
    [ -x "$FIL/filament/bin/matc" ] || (mkdir -p "$FIL" && cd "$FIL" && curl -sfL -m 600 -o f.tgz \
        https://github.com/google/filament/releases/download/v1.56.3/filament-v1.56.3-linux.tgz && tar xzf f.tgz && rm f.tgz)
    [ "$(sha256sum "$FIL/filament/bin/matc" | cut -c1-64)" = "$MATC_SHA" ] || { echo "matc hash changed" >&2; exit 1; }
    MATC=$FIL/filament/bin/matc
else                                                               # another Filament: every material rebuilt with its own
    git apply "$HK/tools/mujofil_filament_new.patch"               # (and cubemaps uploaded as 6 layers: FaceOffsets is gone)
    : "${FILAMENT_TAG:?FILAMENT_TAG (e.g. v1.77.1) must name the release in FILAMENT_DIR}"
    FS=$BUILD/filament-src-$FILAMENT_TAG                           # the Vulkan renderer builds against Filament source
    [ -d "$FS/libs/utils/include" ] || (rm -rf "$FS" && git clone -q --depth 1 --filter=blob:none --sparse \
        --branch "$FILAMENT_TAG" https://github.com/google/filament.git "$FS" && cd "$FS" && \
        git sparse-checkout set libs/utils/include libs/bluevk/include)   # headers the release doesn't ship; mujofil
    rm -rf third_party/utils_include third_party/bluevk/include    # keeps 1.56.3's, which shadow the new release's
    cp -r "$FS/libs/utils/include" third_party/utils_include; cp -r "$FS/libs/bluevk/include" third_party/bluevk/include
    MATC=$FILAMENT_DIR/bin/matc                                    # matc (a material loads only in its matc's version),
    for f in materials_layered_src/*.mat; do                       # from the sources mujofil keeps for its layered set
        "$MATC" -a opengl -a vulkan -p desktop -o "mujofil/materials/$(basename "${f%.mat}").filamat" "$f"
    done
fi
"$MATC" -a opengl -a vulkan -p desktop -o mujofil/materials/textured_pbr.filamat "$HK/tools/mujofil_materials/textured_pbr.mat"
# 3. build the wheel (the build venv holds scikit-build-core and pybind11)
[ -x "$BUILD/mjfil-build/bin/python" ] || python3 -m venv "$BUILD/mjfil-build"
"$BUILD/mjfil-build/bin/pip" install -q -U pip scikit-build-core pybind11
CT=$T/targets/x86_64-linux; PB=$("$BUILD/mjfil-build/bin/python" -m pybind11 --cmakedir)
# libc++ linked in statically with its symbols hidden: other wheels bring their own older libc++.so.1 (open3d, which DimOS's
# sim imports first), and a shared one would resolve to theirs ("undefined symbol ...__hash_memory", 25 Sep)
# mujofil's CMake links -lc++ -lc++abi itself, so a folder holding only the static archives goes first on the search path
mkdir -p "$BUILD/libcxx-static" && ln -sf "$T/lib/libc++.a" "$T/lib/libc++abi.a" "$BUILD/libcxx-static/"
L="-fuse-ld=lld -L$BUILD/libcxx-static -L$T/lib -Wl,-rpath,$T/lib -Wl,--exclude-libs,ALL"
export PATH=$T/bin:$PATH CC=$T/bin/clang CXX=$T/bin/clang++ LDFLAGS="$L"
# -Wno-invalid-specialization: newer clang rejects Filament 1.56.3's std trait specialisations for its half type;
# -idirafter /usr/include: Ubuntu's Python header includes its multiarch pyconfig.h from there; lld: conda's ld can't
# read the system glibc
export CMAKE_ARGS="-DCUDAToolkit_ROOT=$T -DCUDA_INCLUDE_DIRS=$CT/include -D_cudart_static=$CT/lib/libcudart_static.a \
 -D_culibos=$CT/lib/libculibos.a -Dpybind11_DIR=$PB -DPython_EXECUTABLE=$BUILD/mjfil-build/bin/python -DCMAKE_PREFIX_PATH=$T \
 '-DCMAKE_CXX_FLAGS=-Wno-invalid-specialization -idirafter /usr/include' '-DCMAKE_EXE_LINKER_FLAGS=$L' \
 '-DCMAKE_SHARED_LINKER_FLAGS=$L' '-DCMAKE_MODULE_LINKER_FLAGS=$L'"
rm -rf dist build && "$BUILD/mjfil-build/bin/pip" wheel . --no-deps --no-build-isolation -w dist > build.log 2>&1 || { tail -20 build.log; exit 1; }
# 4. into DimOS's venv without touching anything else there
[ "${INSTALL:-1}" = 1 ] || { echo "built $(ls dist/mujofil-*.whl) (not installed)"; exit 0; }
if [ -n "${TARGET:-}" ]; then
    rm -rf "$TARGET" && uv pip install --python "$DIMOS_PY" --no-deps --target "$TARGET" dist/mujofil-*.whl
    echo "installed into $TARGET (use with HK_MUJOFIL_DIR=$TARGET)"; exit 0
fi
uv pip install --python "$DIMOS_PY" --no-deps --reinstall dist/mujofil-*.whl
"$DIMOS_PY" -c "import mujoco, mujofil; print('mujofil built against MuJoCo', mujoco.__version__)" 2>&1 | grep -v "^Warp" | tail -1
echo "built for MuJoCo $MJ_VER"
