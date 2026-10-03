# CUDA-enabled OpenCV (`cv::cuda::ORB`) for the vision tutorial

This guide covers GPU-enabled `cv::cuda::ORB` for the vision tutorial. It describes
the Esri OpenCV package, how it works with the CPU OpenCV package, and how to
provision and validate it. Testing used OpenCV 4.13.0 on an RTX PRO 2000 Blackwell
GPU (sm_120).

---

## Package availability

The package search found no CUDA builds on conda-forge. It found a CUDA-enabled
OpenCV 4.13.0 package in the Esri channel.

### conda-forge builds OpenCV without CUDA
- The recipe sets these build options: conda-forge/opencv-feedstock `recipe/build.sh` sets
  `-DWITH_CUDA=0 -DWITH_CUBLAS=0 -DWITH_OPENCL=0`.
- The build configuration has no CUDA variant: `recipe/conda_build_config.yaml` has only `qt_version`
  (`none`/`6`) and an macOS SDK key, with no `cuda`, `cuda_compiler`, or
  `cuda_compiler_version`.
- The package listings show: The anaconda.org API lists **6357** `conda-forge/libopencv`
  files across every version/platform; **0** carry a `cuda` build string. Same for
  `conda-forge/opencv`. (Latest is 4.13.0 `qt6_py312..._610`, which is what
  `feature.vision` pins.)
- The package configuration has remained CPU-only: Feedstock issues
  [#74](https://github.com/conda-forge/opencv-feedstock/issues/74) (2017) and
  [#109](https://github.com/conda-forge/opencv-feedstock/issues/109) (2018) request
  CUDA builds; they were declined (CI/binary-size/licensing). Builds remain CPU-only.

### CUDA-enabled OpenCV package in the Esri channel
A cross-channel sweep (anaconda.org `search` API over every channel shipping
`opencv`/`libopencv`, then a per-channel scan of build strings for `cuda`/`gpu`)
found CUDA builds on: `ab-geo` (4.8.0, CUDA 11.8, old), `edj.david` (4.6.0, old),
`rocketce` (ppc64le only), `sdy623` (win-64 only), and `Esri`. The selected package:

| Field | Value |
|---|---|
| Package | **`Esri::libopencv`** |
| Version / build | **`4.13.0` / `cuda129_py313_4`** (linux-64) |
| Matches vision env? | **Yes, OpenCV 4.13.0** (conda-forge pins the same) |
| CUDA | 12.9 (cudart/cublas/cufft/npp 12.9; cudnn 9.10). CUDA >=12.8 supports Blackwell sm_120 |
| License | **Apache-2.0** (OpenCV's own license) |
| Public? | **Yes** (`public: true`); uploaded 2026-03-17 |
| Size / sha256 | 86 MB / `1a9a3286db27f75bc4d01e505cb8b39417f61b32121992c91466bfa97262b278` |
| Modules | Full contrib CUDA set incl. `libopencv_cudafeatures2d.so` (`cv::cuda::ORB`), `cudaarithm`, `cudawarping`, `cudafilters`, `cudaimgproc`, `cudaoptflow`, `cudastereo`, ... |
| GPU code | SASS `sm_50..sm_90` + **`compute_50` PTX** (no native sm_120 -- see below) |

**Why not install it directly with conda:** the package's dependency pins
(`gstreamer >=1.24.12,<1.25`, plus `ffmpeg 8`, `hdf5 1.14.5`, `cudnn`, `cusparselt`,
`cudss`, `cufile`) no longer co-solve against *current* conda-forge, and
`cv::cuda::ORB` needs none of them (those are `videoio`/`highgui`/`hdf`/`dnn`
modules). The setup extracts the C++ shared libraries and headers and supplies the
CUDA 12.9 runtime the ORB path actually links (`cudart`, `cublas`, `cufft`, `npp`)
from conda-forge. This is what the `cudabuild` pixi feature + provisioning script do.

---

## Install and validate

The files are written to `build/vendor/opencv-cuda/` (gitignored). Run these commands:

```bash
# 1. download + verify(sha256) + extract the prebuilt's libs & headers
pixi run -e cudabuild provision-cuda-opencv

# 2. compile + run the C++ cv::cuda::ORB smoke test and CPU-vs-CUDA fps bench
pixi run -e cudabuild validate-cuda-opencv
```

Then run the tutorial with GPU OpenCV. Use the integrated `vision-cuda` env (it has
the vision stack **and** the CUDA 12.9 runtime in one process) and put the vendored
CUDA libs/headers first on cppyy's search path:

```bash
export OPENCV_CUDA_ROOT="$PWD/build/vendor/opencv-cuda"
# CUDA OpenCV FIRST so every libopencv_*.so.413 soname resolves to the CUDA build:
export LD_LIBRARY_PATH="$OPENCV_CUDA_ROOT/lib:$LD_LIBRARY_PATH"
export CPLUS_INCLUDE_PATH="$OPENCV_CUDA_ROOT/include/opencv4:$CPLUS_INCLUDE_PATH"
pixi run -e vision-cuda demo-vision-features
```

cv_kit auto-detects at bringup: with the CUDA libs first on the path and the runtime
present, `cv::cuda::getCudaEnabledDeviceCount()` returns >=1 and it takes the GPU
path; otherwise it falls back to CPU `cv::ORB` unchanged. To confirm detection
through the exact cppyy path cv_kit uses:

```bash
pixi run -e vision-cuda python cv_kit/cpp/build_opencv_cuda.py validate-cppyy
```

---

## Use one OpenCV build per process

The `vision`/`vision-cuda` env's conda-forge OpenCV and the vendored CUDA OpenCV are
**both version 4.13.0 and share their sonames** (`libopencv_core.so.413`, `..._features2d.so.413`,
etc.). **Loading both `libopencv_core` variants in one process can cause a crash or other incorrect behavior**
(duplicate globals/registries -> segfault or silent misbehaviour).

Use one OpenCV build per process. Set library search paths so every OpenCV library
comes from the same build:

- **Put `build/vendor/opencv-cuda/lib` FIRST on `LD_LIBRARY_PATH`.** cppyy resolves
  C++ symbols by scanning `LD_LIBRARY_PATH` for the owning `.so`, so with the CUDA dir
  first, every `libopencv_*.so.413` binds to the CUDA build and the env's CPU OpenCV
  is hidden by the CUDA build. This is one coherent 4.13.0 set (same headers, same ABI), so
  `core`/`imgproc`/`features2d` calls stay correct while `cuda*` modules become available.
- **Also put the CUDA headers first** (`CPLUS_INCLUDE_PATH`), so cling finds
  `opencv2/cudafeatures2d.hpp` (absent from the CPU package) and parses the *matching*
  4.13.0 core headers.
- **Do not** add the CUDA libs to `LD_LIBRARY_PATH` *after* the env lib dir and expect
  it to work -- the env's CPU `core` would load first and the CUDA modules would then
  bind against a second `core`. First, or not at all.

Runtime source options:
- **`vision-cuda` (recommended):** CUDA 12.9 runtime is in the same env; only prepend
  the vendored `lib` dir.
- **`vision` + standalone `cudabuild`:** prepend BOTH `build/vendor/opencv-cuda/lib`
  and `.pixi/envs/cudabuild/lib` (the latter has cudart/cublas/cufft/npp).

---

## Validation results (this machine)

RTX PRO 2000 Blackwell Laptop GPU, **compute cap 12.0 (sm_120)**, 8 GB, driver
580.159.03 (CUDA 13.0). 640x480, N=2000 features, 50-iter mean. Machine was shared
(short window) -- treat fps as indicative, the ratio as the signal.

**C++** (`validate`):
```
getCudaEnabledDeviceCount() = 1
device 0: NVIDIA RTX PRO 2000 Blackwell ... sm_120, Driver/Runtime 13.0/12.90
CPU  cv::ORB      : 2000 kp, desc 2000x32 CV_8U, 8.27 ms, 120.9 fps
CUDA warmup (PTX->sm_120 JIT, first run): 2312 ms   # 19 ms once ~/.nv cache is warm
CUDA cv::cuda::ORB: desc 2000x32 CV_8U, 1.77 ms, 564.3 fps
SPEEDUP = 4.67x
```

**cppyy** (`validate-cppyy`, the cv_kit path): `getCudaEnabledDeviceCount()=1`,
descriptors `1965x32 CV_8U`, `526.9 fps`. PASS.

The test produced **Nx32 `CV_8U`** and the GPU path is ~4.7-4.9x
the CPU path here.

### Blackwell (sm_120)
The prebuilt has **no native sm_120 SASS** (top SASS is sm_90) but **does embed
`compute_50` PTX**. PTX is forward-compatible: the driver (CUDA 13.0, sm_120-aware)
**JIT-compiles the PTX to sm_120 at first kernel launch**. Cost: a one-time ~2.3 s
warmup per fresh machine, then cached in `~/.nv/ComputeCache` (subsequent starts
~19 ms). The tested kernels ran correctly. The package is not tuned for Blackwell-specific instructions. If you
want native sm_120 SASS (no JIT warmup, potentially faster), use the source build.

---

## Build from source

Use this if the prebuilt is unavailable, you need native sm_120 SASS, or you distrust
a third-party channel. Needs the CUDA *toolchain* (`cuda-nvcc`, `cuda-cudart-dev`,
`libcublas-dev`, `libnpp-dev`, `libcufft-dev` @ 12.9) plus `cmake`/`ninja`:

```bash
pixi run -e cudabuild python cv_kit/cpp/build_opencv_cuda.py build-from-source
```

It fetches OpenCV + opencv_contrib **4.13.0** (matching the env), configures with
`WITH_CUDA=ON`, `BUILD_LIST=core,imgproc,features2d,flann,cuda{arithm,warping,filters,imgproc,features2d}`,
python OFF, `CUDA_ARCH_BIN=7.5;8.0;8.6;8.9;9.0;12.0` (includes **sm_120**) +
`CUDA_ARCH_PTX=12.0`, and installs into `build/vendor/opencv-cuda/` -- the same
layout the prebuilt uses, so the consume/coexistence steps above are identical.
Expect ~30-90 min on 16 cores. (Not exercised here because the prebuilt validated.)

---

## Reproduce the package search

```bash
# conda-forge has 0 cuda libopencv builds (of thousands):
curl -s https://api.anaconda.org/package/conda-forge/libopencv \
  | jq -r '.files[] | select(.attrs.build|test("cuda";"i")) | .attrs.build'   # -> (empty)
# the recipe disables CUDA:
curl -s https://raw.githubusercontent.com/conda-forge/opencv-feedstock/main/recipe/build.sh \
  | grep -i 'WITH_CUDA\|WITH_CUBLAS'                                           # -> =0
# the Esri prebuilt exists, linux-64, 4.13.0, cuda129:
curl -s https://api.anaconda.org/package/Esri/libopencv \
  | jq -r '.files[] | select(.attrs.subdir=="linux-64" and (.attrs.build|test("cuda129"))) | "\(.version) \(.attrs.build)"'
```

## Sources
- conda-forge opencv-feedstock recipe: <https://github.com/conda-forge/opencv-feedstock>
- Feedstock CUDA requests: [#74](https://github.com/conda-forge/opencv-feedstock/issues/74), [#109](https://github.com/conda-forge/opencv-feedstock/issues/109)
- Esri libopencv (anaconda.org): <https://anaconda.org/Esri/libopencv>
- CUDA GPU compute capabilities (sm_120 = Blackwell): <https://developer.nvidia.com/cuda-gpus>
