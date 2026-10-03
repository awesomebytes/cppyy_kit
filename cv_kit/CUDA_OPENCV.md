# CUDA-enabled OpenCV (`cv::cuda::ORB`) for the vision tutorial

The default `vision` environment uses CPU OpenCV; CUDA ORB requires an NVIDIA GPU
and compatible driver. Follow the [install and build steps](#install-and-validate) to enable and validate it.

---

## Install and validate

The files are written to `build/vendor/opencv-cuda/` (gitignored). Run these commands:

```bash
pixi install -e cudabuild
# download, verify (sha256), and extract the prebuilt libraries and headers
pixi run -e cudabuild provision-cuda-opencv

# compile and run the C++ cv::cuda::ORB smoke test and CPU-vs-CUDA FPS benchmark
pixi run -e cudabuild validate-cuda-opencv
```

Then run the tutorial with GPU OpenCV. Use the integrated `vision-cuda` env (it has
the vision stack **and** the CUDA 12.9 runtime in one process) and put the vendored
CUDA libs/headers first on cppyy's search path:

```bash
pixi install -e vision-cuda
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

## Package details (recorded July 2026)

The package search at that time found no CUDA builds on conda-forge and identified
the Esri OpenCV 4.13.0 build used by the provisioning script. These package and
feedstock findings are a dated snapshot; use the install steps above for the
repository's validated route.

### conda-forge and Esri packages

- The conda-forge OpenCV recipe set `WITH_CUDA=0`, `WITH_CUBLAS=0`, and
  `WITH_OPENCL=0`. The package listing search found 0 CUDA build strings across
  6,357 conda-forge `libopencv` files at the time of the search.
- A cross-channel sweep found older CUDA builds in `ab-geo` (4.8.0), `edj.david`
  (4.6.0), `rocketce` (ppc64le), and `sdy623` (win-64), as well as Esri. The
  selected package was **`Esri::libopencv`**, version **4.13.0**, build
  `cuda129_py313_4` (linux-64), with CUDA 12.9 and
  `libopencv_cudafeatures2d.so` for `cv::cuda::ORB`. It was uploaded on 2026-03-17.
- The package was Apache-2.0 and 86 MB (SHA-256
  `1a9a3286db27f75bc4d01e505cb8b39417f61b32121992c91466bfa97262b278`). It
  contains SASS through `sm_90` and `compute_50` PTX, but no native `sm_120` SASS.
- Its dependency pins did not co-solve with the vision environment's conda-forge
  packages. The provisioning task extracts its C++ libraries and headers and
  supplies the CUDA runtime packages the ORB path needs (`cudart`, `cublas`,
  `cufft`, and `npp`).

See the package and feedstock links under [Sources](#sources). The commands to
reproduce the package search are retained below.

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
