# cv_kit — SKILL (seed)

> Seed cheat sheet. A full LLM-facing SKILL.md (when-to-use / copy-paste
> patterns / gotchas) is a planned deliverable (tracked in the project plan:
> "SKILL.md for every kit"). For now this points at the authoritative sources.

**What:** drive OpenCV's C++ API (core / imgproc / features2d) from Python via
cppyy, with a **zero-copy** bridge from a ROS 2 `sensor_msgs/Image` (C++
message) into `cv::Mat`. Pairs with [`dbow_kit`](../dbow_kit/) for loop closure.

**Why (not cv2):** composition. A `cv::Mat` can alias a C++ message's `data`
buffer with no copy, run C++ `cv::ORB`, and hand descriptors straight to DBoW2 —
the whole vision front-end stays in one C++ address space, Python only
orchestrates. See [`WHY.md`](WHY.md).

**Bring up:**
```python
import cv_kit
cv = cv_kit.bringup_cv()          # JIT-includes opencv4, loads libopencv_*.so
orb = cv_kit.create_orb(500)      # CUDA auto-detected; CPU cv::ORB otherwise
mat = cv_kit.msg_to_mat(image)    # zero-copy view over the message's data buffer
```

**Footgun (dangling Mat):** `msg_to_mat` / `mat_to_numpy(copy=False)` return
views that ALIAS C++/message storage — keep the backing object alive while you
use the view (use the Mat inside the callback that owns the message).

**Evidence & CUDA:** [`REPORT.md`](REPORT.md) (probe matrix + benchmarks) and
[`CUDA_OPENCV.md`](CUDA_OPENCV.md) (the conda-forge-has-no-CUDA verdict and the
vendored Esri prebuilt route). The end-to-end story is the tutorial:
[`docs/tutorials/vision_loop_closure.md`](../docs/tutorials/vision_loop_closure.md).

**Demos:** `cv_kit/demos/` (`demo_spine`, `demo_features`, `demo_loop`,
`demo_posegraph`, `bench_vision`). **CUDA build:** `cv_kit/cpp/build_opencv_cuda.py`.
