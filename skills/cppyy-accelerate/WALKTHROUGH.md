# Walkthrough — accelerating a slow point-cloud pipeline

This is `SKILL.md` run end-to-end on a real case: the naive pure-Python voxel
downsampler in `examples/accelerate_demo/slow_pointcloud_pipeline.py`. It is both
the skill's validation (the numbers below are measured, not asserted) and the
ROSCon demo seed. Environment: `pixi run -e pcl`, `ROS_DOMAIN_ID` set, this machine
(shared; medians).

## Step 1 — PROFILE

```bash
pixi run -e pcl python skills/cppyy-accelerate/scripts/profile_target.py \
    examples/accelerate_demo/slow_pointcloud_pipeline.py -- -n 100000
```

```
Python hotspots -- by own time (tottime):
 tottime_s  cumtime_s    ncalls  function
--------------------------------------------------------------------------
     0.076      0.084         1  slow_pointcloud_pipeline.py:24(voxel_downsample_slow)
     0.009      0.009    100477  ~:0(<method 'get' of 'dict' objects>)
     ...
Python<->C++ boundary (cppyy_kit tracer):
  (no crossings recorded -- target does not use cppyy_kit yet ...)

VERDICT:
  * hottest pure-Python frame: voxel_downsample_slow -- 0.076 s own time over 1 calls.
    If it loops over C++-backed / array data, that is your MAP target.
```

One frame owns essentially all the time, and it is a **per-point Python loop over
array data** (100k iterations; the `dict.get` line confirms the per-point bucketing).
No boundary crossings — this is plain Python, the classic "before".

## Step 2 — MAP

Hotspot shape = *pure-Python loop over point-cloud data* → first row of the decision
tree → **pcl_kit**: do the voxel grid in C++ (`cloud_from_numpy` → `voxel_downsample`
→ `cloud_to_numpy`). Not a DON'T case: it's a hot per-point loop, not a one-shot
batch step, and PCL has no maintained Python binding.

## Step 3 — APPLY

The entire loop body collapses to three kit calls
(`examples/accelerate_demo/fast_pointcloud_pipeline.py`):

```python
def voxel_downsample_fast(points, leaf):
    cloud = pcl_kit.cloud_from_numpy(points)              # one memcpy into the C++ cloud
    downsampled = pcl_kit.voxel_downsample(cloud, leaf)   # compile-cached VoxelGrid (C++)
    return pcl_kit.cloud_to_numpy(downsampled)            # centroids back to (M,3) NumPy
```

vs the ~30-line naive loop it replaces. `voxel_downsample` runs PCL's `VoxelGrid`
compiled once into the kit's `.so` (COMMON_PATTERNS §23), so there is no first-use
JIT stall to warm.

## Step 4 — VERIFY

**Contract** (`examples/accelerate_demo/test_pipeline.py`): the naive and PCL grids
group points identically (`floor(p / leaf)`), so the test keys both outputs by voxel
index and asserts the same occupied voxels + matching centroids (only float-summation
drift allowed). Green before and after:

```
test_accelerated_matches_naive[5000]    PASSED
test_accelerated_matches_naive[100000]  PASSED
test_downsample_actually_reduces        PASSED
```

**Number** (`bench_before_after.compare`, 100k points, leaf 0.05, warmed, median):

| variant | median | speedup |
|---|--:|--:|
| naive Python loop | 47.9 ms | 1.0× (base) |
| **pcl_kit (C++ VoxelGrid)** | **3.07 ms** | **15.6×** |

Same output (8000 centroids, identical voxels), **~15.6× faster per call**. One-time
costs, amortized and reported separately: PCL bringup (~1.3 s header parse, or ~6 ms
frozen) and the compile cache's first-run `.so` build (~3 s, once per machine) — both
at init, not per frame. The honest residual after that is a few ms of cppyy call
wrappers to the kit entry points.

## What this demonstrates

An agent, given only "make this faster", profiled → mapped to the right kit →
applied a minimal diff → proved same-output at 15.6× via the tests-as-contract
discipline. That is the ROSCon story, and the loop this skill automates.
