# Walkthrough: speed up a point-cloud pipeline

This walkthrough applies `SKILL.md` to the Python voxel downsampler in
`examples/accelerate_demo/slow_pointcloud_pipeline.py`. The timings below were
measured on a shared machine. The command uses `pixi run -e pcl` with
`ROS_DOMAIN_ID` set.

## Step 1: Profile

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

Suggested next steps:
  * function with highest own time: voxel_downsample_slow (0.076 s own time over 1 calls).
    If this function processes array data in a loop, consider moving the loop to C++.
```

The per-point Python loop over array data accounts for most of the time. It runs
100,000 iterations. The `dict.get` line shows the per-point bucketing. The profile
records no cppyy_kit boundary crossings.

## Step 2: Choose a remedy

The hotspot is a *Python loop over point-cloud data*. Use **pcl_kit** to run the
voxel grid in C++ (`cloud_from_numpy` → `voxel_downsample` → `cloud_to_numpy`). This
loop runs for every point. PCL has no maintained Python binding.

## Step 3: Apply the change

The replacement in `examples/accelerate_demo/fast_pointcloud_pipeline.py` uses three
kit calls:

```python
def voxel_downsample_fast(points, leaf):
    cloud = pcl_kit.cloud_from_numpy(points)              # one memcpy into the C++ cloud
    downsampled = pcl_kit.voxel_downsample(cloud, leaf)   # compile-cached VoxelGrid (C++)
    return pcl_kit.cloud_to_numpy(downsampled)            # centroids back to (M,3) NumPy
```

The original loop was about 30 lines. `voxel_downsample` calls PCL's `VoxelGrid`,
compiled once into the kit's `.so` (COMMON_PATTERNS §23). It does not need a first-use
JIT warmup.

## Step 4: Verify and measure

**Contract** (`examples/accelerate_demo/test_pipeline.py`): the naive and PCL grids
group points identically (`floor(p / leaf)`), so the test keys both outputs by voxel
index and checks the occupied voxels and centroids. It allows small differences from
floating-point summation. Tests passed before and after the change:

```
test_accelerated_matches_naive[5000]    PASSED
test_accelerated_matches_naive[100000]  PASSED
test_downsample_actually_reduces        PASSED
```

**Timing** (`bench_before_after.compare`, 100k points, leaf 0.05, after warmup, median):

| variant | median | speedup |
|---|--:|--:|
| naive Python loop | 47.9 ms | 1.0× (base) |
| **pcl_kit (C++ VoxelGrid)** | **3.07 ms** | **15.6×** |

Both versions return the same 8,000 centroids and voxels. The pcl_kit version is
about 15.6× faster per call. One-time costs are separate: PCL startup takes about
1.3 s for header parsing, or about 6 ms when frozen. The compile cache's first `.so`
build takes about 3 s per machine. These costs occur at startup, not per frame. The
kit calls still add a few milliseconds of cppyy wrapper overhead.

## What this demonstrates

The example profiles the program, selects a kit, replaces the loop, checks the
output, and measures the speedup. The skill describes the same steps.
