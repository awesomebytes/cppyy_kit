#!/usr/bin/env python
"""
The "after" for the cppyy-accelerate walkthrough: the naive voxel downsampler
(slow_pointcloud_pipeline.py) accelerated with pcl_kit, produced by following
skills/cppyy-accelerate/SKILL.md (PROFILE -> MAP -> APPLY -> VERIFY).

The whole per-point Python loop collapses to three pcl_kit calls -- copy the array
into a PCL cloud, run the compile-cached ``VoxelGrid`` (C++), copy the centroids
back -- so the work happens in C++ and the Python side only orchestrates. Same
voxel grouping and centroids as the naive version (test_pipeline.py is the
contract), at a fraction of the time (WALKTHROUGH.md has the measured table).

Run:  pixi run -e pcl python examples/accelerate_demo/fast_pointcloud_pipeline.py
"""
import argparse
import time

import pcl_kit

from slow_pointcloud_pipeline import make_cloud


def voxel_downsample_fast(points, leaf):
    """Same contract as ``voxel_downsample_slow`` -- (N,3) in, (M,3) centroids out
    -- but the voxel grid runs in C++ via pcl_kit (no per-point Python)."""
    cloud = pcl_kit.cloud_from_numpy(points)              # one memcpy into the C++ cloud
    downsampled = pcl_kit.voxel_downsample(cloud, leaf)   # compile-cached VoxelGrid (C++)
    return pcl_kit.cloud_to_numpy(downsampled)            # centroids back to (M,3) NumPy


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-n", "--points", type=int, default=100_000)
    ap.add_argument("--leaf", type=float, default=0.05)
    args = ap.parse_args()

    points = make_cloud(args.points)
    pcl_kit.bringup_pcl(with_ros=False)          # bring PCL up before timing the op
    t0 = time.perf_counter()
    out = voxel_downsample_fast(points, args.leaf)
    dt = (time.perf_counter() - t0) * 1000
    print("voxel downsample (pcl_kit / C++): %d -> %d points in %.1f ms"
          % (len(points), len(out), dt))


if __name__ == "__main__":
    main()
