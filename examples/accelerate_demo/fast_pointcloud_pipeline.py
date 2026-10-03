#!/usr/bin/env python
"""
This is the "after" case in the cppyy-accelerate walkthrough. It uses pcl_kit to
speed up the voxel downsampler in slow_pointcloud_pipeline.py.

Three pcl_kit calls copy the array into a PCL cloud, run the compile-cached
``VoxelGrid`` in C++, then copy the centroids back. The result uses the same voxel
grouping and centroids as the naive version. test_pipeline.py checks the output.
WALKTHROUGH.md reports the measured timings.

Run:  pixi run -e pcl python examples/accelerate_demo/fast_pointcloud_pipeline.py
"""
import argparse
import time

import pcl_kit

from slow_pointcloud_pipeline import make_cloud


def voxel_downsample_fast(points, leaf):
    """Use the same input and output shapes as ``voxel_downsample_slow``. pcl_kit
    runs the voxel grid in C++ without a Python loop over the points."""
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
