#!/usr/bin/env python
"""
Reproduce the accelerate walkthrough's before/after table: naive Python voxel
downsample vs the pcl_kit-accelerated version, same input, warmed medians.

Run:  pixi run -e pcl python examples/accelerate_demo/bench.py [-n N] [--leaf L]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "skills", "cppyy-accelerate", "scripts"))

import pcl_kit  # noqa: E402
from bench_before_after import compare  # noqa: E402
from slow_pointcloud_pipeline import make_cloud, voxel_downsample_slow  # noqa: E402
from fast_pointcloud_pipeline import voxel_downsample_fast  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-n", "--points", type=int, default=100_000)
    ap.add_argument("--leaf", type=float, default=0.05)
    args = ap.parse_args()

    pcl_kit.bringup_pcl(with_ros=False)      # bring PCL up (+ build/load the cache) first
    points = make_cloud(args.points)
    print("voxel downsample, n=%d, leaf=%.3f (warmed medians):" % (args.points, args.leaf))
    compare([("naive Python loop", lambda: voxel_downsample_slow(points, args.leaf)),
             ("pcl_kit (C++ VoxelGrid)", lambda: voxel_downsample_fast(points, args.leaf))],
            n=5, warmup=2)


if __name__ == "__main__":
    main()
