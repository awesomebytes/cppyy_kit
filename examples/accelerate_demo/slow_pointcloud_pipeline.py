#!/usr/bin/env python
"""
A pure-Python point-cloud voxel-grid downsampler used as the "before" case in the
cppyy-accelerate walkthrough (skills/cppyy-accelerate/).

It follows the same method as PCL's ``VoxelGrid``: group points into leaf-sized
voxels and return each occupied voxel's centroid. It uses a Python loop over the
points. The walkthrough profiles this loop and uses pcl_kit to run it in C++.

The test checks that ``voxel_downsample_slow`` and PCL's VoxelGrid group points the
same way (``floor(p / leaf)`` per axis) and return the same centroids. The accelerated
version must pass this comparison in test_pipeline.py.

Run:  pixi run -e pcl python examples/accelerate_demo/slow_pointcloud_pipeline.py
"""
import argparse
import time

import numpy as np


def voxel_downsample_slow(points, leaf):
    """Downsample an (N,3) float array and return an (M,3) array of occupied-voxel
    centroids. Uses a Python loop over the points."""
    voxels = {}
    inv = 1.0 / leaf
    for i in range(points.shape[0]):
        x = float(points[i, 0])
        y = float(points[i, 1])
        z = float(points[i, 2])
        # Group by the absolute voxel index floor(p / leaf), as in PCL VoxelGrid.
        key = (int(np.floor(x * inv)), int(np.floor(y * inv)), int(np.floor(z * inv)))
        acc = voxels.get(key)
        if acc is None:
            voxels[key] = [x, y, z, 1]
        else:
            acc[0] += x
            acc[1] += y
            acc[2] += z
            acc[3] += 1
    out = np.empty((len(voxels), 3), dtype=np.float32)
    for i, acc in enumerate(voxels.values()):
        n = acc[3]
        out[i, 0] = acc[0] / n
        out[i, 1] = acc[1] / n
        out[i, 2] = acc[2] / n
    return out


def make_cloud(n=100_000, seed=0):
    """A reproducible (n,3) float32 cloud in a 1 m cube."""
    return np.random.default_rng(seed).random((n, 3), dtype=np.float32)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-n", "--points", type=int, default=100_000)
    ap.add_argument("--leaf", type=float, default=0.05)
    args = ap.parse_args()

    points = make_cloud(args.points)
    t0 = time.perf_counter()
    out = voxel_downsample_slow(points, args.leaf)
    dt = (time.perf_counter() - t0) * 1000
    print("voxel downsample (pure Python): %d -> %d points in %.1f ms"
          % (len(points), len(out), dt))


if __name__ == "__main__":
    main()
