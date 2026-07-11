#!/usr/bin/env python
"""
A deliberately slow, pure-Python point-cloud voxel-grid downsampler -- the
"before" for the cppyy-accelerate skill walkthrough (skills/cppyy-accelerate/).

It does exactly what PCL's ``VoxelGrid`` does -- bucket points into leaf-sized
voxels and emit each occupied voxel's centroid -- but in a per-point Python loop,
which is where all the time goes. Profiling this (see the skill's PROFILE step)
points a coding agent straight at the loop, and the MAP step routes it to pcl_kit.

The downsample is a real, testable contract: ``voxel_downsample_slow`` and PCL's
VoxelGrid group points identically (``floor(p / leaf)`` per axis) and average each
group, so the accelerated version must produce the same centroid set -- that is the
differential test (test_pipeline.py) the accelerated code must still pass.

Run:  pixi run -e pcl python examples/accelerate_demo/slow_pointcloud_pipeline.py
"""
import argparse
import time

import numpy as np


def voxel_downsample_slow(points, leaf):
    """Naive voxel-grid downsample of an (N,3) float array. Returns the (M,3) array
    of occupied-voxel centroids. Pure Python loop over the N points -- the hot path."""
    voxels = {}
    inv = 1.0 / leaf
    for i in range(points.shape[0]):
        x = float(points[i, 0])
        y = float(points[i, 1])
        z = float(points[i, 2])
        # Group by the absolute voxel index floor(p / leaf) -- matches PCL VoxelGrid.
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
