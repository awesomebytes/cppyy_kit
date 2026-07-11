#!/usr/bin/env python3
"""The differential contract for the accelerate walkthrough: the pcl_kit-accelerated
voxel downsampler must produce the *same* result as the naive pure-Python one.

This is the "tests-as-contract" gate the cppyy-accelerate skill's VERIFY step runs:
the acceleration is only valid if it preserves behaviour. The naive and PCL grids
group points identically (``floor(p / leaf)`` per axis), so we key both outputs by
voxel index and assert (a) the exact same occupied voxels and (b) matching centroids
(to float-summation precision). Gated on pcl (the pcl feature env)."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from slow_pointcloud_pipeline import make_cloud, voxel_downsample_slow  # noqa: E402

try:
    import pcl_kit
    pcl_kit._pcl_include_dir()          # raises if PCL headers aren't in this env
    _HAVE_PCL = True
except Exception:
    _HAVE_PCL = False

pytestmark = pytest.mark.skipif(not _HAVE_PCL, reason="pcl not installed (use the pcl env)")

LEAF = 0.05


def _keyed(arr, leaf):
    """{voxel index -> centroid}. A voxel's centroid lies inside the voxel, so
    floor(centroid / leaf) recovers its integer index for both implementations."""
    return {tuple(np.floor(row / leaf).astype(int)): row for row in arr}


@pytest.mark.parametrize("n", [5_000, 100_000])
def test_accelerated_matches_naive(n):
    from fast_pointcloud_pipeline import voxel_downsample_fast
    points = make_cloud(n)
    slow = _keyed(voxel_downsample_slow(points, LEAF), LEAF)
    fast = _keyed(voxel_downsample_fast(points, LEAF), LEAF)

    # (a) exactly the same occupied voxels
    assert set(slow) == set(fast)
    # (b) matching centroids (float-summation tolerance, not a grouping difference)
    max_diff = max(float(np.max(np.abs(slow[k] - fast[k]))) for k in slow)
    assert max_diff < 1e-2, "centroid mismatch %.4g" % max_diff


def test_downsample_actually_reduces():
    points = make_cloud(20_000)
    out = voxel_downsample_slow(points, LEAF)
    assert 0 < len(out) < len(points)
