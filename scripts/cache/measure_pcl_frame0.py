#!/usr/bin/env python
"""
Measure pcl_kit's compile-cache adoption on the d02 showcase's frame-0
(cloud_from_msg -> voxel_downsample -> msg_from_cloud), in cold subprocesses.

  pixi run -e pcl python scripts/cache/measure_pcl_frame0.py

In the showcase the publisher builds the input PointCloud2 once at init (that pays
toROSMsg's first-use), so the first *processed* frame's cost is dominated by the
VoxelGrid template first-use -- exactly what the compile cache moves into the .so.
"JIT" forces the Python-driven VoxelGrid path (CPPYY_KIT_NO_CACHE=1); "cached" runs
the compiled voxel_downsample from the kit's .so (run 1 a miss, runs 2+ hits).
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

N_POINTS = 100_000
LEAF = 0.05


def _worker():
    import numpy as np
    import pcl_kit
    pcl_kit.bringup_pcl(with_ros=True)
    rng = np.random.default_rng(0)
    base = pcl_kit.cloud_from_numpy(rng.random((N_POINTS, 3), dtype=np.float32))
    in_msg = pcl_kit.msg_from_cloud(base)   # toROSMsg first-use lands here (init)

    t = {}

    def stage(name, fn):
        s = time.perf_counter()
        r = fn()
        t[name] = (time.perf_counter() - s) * 1000
        return r

    def frame():
        cloud = pcl_kit.cloud_from_msg(in_msg)
        out = pcl_kit.voxel_downsample(cloud, LEAF)
        pcl_kit.msg_from_cloud(out)
        return out

    out = stage("frame0", frame)
    print("JSON:" + json.dumps({"frame0_ms": t["frame0"], "out": int(out.size()),
                                "cached": pcl_kit._CACHED}))


def _run(no_cache):
    env = os.environ.copy()
    if no_cache:
        env["CPPYY_KIT_NO_CACHE"] = "1"
    else:
        env.pop("CPPYY_KIT_NO_CACHE", None)
    p = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker"],
                       capture_output=True, text=True, env=env)
    for line in p.stdout.splitlines():
        if line.startswith("JSON:"):
            return json.loads(line[5:])
    raise RuntimeError("worker failed:\n%s\n%s" % (p.stdout, p.stderr))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--worker", action="store_true")
    ap.add_argument("-n", "--runs", type=int, default=3)
    args = ap.parse_args()
    if args.worker:
        return _worker()

    import cppyy_kit
    print("pcl_kit d02 frame-0  (from_msg -> voxel -> to_msg, %d runs)\n" % args.runs)
    jit = [_run(no_cache=True) for _ in range(args.runs)]
    assert all(d["out"] == jit[0]["out"] for d in jit)
    print("JIT (Python VoxelGrid):   frame0 %.0f ms   out=%d   cached=%s"
          % (statistics.median(d["frame0_ms"] for d in jit), jit[0]["out"], jit[0]["cached"]))

    cppyy_kit.clear_cache()
    for i in range(1, args.runs + 1):
        d = _run(no_cache=False)
        assert d["cached"] is True and d["out"] == jit[0]["out"]
        note = "MISS compile" if i == 1 else "hit"
        print("cached run %d:            frame0 %.0f ms   out=%d   (%s)"
              % (i, d["frame0_ms"], d["out"], note))


if __name__ == "__main__":
    main()
