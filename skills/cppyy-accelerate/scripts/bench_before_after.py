#!/usr/bin/env python
"""
VERIFY step helper for the cppyy-accelerate skill: a small timing harness for the
before/after table. Use it two ways.

As a library (per-operation timing, warmed -- what the walkthrough uses)::

    from bench_before_after import compare
    compare([("naive Python", lambda: slow(pts, 0.05)),
             ("pcl_kit (C++)", lambda: fast(pts, 0.05))])

As a CLI (whole-script wall time, cold per run -- includes bringup)::

    python bench_before_after.py -n 5 \
        --before "python examples/accelerate_demo/slow_pointcloud_pipeline.py" \
        --after  "python examples/accelerate_demo/fast_pointcloud_pipeline.py"

Report the median; the first row is the baseline and later rows show the speedup
against it. Always pair the number with the correctness gate (the target's tests) --
a faster result that fails the differential test is not an acceleration.
"""
import argparse
import shlex
import statistics
import subprocess
import sys
import time


def time_callable(fn, n=5, warmup=1):
    """Median wall time (ms) of ``fn()`` over ``n`` runs after ``warmup`` untimed
    runs (so a first-use JIT / cache miss doesn't skew the median)."""
    for _ in range(warmup):
        fn()
    samples = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        samples.append((time.perf_counter() - t0) * 1000)
    return statistics.median(samples)


def compare(rows, n=5, warmup=1):
    """``rows`` = list of ``(label, callable)``; time each and print a table with the
    speedup vs the first (baseline) row. Returns ``[(label, median_ms), ...]``."""
    results = [(label, time_callable(fn, n=n, warmup=warmup)) for label, fn in rows]
    base = results[0][1]
    width = max(len(label) for label, _ in results)
    print("%-*s %12s %10s" % (width, "variant", "median_ms", "speedup"))
    print("-" * (width + 24))
    for label, ms in results:
        sp = ("%.1fx" % (base / ms)) if ms > 0 else "-"
        print("%-*s %12.3f %10s" % (width, label, ms, sp if label != results[0][0] else "1.0x (base)"))
    return results


def _time_command(cmd, n):
    samples = []
    for _ in range(n):
        t0 = time.perf_counter()
        p = subprocess.run(shlex.split(cmd), capture_output=True, text=True)
        samples.append((time.perf_counter() - t0) * 1000)
        if p.returncode != 0:
            sys.exit("command failed: %s\n%s" % (cmd, p.stderr))
    return statistics.median(samples)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--before", required=True, help="baseline command")
    ap.add_argument("--after", required=True, help="accelerated command")
    ap.add_argument("-n", "--runs", type=int, default=5)
    args = ap.parse_args()
    before = _time_command(args.before, args.runs)
    after = _time_command(args.after, args.runs)
    print("%-14s %12.0f ms" % ("before", before))
    print("%-14s %12.0f ms   (%.1fx faster)" % ("after", after, before / after if after else 0))


if __name__ == "__main__":
    main()
