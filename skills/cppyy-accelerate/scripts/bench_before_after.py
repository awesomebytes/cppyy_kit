#!/usr/bin/env python
"""
Timing helper for the cppyy-accelerate skill. It can time a function or a command.

As a library, it times a warmed operation, as in the walkthrough::

    from bench_before_after import compare
    compare([("naive Python", lambda: slow(pts, 0.05)),
             ("pcl_kit (C++)", lambda: fast(pts, 0.05))])

As a CLI, it measures whole-script wall time, including startup, on each run::

    python bench_before_after.py -n 5 \
        --before "python examples/accelerate_demo/slow_pointcloud_pipeline.py" \
        --after  "python examples/accelerate_demo/fast_pointcloud_pipeline.py"

The report uses the median. The first row is the baseline; later rows show speedup
against it. Check the target's tests as well as timing. The output must still match.
"""
import argparse
import shlex
import statistics
import subprocess
import sys
import time


def time_callable(fn, n=5, warmup=1):
    """Return the median wall time in milliseconds over ``n`` runs, after ``warmup``
    untimed runs. This keeps first-use JIT and cache costs out of the median."""
    for _ in range(warmup):
        fn()
    samples = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        samples.append((time.perf_counter() - t0) * 1000)
    return statistics.median(samples)


def compare(rows, n=5, warmup=1):
    """Time each ``(label, callable)`` row and print its median and speedup against
    the first row. Return ``[(label, median_ms), ...]``."""
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
