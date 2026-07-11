#!/usr/bin/env python
"""
PROFILE step of the cppyy-accelerate skill: run a target script under BOTH a Python
profiler (cProfile) and the cppyy_kit boundary tracer, and print one combined
hotspot report -- where Python time goes, and what crossed the Python<->C++ boundary
(with the C++ signatures and their cost). That report is the input to the MAP step.

    python skills/cppyy-accelerate/scripts/profile_target.py \
        examples/accelerate_demo/slow_pointcloud_pipeline.py -- -n 100000

Everything after ``--`` is passed to the target as its argv. The target runs in this
process so cProfile sees its calls and the tracer (if the target uses cppyy_kit)
records its crossings. Read the two tables together: a fat pure-Python function that
dominates tottime is a MAP candidate; a boundary line with high total_ms is a
first-use JIT / crossing cost the cache or a bulk pattern addresses.
"""
import argparse
import cProfile
import io
import os
import pstats
import runpy
import sys
import time


def _split_argv(argv):
    if "--" in argv:
        i = argv.index("--")
        return argv[:i], argv[i + 1:]
    return argv[:1], argv[1:]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", help="path to the target .py script")
    ap.add_argument("--top", type=int, default=12, help="rows per table (default 12)")
    own, target_argv = _split_argv(sys.argv[1:])
    args = ap.parse_args(own)

    target = os.path.abspath(args.target)
    if not os.path.isfile(target):
        sys.exit("no such target script: %s" % target)

    # Best-effort boundary trace: only if cppyy_kit is importable (the target may be
    # plain Python that we're about to accelerate -- then there's simply no trace).
    trace = None
    try:
        from cppyy_kit import trace as _trace
        trace = _trace
        trace.start()
    except Exception:
        pass

    sys.argv = [target] + target_argv
    pr = cProfile.Profile()
    wall0 = time.perf_counter()
    pr.enable()
    try:
        runpy.run_path(target, run_name="__main__")
    finally:
        pr.disable()
        wall = (time.perf_counter() - wall0) * 1000
        manifest = trace.stop() if trace and trace.enabled() else None

    print("\n" + "=" * 74)
    print("PROFILE  %s   (wall %.0f ms)" % (os.path.basename(target), wall))
    print("=" * 74)

    _print_pstats(pr, args.top)
    _print_trace(manifest)
    _print_verdict(pr, manifest)


def _stats_rows(pr, sort_key, top):
    buf = io.StringIO()
    st = pstats.Stats(pr, stream=buf).sort_stats(sort_key)
    rows = []
    for func, (cc, nc, tt, ct, _callers) in st.stats.items():
        rows.append((tt, ct, nc, "%s:%d(%s)" % (os.path.basename(func[0]), func[1], func[2])))
    rows.sort(key=lambda r: -(r[0] if sort_key == "tottime" else r[1]))
    return rows[:top]


def _print_pstats(pr, top):
    print("\nPython hotspots -- by own time (tottime):")
    print("%10s %10s %9s  %s" % ("tottime_s", "cumtime_s", "ncalls", "function"))
    print("-" * 74)
    for tt, ct, nc, name in _stats_rows(pr, "tottime", top):
        print("%10.3f %10.3f %9d  %s" % (tt, ct, nc, name))


def _print_trace(manifest):
    print("\nPython<->C++ boundary (cppyy_kit tracer):")
    if not manifest:
        print("  (no crossings recorded -- target does not use cppyy_kit yet, or the")
        print("   cppyy_kit env is not active. This is expected for a plain-Python 'before'.)")
        return
    s = manifest.get("summary", {})
    by_kind = s.get("by_kind", {})
    if by_kind:
        print("  %-18s %6s %11s" % ("crossing", "count", "total_ms"))
        for kind, info in sorted(by_kind.items(), key=lambda kv: -kv[1]["total_ms"]):
            print("  %-18s %6d %11.1f" % (kind, info["count"], info["total_ms"]))
    inst = manifest.get("instantiations", [])
    if inst:
        print("  instantiation manifest (C++ signatures crossed, by cost):")
        for row in inst[:8]:
            print("    %8.1f ms  x%-4d %s" % (row["total_ms"], row["count"], row["signature"]))


def _print_verdict(pr, manifest):
    print("\nVERDICT (feed to the MAP step):")
    rows = _stats_rows(pr, "tottime", 1)
    if rows:
        tt, ct, nc, name = rows[0]
        print("  * hottest pure-Python frame: %s -- %.3f s own time over %d calls."
              % (name, tt, nc))
        print("    If it loops over C++-backed / array data, that is your MAP target.")
    if manifest and manifest.get("instantiations"):
        top = manifest["instantiations"][0]
        print("  * costliest crossing: %s (%.0f ms). If it is a first-use spike, the"
              % (top["signature"], top["total_ms"]))
        print("    compile cache / warmup applies (COMMON_PATTERNS §23, §15).")
    print("  See skills/cppyy-accelerate/SKILL.md 'MAP' for the decision tree.")
    print("=" * 74 + "\n")


if __name__ == "__main__":
    main()
