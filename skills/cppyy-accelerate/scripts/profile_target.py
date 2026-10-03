#!/usr/bin/env python
"""
Run a target script under cProfile and the cppyy_kit boundary tracer. The report
shows each function's own time and cumulative time, plus the C++ signatures that
crossed the boundary and their total cost. Use high own time to find work performed
in a function. Use high cumulative time to find work performed by that function or
by functions it calls. Boundary cost can include JIT compilation and repeated calls.

    python skills/cppyy-accelerate/scripts/profile_target.py \
        examples/accelerate_demo/slow_pointcloud_pipeline.py -- -n 100000

Arguments after ``--`` are passed to the target. The target runs in this process so
cProfile can record its calls. If it uses cppyy_kit, the tracer records its boundary
crossings. Use both tables: high ``tottime`` points to Python work, while high
``total_ms`` can point to first-use JIT or repeated boundary costs.
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

    # Start the boundary tracer when cppyy_kit is importable. A plain Python target
    # may have no crossings to report.
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
    print("\nSuggested next steps:")
    rows = _stats_rows(pr, "tottime", 1)
    if rows:
        tt, ct, nc, name = rows[0]
        print("  * function with highest own time: %s (%.3f s own time over %d calls)."
              % (name, tt, nc))
        print("    If this function processes array data in a loop, consider moving the loop to C++.")
    if manifest and manifest.get("instantiations"):
        top = manifest["instantiations"][0]
        print("  * costliest crossing: %s (%.0f ms). A first-use spike can be reduced with"
              % (top["signature"], top["total_ms"]))
        print("    compile cache / warmup applies (COMMON_PATTERNS §23, §15).")
    print("  See skills/cppyy-accelerate/SKILL.md 'Choose a remedy' for the decision table.")
    print("=" * 74 + "\n")


if __name__ == "__main__":
    main()
