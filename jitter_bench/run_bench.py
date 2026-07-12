#!/usr/bin/env python3
"""jitter_bench matrix runner -- variant x condition, one command.

Runs the requested loop variants under the requested conditions (idle / load), applies the
real-time knobs once up front (mlockall / CPU affinity / scheduling policy), and prints the
jitter table + a latency histogram per cell. ``--json`` also dumps the full stats.

Examples
--------
  # The reference matrix (all variants, idle + load), 60 s each:
  ROS_DOMAIN_ID=63 python jitter_bench/run_bench.py --duration 60 --mlock --cpu 2

  # One cell, re-runnable in a single command (Stage-1 rerun shape):
  python jitter_bench/run_bench.py --variant a1 --condition idle --duration 60 \
      --sched fifo --mlock --cpu 2 --preempt-label full

  # Fast smoke (what the CI test drives):
  python jitter_bench/run_bench.py --variant a1,b --duration 1 --smoke
"""
import argparse
import json
import os
import platform
import sys
import time

# jitter_bench is a top-level package on PYTHONPATH (pixi.toml), like ik_bench.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from jitter_bench import harness   # noqa: E402
from jitter_bench.load import BackgroundLoad   # noqa: E402


# --- variant adapters: each yields (recorder, base_ns, period_ns, n) --------
class PyVariant:
    """a1 / a2: the pure-Python timer loop with a chosen sleep mechanism."""

    def __init__(self, sleep_kind):
        self.sleep_kind = sleep_kind

    def prepare(self, compute_iters, rate):
        self._body, _ = harness.small_compute(compute_iters)

    def run(self, rate, duration):
        return harness.run_fixed_rate(rate, duration, self.sleep_kind, body=self._body)

    def close(self):
        pass


class CppVariant:
    """b: the C++ wait+compute loop (cppdef_cached + nogil)."""

    def __init__(self, use_nogil=True):
        self.use_nogil = use_nogil
        self._iters = 50

    def prepare(self, compute_iters, rate):
        from jitter_bench import cpp_loop
        self._cpp_loop = cpp_loop
        self._iters = compute_iters
        cpp_loop.ensure_built()          # compile-cache before the timed run

    def run(self, rate, duration):
        return self._cpp_loop.run_cpp_loop(rate, duration, compute_iters=self._iters,
                                           use_nogil=self.use_nogil)

    def close(self):
        pass


class ControlVariant:
    """c: the real in-process ros2_control loop driven from Python (control_kit)."""

    def __init__(self, controller="python"):
        self.controller = controller
        self._loop = None

    def prepare(self, compute_iters, rate):
        from jitter_bench.control_loop import ControlLoop
        self._loop = ControlLoop(rate, controller=self.controller).setup()

    def run(self, rate, duration):
        return harness.run_fixed_rate(rate, duration, "clock_nanosleep",
                                      body=self._loop.body)

    def close(self):
        if self._loop is not None:
            self._loop.teardown()
            self._loop = None


VARIANTS = {
    "a1": ("pure-Python, clock_nanosleep(ABSTIME)", lambda: PyVariant("clock_nanosleep")),
    "a2": ("pure-Python, time.sleep-to-deadline", lambda: PyVariant("time_sleep")),
    "b":  ("cppyy_kit C++ loop (cppdef_cached+nogil)", lambda: CppVariant(use_nogil=True)),
    "c":  ("control_kit ros2_control loop (Python controller)", lambda: ControlVariant()),
}


def _machine_facts():
    def _read(path):
        try:
            with open(path) as fh:
                return fh.read().strip()
        except OSError:
            return "?"
    try:
        import resource
        rtprio = resource.getrlimit(resource.RLIMIT_RTPRIO)[0]
        memlock = resource.getrlimit(resource.RLIMIT_MEMLOCK)[0]
    except Exception:
        rtprio = memlock = "?"
    return {
        "kernel": platform.release(),
        "cmdline": _read("/proc/cmdline"),
        "ncpu": os.cpu_count(),
        "sched_rt_runtime_us": _read("/proc/sys/kernel/sched_rt_runtime_us"),
        "rlimit_rtprio": rtprio,
        "rlimit_memlock": memlock,
        "python": platform.python_version(),
    }


def _run_cell(variant, rate, duration, drop_warmup):
    rec, base_ns, period_ns, n = variant.run(rate, duration)
    stats = harness.compute_stats(rec.timestamps(), base_ns, period_ns,
                                  drop_warmup=drop_warmup)
    return stats


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", default="a1,a2,b,c",
                    help="comma list of a1,a2,b,c (default: all)")
    ap.add_argument("--condition", default="idle,load",
                    help="comma list of idle,load (default: both)")
    ap.add_argument("--rate", type=float, default=1000.0, help="target loop rate (Hz)")
    ap.add_argument("--duration", type=float, default=60.0, help="seconds per cell")
    ap.add_argument("--compute-iters", type=int, default=50,
                    help="per-cycle 'control law' fold iterations (kept << period)")
    ap.add_argument("--drop-warmup", type=int, default=100,
                    help="cycles dropped from stats (first-use/settling); reported explicitly")
    ap.add_argument("--mlock", action="store_true", help="attempt mlockall")
    ap.add_argument("--timerslack-ns", type=int, default=1,
                    help="thread timer slack in ns (default 1 = tuned; -1 = leave OS default 50us)")
    ap.add_argument("--cpu", type=int, default=None, help="pin the bench to this CPU")
    ap.add_argument("--sched", default="other", help="scheduling policy: other|fifo|rr")
    ap.add_argument("--prio", type=int, default=80, help="rt priority for fifo/rr")
    ap.add_argument("--load-n", type=int, default=8, help="busy-loop procs for the load condition")
    ap.add_argument("--preempt-label", default=None,
                    help="record the (owner-set) preemption mode for the report; not applied")
    ap.add_argument("--smoke", action="store_true",
                    help="tiny run for CI (skip load, short, tolerate variant unavailability)")
    ap.add_argument("--json", dest="json_out", default=None, help="write full stats JSON here")
    ap.add_argument("--no-hist", action="store_true", help="suppress the per-cell histogram")
    args = ap.parse_args(argv)

    want_variants = [v.strip() for v in args.variant.split(",") if v.strip()]
    want_conditions = [c.strip() for c in args.condition.split(",") if c.strip()]
    if args.smoke:
        want_conditions = ["idle"]

    # Capture the FULL cpu set BEFORE pinning -- affinity restricts sched_getaffinity, so
    # the load-cpu pool and the reported cpu count must be read first.
    all_cpus = sorted(os.sched_getaffinity(0))
    load_cpus = [c for c in all_cpus if c != args.cpu][-args.load_n:] or all_cpus

    # --- apply the RT knobs once, record outcomes ---
    knobs = {}
    knobs["timerslack"] = harness.apply_timerslack(args.timerslack_ns)
    knobs["scheduling"] = harness.apply_scheduling(args.sched, args.prio)
    knobs["affinity"] = harness.apply_affinity(args.cpu)
    knobs["mlockall"] = harness.try_mlockall() if args.mlock else (False, "not requested")

    facts = _machine_facts()
    report = {
        "meta": {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "rate_hz": args.rate, "duration_s": args.duration,
            "compute_iters": args.compute_iters, "drop_warmup": args.drop_warmup,
            "preempt_label": args.preempt_label or "unknown (needs sudo to read/set)",
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        },
        "machine": facts, "knobs": {k: {"ok": v[0], "detail": v[1]} for k, v in knobs.items()},
        "cells": [],
    }

    print("=" * 78)
    print("jitter_bench -- low-jitter control reference (current kernel)")
    print("=" * 78)
    print("kernel=%s  ncpu=%s  rtprio_limit=%s  memlock_limit=%s"
          % (facts["kernel"], facts["ncpu"], facts["rlimit_rtprio"], facts["rlimit_memlock"]))
    print("rate=%g Hz  duration=%g s/cell  compute_iters=%d  drop_warmup=%d  preempt=%s"
          % (args.rate, args.duration, args.compute_iters, args.drop_warmup,
             report["meta"]["preempt_label"]))
    for k, (ok, detail) in knobs.items():
        print("  %-11s %s  (%s)" % (k + ":", "OK" if ok else "--", detail))
    print()

    rows = []
    for vkey in want_variants:
        if vkey not in VARIANTS:
            print("!! unknown variant %r, skipping" % vkey)
            continue
        label, factory = VARIANTS[vkey]
        variant = factory()
        try:
            variant.prepare(args.compute_iters, args.rate)
        except Exception as exc:
            reason = type(exc).__name__ + ": " + str(exc).splitlines()[0]
            print("[%s] %s -- UNAVAILABLE (%s)" % (vkey, label, reason))
            report["cells"].append({"variant": vkey, "label": label, "unavailable": reason})
            continue
        try:
            for cond in want_conditions:
                load = None
                if cond == "load":
                    load = BackgroundLoad(n=args.load_n, load_cpus=load_cpus,
                                          avoid_cpu=args.cpu).start()
                    time.sleep(1.0)      # let the load ramp
                try:
                    stats = _run_cell(variant, args.rate, args.duration, args.drop_warmup)
                finally:
                    if load is not None:
                        load.stop()
                lat = stats["latency_us"]
                pj = stats["period_jitter_us"]
                print("[%s | %s] %s" % (vkey, cond, label))
                if load is not None:
                    print("   load: %s" % load.describe())
                print("   cycles=%d used=%d (dropped %d)  achieved=%.1f Hz  late(>1.5x)=%d (%.2f%%)  overrun=%d"
                      % (stats["cycles"], stats["cycles_used"], stats["dropped_warmup"],
                         stats["achieved_hz"], stats["late_1_5x"], stats["late_1_5x_pct"],
                         stats["overrun_full_period"]))
                print("   latency us: min=%.1f mean=%.1f p50=%.1f p99=%.1f p99.9=%.1f max=%.1f (std %.1f)"
                      % (lat["min"], lat["mean"], lat["p50"], lat["p99"], lat["p99.9"],
                         lat["p100"], lat["std"]))
                print("   period jitter us: mean=%.2f |p99|=%.1f |p99.9|=%.1f max=%.1f"
                      % (pj["mean"], pj["abs_p99"], pj["abs_p999"], pj["max"]))
                if not args.no_hist:
                    print(harness.latency_histogram(stats["_latency_us_arr"]))
                print()
                cell = {"variant": vkey, "label": label, "condition": cond,
                        "load": load.describe() if load else None,
                        **{k: v for k, v in stats.items() if not k.startswith("_")}}
                report["cells"].append(cell)
                rows.append((vkey, cond, lat, stats))
        finally:
            variant.close()

    # --- summary table ---
    if rows:
        print("-" * 78)
        print("SUMMARY  (wakeup latency vs programmed deadline, us)")
        print("%-4s %-6s %8s %8s %8s %8s %8s %9s"
              % ("var", "cond", "p50", "p99", "p99.9", "max", "mean", "late%"))
        for vkey, cond, lat, stats in rows:
            print("%-4s %-6s %8.1f %8.1f %8.1f %8.1f %8.1f %8.2f%%"
                  % (vkey, cond, lat["p50"], lat["p99"], lat["p99.9"], lat["p100"],
                     lat["mean"], stats["late_1_5x_pct"]))
        print("-" * 78)

    if args.json_out:
        parent = os.path.dirname(os.path.abspath(args.json_out))
        os.makedirs(parent, exist_ok=True)
        with open(args.json_out, "w") as fh:
            json.dump(report, fh, indent=2)
        print("wrote %s" % args.json_out)

    return report


if __name__ == "__main__":
    main()
