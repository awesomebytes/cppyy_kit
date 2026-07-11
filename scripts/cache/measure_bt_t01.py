#!/usr/bin/env python
"""
Measure bt_kit's compile-cache adoption end-to-end on a t01-shape workload
(4 simple actions in a Sequence, one tick), in cold subprocesses.

  pixi run -e bt python scripts/cache/measure_bt_t01.py               # JIT vs cached
  RCLCPPYY_FROZEN=1 python scripts/freeze/run_frozen.py \
      scripts/cache/measure_bt_t01.py                                 # + frozen

Each row is a fresh process (bringup + first-use are once-per-process costs). "JIT"
forces the pre-cache path (CPPYY_KIT_NO_CACHE=1); "cached" is the default, run 1 a
cache miss (compiles the .so), runs 2+ hits. When launched via run_frozen.py the
header parse is served from the PCH, composing with the cache.
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

NAMES = ["CheckBattery", "OpenGripper", "ApproachObject", "CloseGripper"]
XML = ('<root BTCPP_format="4"><BehaviorTree ID="M"><Sequence>'
       + "".join("<%s/>" % n for n in NAMES) + "</Sequence></BehaviorTree></root>")


def _worker():
    import bt_kit
    from cppyy_kit import freeze
    t = {}

    def stage(name, fn):
        s = time.perf_counter()
        r = fn()
        t[name] = (time.perf_counter() - s) * 1000
        return r

    bt = stage("bringup", bt_kit.bringup_bt)
    factory = bt.BehaviorTreeFactory()

    # First registration pays the first-use JIT on the JIT path; ~ms on the cache path.
    stage("first_register", lambda: factory.registerSimpleAction(NAMES[0], lambda n: bt_kit.SUCCESS))

    def _rest():
        for n in NAMES[1:]:
            factory.registerSimpleAction(n, lambda node: bt_kit.SUCCESS)
    stage("register_rest", _rest)
    tree = stage("build_tree", lambda: factory.create_tree_from_text(XML))
    st = stage("first_tick", tree.tickWhileRunning)
    print("JSON:" + json.dumps({"stages": t, "status": int(st),
                                "frozen": freeze.active("bt"), "cached": bt_kit._CACHED}))


def _run(no_cache):
    env = os.environ.copy()
    if no_cache:
        env["CPPYY_KIT_NO_CACHE"] = "1"
    else:
        env.pop("CPPYY_KIT_NO_CACHE", None)
    s = time.perf_counter()
    p = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker"],
                       capture_output=True, text=True, env=env)
    wall = (time.perf_counter() - s) * 1000
    for line in p.stdout.splitlines():
        if line.startswith("JSON:"):
            d = json.loads(line[5:])
            d["wall_ms"] = wall
            return d
    raise RuntimeError("worker failed:\n%s\n%s" % (p.stdout, p.stderr))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--worker", action="store_true")
    ap.add_argument("-n", "--runs", type=int, default=3)
    args = ap.parse_args()
    if args.worker:
        return _worker()

    import cppyy_kit
    frozen = os.environ.get("RCLCPPYY_FROZEN") == "1"
    tag = "frozen" if frozen else "L0"
    print("bt_kit t01 cold-run  (%s, %d runs each)\n" % (tag, args.runs))

    jit = [_run(no_cache=True) for _ in range(args.runs)]
    j = jit[-1]
    assert all(d["status"] == 2 for d in jit)
    print("%-22s %13s %11s %10s" % ("config", "first_register", "first_tick", "wall"))
    print("-" * 60)
    print("%-22s %11.0f ms %8.0f ms %7.0f ms   (JIT baseline, cached=%s)"
          % ("JIT (no cache)", j["stages"]["first_register"], j["stages"]["first_tick"],
             statistics.median(d["wall_ms"] for d in jit), j["cached"]))

    cppyy_kit.clear_cache()
    for i in range(1, args.runs + 1):
        d = _run(no_cache=False)
        assert d["status"] == 2 and d["cached"] is True
        note = "MISS compile" if i == 1 else "hit"
        print("%-22s %11.0f ms %8.0f ms %7.0f ms   (%s)"
              % ("cached run %d" % i, d["stages"]["first_register"],
                 d["stages"]["first_tick"], d["wall_ms"], note))


if __name__ == "__main__":
    main()
