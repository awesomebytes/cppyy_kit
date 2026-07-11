#!/usr/bin/env python
"""
Validate + measure the cppyy_kit compile cache on the bt tick path (the headline
first-use JIT the PCH cannot touch, per docs/FREEZE.md).

Two ways to hand a Python leaf to BehaviorTree.CPP:
  * BASELINE -- ``factory.registerSimpleAction(id, leaf)`` (what bt_kit does today):
    cppyy JIT-compiles the ``std::function<NodeStatus(TreeNode&)>`` thunk AND the
    register call wrapper on first use (~0.4 s, every process).
  * CACHED -- route the crossing through a **trampoline** built by
    ``cppyy_kit.cppdef_cached(..., trampoline=True)``: the std::function and the
    registration are compiled ONCE into a ``.so`` (using CPyCppyy's public API to
    turn the C++ ``TreeNode&`` into the Python node proxy), and every later run
    ``load_library``'s it -- the first live call is a ~ms symbol call.

This script is also the **bt_kit adoption reference** (bt_kit/ is out of the M2a
lane): TRAMP_CODE/TRAMP_DECLS + register_cached_action() are what a kit drops in.

    pixi run -e bt python scripts/cache/validate_cache_bt.py            # cold table
    RCLCPPYY_FROZEN=1 python scripts/freeze/run_frozen.py \
        scripts/cache/validate_cache_bt.py                             # frozen+cached
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

# --- the trampoline glue a kit would adopt --------------------------------
# Definitions (compiled once into the cached .so). Builds the std::function in
# compiled code and registers it; converts TreeNode& -> Python proxy via CPyCppyy.
TRAMP_CODE = r"""
#include <Python.h>
#include <behaviortree_cpp/bt_factory.h>
#include <CPyCppyy/API.h>
#include <functional>
namespace btcache {
static BT::NodeStatus call_py(PyObject* pyfn, BT::TreeNode& node) {
  PyGILState_STATE g = PyGILState_Ensure();
  PyObject* pynode = CPyCppyy::Instance_FromVoidPtr((void*)&node, "BT::TreeNode");
  PyObject* res = PyObject_CallFunctionObjArgs(pyfn, pynode, nullptr);
  long status = 3;  // FAILURE if the Python call errored
  if (res) { status = PyLong_AsLong(res); Py_DECREF(res); } else { PyErr_Print(); }
  Py_XDECREF(pynode);
  PyGILState_Release(g);
  return static_cast<BT::NodeStatus>(status);
}
void register_py_action(BT::BehaviorTreeFactory& factory, const std::string& id, PyObject* pyfn) {
  Py_XINCREF(pyfn);  // the std::function outlives this call; pin the callable
  factory.registerSimpleAction(id, [pyfn](BT::TreeNode& n) { return call_py(pyfn, n); });
}
}  // namespace btcache
"""
# Bodiless declarations (cheap to cppdef on a cache hit; the .so has the bodies).
TRAMP_DECLS = r"""
#include <Python.h>
#include <behaviortree_cpp/bt_factory.h>
namespace btcache {
  void register_py_action(BT::BehaviorTreeFactory& factory, const std::string& id, PyObject* pyfn);
}
"""

XML = '<root BTCPP_format="4"><BehaviorTree ID="M"><A/></BehaviorTree></root>'


def _bt_paths():
    from ament_index_python.packages import get_package_prefix
    p = get_package_prefix("behaviortree_cpp")
    return os.path.join(p, "include"), os.path.join(p, "lib")


def register_cached_action(factory, node_id, leaf):
    """The kit-adoption one-liner: compile-cache the tick trampoline (once) and
    register ``leaf`` through it. ``leaf(node)`` returns an int NodeStatus."""
    import cppyy
    import cppyy_kit
    inc, lib = _bt_paths()
    cppyy_kit.cppdef_cached(TRAMP_CODE, decls=TRAMP_DECLS, name="bt_tick",
                            trampoline=True, include_paths=[inc],
                            library_paths=[lib], libraries=["behaviortree_cpp"])
    cppyy.gbl.btcache.register_py_action(factory, node_id, leaf)


# --- worker: one cold end-to-end run, prints JSON --------------------------
def _worker(variant):
    import bt_kit
    from cppyy_kit import freeze

    t = {}

    def stage(name, fn):
        s = time.perf_counter()
        r = fn()
        t[name] = (time.perf_counter() - s) * 1000
        return r

    bt = stage("bringup", bt_kit.bringup_bt)
    keep = []

    def leaf(node):
        return 2  # SUCCESS

    factory = bt.BehaviorTreeFactory()
    keep.append(leaf)
    if variant == "baseline":
        stage("register", lambda: factory.registerSimpleAction("A", leaf))
    else:
        stage("register", lambda: register_cached_action(factory, "A", leaf))
    tree = stage("build_tree", lambda: factory.create_tree_from_text(XML))
    st = stage("first_tick", tree.tickWhileRunning)
    out = {"variant": variant, "stages": t, "status": int(st),
           "frozen": freeze.active("bt"), "keep": len(keep)}
    print("JSON:" + json.dumps(out))


def _run_worker(variant, extra_env=None):
    env = os.environ.copy()
    if extra_env:
        env.update(extra_env)
    s = time.perf_counter()
    p = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker", variant],
                       capture_output=True, text=True, env=env)
    wall = (time.perf_counter() - s) * 1000
    for line in p.stdout.splitlines():
        if line.startswith("JSON:"):
            d = json.loads(line[5:])
            d["wall_ms"] = wall
            return d
    raise RuntimeError("worker %s failed:\n%s\n%s" % (variant, p.stdout, p.stderr))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--worker", metavar="VARIANT")
    ap.add_argument("-n", "--runs", type=int, default=3)
    args = ap.parse_args()
    if args.worker:
        return _worker(args.worker)

    import cppyy_kit
    frozen = os.environ.get("RCLCPPYY_FROZEN") == "1"
    label = "L1 frozen + cache" if frozen else "L0 + cache"
    print("bt tick-path cache validation  (%s, %d runs)\n" % (label, args.runs))

    # BASELINE (no cache) for reference -- median of the runs.
    base = [_run_worker("baseline") for _ in range(args.runs)]
    b = base[-1]
    print("BASELINE (cppyy JIT, no cache): register %.0f ms | first_tick %.0f ms | "
          "wall %.0f ms | status %d | frozen=%s"
          % (b["stages"]["register"], b["stages"]["first_tick"],
             statistics.median(d["wall_ms"] for d in base), b["status"], b["frozen"]))

    # CACHED: clear the cache so run 1 is a guaranteed miss, then runs 2..N are hits.
    cppyy_kit.clear_cache()
    print("\nCACHED trampoline (run 1 = miss/compile, runs 2+ = hit):")
    print("%-6s %10s %12s %10s %8s %s" % ("run", "register", "first_tick", "wall", "status", "note"))
    print("-" * 62)
    for i in range(1, args.runs + 1):
        d = _run_worker("cached")
        note = "MISS (compiled .so)" if i == 1 else "hit (load .so)"
        assert d["status"] == 2, "cached tree did not reach SUCCESS"
        print("%-6d %8.0f ms %10.0f ms %8.0f ms %8d  %s"
              % (i, d["stages"]["register"], d["stages"]["first_tick"],
                 d["wall_ms"], d["status"], note))

    for row in cppyy_kit.cache_info():
        print("\ncached artifact: %s" % os.path.basename(row["so"]))


if __name__ == "__main__":
    main()
