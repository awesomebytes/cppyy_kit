#!/usr/bin/env python
"""
The IK benchmark -- ONE Python script that benchmarks every available IK solver on
the same Panda targets and prints one table (+ ``--json``).

The story (docs/ik_bench/WHY.md): some IK solvers ship only as C++ MoveIt plugins
(bio_ik, pick_ik are not even packaged); others are packaged (KDL, trac_ik); a plain
Python baseline rounds it out. Normally comparing them means C++ harnesses, launch
files and parameter servers. Here **cppyy + moveit_kit** load each C++ plugin
in-process via pluginlib (COMMON_PATTERNS 19) and drive ``RobotState::setFromIK`` --
so this single Python file is the whole harness.

Method (honest):
  * Same robot: the MoveIt panda test model. Same seeded target set for every solver
    (``--n`` targets: reachable configs + near-joint-limit configs; each target is a
    pose from FK of a valid config, paired with a DIFFERENT random seed config the
    solver starts from -- so the solver must actually search).
  * Per solver, per target: one ``setFromIK`` (MoveIt plugins) or one DLS solve
    (Python) with the same per-solve ``--timeout``. Success is verified INDEPENDENTLY
    by forward-kinematics error against the target (position < ``--pos-tol`` m AND
    orientation < ``--ori-tol`` rad) -- not by trusting the solver's own verdict.
  * Warmup solves are excluded. Solve-rate is the median of ``--repeats`` timed
    passes. Error stats are over the verified successes. Each solver runs in a fresh
    subprocess (cppyy and NumPy stay isolated; a blocked plugin can't sink the run).

Run: ``pixi run -e ik bench-ik``  (or ``python ik_bench/run_bench.py --json out.json``)
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "61")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from ik_bench import solvers as S  # noqa: E402

DEFAULT_TARGETS = os.path.join(REPO, "build", "ik_bench", "targets.json")


# ======================================================================
# Target generation (MoveIt FK -> canonical, seeded, cached to JSON)
# ======================================================================
def worker_gen_targets(n, seed, out_path):
    """Generate the seeded target set using MoveIt's FK (canonical) and cache it.
    ~1/4 of the targets are near-joint-limit configs (harder for local solvers)."""
    import numpy as np
    import cppyy
    import moveit_kit
    from ik_bench.panda import PandaChain

    moveit = moveit_kit.bringup_moveit()
    cfg = moveit_kit.panda_config()
    model = moveit_kit.build_robot_model(cfg.urdf, cfg.srdf)
    jmg = model.getJointModelGroup(S.GROUP)
    state = moveit.core.RobotState(model)
    chain = PandaChain()
    rng = np.random.default_rng(seed)

    def fk_pose(q):
        v = cppyy.gbl.std.vector["double"]([float(x) for x in q])
        state.setJointGroupPositions(jmg, v)
        state.update()
        tf = state.getGlobalLinkTransform(S.TIP)
        tr = tf.translation()
        quat = cppyy.gbl.Eigen.Quaterniond(tf.rotation())
        return [tr[0], tr[1], tr[2], quat.x(), quat.y(), quat.z(), quat.w()]

    n_near = n // 4
    targets = []
    for i in range(n):
        near = i >= (n - n_near)
        # near-limit configs sit close to the bounds (negative margin); reachable
        # configs keep a small margin away from the extremes.
        margin = -0.02 if near else 0.05
        q_target = chain.random_config(rng, margin=margin)
        q_seed = chain.random_config(rng, margin=0.05)
        targets.append({
            "target": fk_pose(q_target),
            "seed": [float(x) for x in q_seed],
            "category": "near_limit" if near else "reachable",
        })
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as fh:
        json.dump({"seed": seed, "n": n, "group": S.GROUP, "tip": S.TIP,
                   "targets": targets}, fh)
    print("RESULT " + json.dumps({"generated": len(targets), "path": out_path}))


# ======================================================================
# Metric aggregation shared by all workers
# ======================================================================
def _summarize(key, label, note, verified, pos_errs, ori_errs, rates,
               n, solver_true, category_success):
    def stats(xs):
        if not xs:
            return {"mean": None, "median": None, "max": None}
        return {"mean": statistics.fmean(xs), "median": statistics.median(xs),
                "max": max(xs)}
    return {
        "key": key, "label": label, "note": note, "status": "ok",
        "n": n,
        "success": verified, "success_pct": 100.0 * verified / n if n else 0.0,
        "solver_reported": solver_true,
        "hz_median": statistics.median(rates) if rates else None,
        "hz_all": rates,
        "pos_err": stats(pos_errs), "ori_err": stats(ori_errs),
        "by_category": category_success,
    }


# ======================================================================
# MoveIt-plugin worker (KDL / trac_ik / bio_ik / pick_ik)
# ======================================================================
def worker_moveit(spec_key, targets_path, timeout, repeats, pos_tol, ori_tol):
    import cppyy
    from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
    import moveit_kit
    from ik_bench.panda import pose_error

    spec = S.BY_KEY[spec_key]
    if spec.prefix and os.path.isdir(spec.prefix):
        # make the vendored plugin discoverable by pluginlib's ament-index lookup
        os.environ["AMENT_PREFIX_PATH"] = (
            spec.prefix + os.pathsep + os.environ.get("AMENT_PREFIX_PATH", ""))
        os.environ["LD_LIBRARY_PATH"] = (
            os.path.join(spec.prefix, "lib") + os.pathsep
            + os.environ.get("LD_LIBRARY_PATH", ""))

    with open(targets_path) as fh:
        data = json.load(fh)
    targets = data["targets"]

    moveit = moveit_kit.bringup_moveit(with_kinematics=True)
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    cfg = moveit_kit.panda_config()
    model = moveit_kit.build_robot_model(cfg.urdf, cfg.srdf)
    jmg = model.getJointModelGroup(S.GROUP)

    # build a node carrying this plugin's params under robot_description_kinematics.<group>
    tree = {"robot_description_kinematics": {S.GROUP: dict(spec.params)}} if spec.params else None
    overrides = moveit_kit.parameter_overrides(tree) if tree else None
    node = moveit_kit.make_node("ikbench_" + spec_key, overrides)
    if not moveit_kit.load_kinematics_solver(node, model, S.GROUP, plugin=spec.plugin):
        raise RuntimeError("plugin '%s' failed to load/initialize" % spec.plugin)

    state = moveit.core.RobotState(model)

    def set_seed(seed):
        v = cppyy.gbl.std.vector["double"]([float(x) for x in seed])
        state.setJointGroupPositions(jmg, v)
        state.update()

    def solve_one(t):
        set_seed(t["seed"])
        target = moveit_kit.pose(*t["target"])
        ok = bool(state.setFromIK(jmg, target, float(timeout)))
        state.update()
        got = state.getGlobalLinkTransform(S.TIP)
        # convert MoveIt transform -> 4x4 for the shared FK-error helper
        import numpy as np
        m = np.eye(4)
        tr = got.translation()
        rot = got.rotation()
        for r in range(3):
            m[r, 3] = tr[r]
            for c in range(3):
                m[r, c] = rot(r, c) if callable(rot) else rot[r][c]
        pe, oe = pose_error(m, t["target"])
        return ok, pe, oe

    _run_and_report(spec, targets, solve_one, repeats, pos_tol, ori_tol)


# ======================================================================
# Pure-Python DLS worker (NumPy, no MoveIt/cppyy)
# ======================================================================
def worker_python(spec_key, targets_path, timeout, repeats, pos_tol, ori_tol):
    import numpy as np
    from ik_bench.panda import PandaChain, dls_ik, pose_error

    spec = S.BY_KEY[spec_key]
    with open(targets_path) as fh:
        data = json.load(fh)
    targets = data["targets"]
    chain = PandaChain()

    def solve_one(t):
        # deterministic per-target rng so restarts are reproducible across runs
        key = int(abs(sum(x * (i + 1) for i, x in enumerate(t["seed"])) * 1e6)) % (2 ** 32)
        rng = np.random.default_rng(key)
        ok, q, _ = dls_ik(chain, t["target"], t["seed"], pos_tol=pos_tol,
                          ori_tol=ori_tol, time_budget=float(timeout), rng=rng)
        pe, oe = pose_error(chain.fk(q), t["target"])
        return ok, pe, oe

    _run_and_report(spec, targets, solve_one, repeats, pos_tol, ori_tol)


def _run_and_report(spec, targets, solve_one, repeats, pos_tol, ori_tol):
    n = len(targets)
    # warmup (excluded from timing): first few solves pay JIT / first-use cost
    for t in targets[:min(8, n)]:
        solve_one(t)

    # correctness pass: verified success + error stats + per-category breakdown
    verified = 0
    solver_true = 0
    pos_errs, ori_errs = [], []
    cat_total, cat_ok = {}, {}
    for t in targets:
        ok, pe, oe = solve_one(t)
        cat = t["category"]
        cat_total[cat] = cat_total.get(cat, 0) + 1
        if ok:
            solver_true += 1
        if pe < pos_tol and oe < ori_tol:
            verified += 1
            pos_errs.append(pe)
            ori_errs.append(oe)
            cat_ok[cat] = cat_ok.get(cat, 0) + 1
    category_success = {c: {"ok": cat_ok.get(c, 0), "total": cat_total[c]}
                        for c in cat_total}

    # timed passes: solve-rate = median over `repeats`
    rates = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        for t in targets:
            solve_one(t)
        rates.append(n / (time.perf_counter() - t0))

    out = _summarize(spec.key, spec.label, spec.note, verified, pos_errs,
                     ori_errs, rates, n, solver_true, category_success)
    print("RESULT " + json.dumps(out))


# ======================================================================
# Orchestration
# ======================================================================
def _spawn(worker, extra, timeout_s):
    argv = [sys.executable, "-u", os.path.abspath(__file__), "--worker", worker] + extra
    proc = subprocess.run(argv, capture_output=True, text=True, timeout=timeout_s)
    line = next((ln for ln in proc.stdout.splitlines()
                 if ln.startswith("RESULT ")), None)
    return line, proc


def ensure_targets(args):
    if os.path.isfile(args.targets) and not args.regen:
        with open(args.targets) as fh:
            data = json.load(fh)
        if data.get("n") == args.n and data.get("seed") == args.seed:
            return
    print("  generating %d targets (seed %d) ..." % (args.n, args.seed),
          file=sys.stderr, flush=True)
    line, proc = _spawn("gen_targets",
                        ["--n", str(args.n), "--seed", str(args.seed),
                         "--targets", args.targets], args.gen_timeout)
    if line is None:
        sys.exit("target generation FAILED:\n%s\n%s"
                 % (proc.stdout[-2000:], proc.stderr[-2000:]))


def run(args):
    print("IK benchmark: same Panda, same %d targets, per-solver subprocess."
          % args.n, file=sys.stderr, flush=True)
    print("(Shared machine -> solve-rate is median of %d repeats; provisional.)"
          % args.repeats, file=sys.stderr, flush=True)
    ensure_targets(args)

    which = args.solvers.split(",") if args.solvers else [s.key for s in S.REGISTRY]
    results = []
    for key in which:
        spec = S.BY_KEY.get(key)
        if spec is None:
            print("  ? unknown solver '%s' (skip)" % key, file=sys.stderr)
            continue
        worker = "python" if spec.kind == "python" else "moveit"
        extra = ["--solver", key, "--targets", args.targets,
                 "--timeout", str(args.timeout), "--repeats", str(args.repeats),
                 "--pos-tol", str(args.pos_tol), "--ori-tol", str(args.ori_tol)]
        print("  [%s] running ..." % key, file=sys.stderr, flush=True)
        try:
            line, proc = _spawn(worker, extra, args.solver_timeout)
        except subprocess.TimeoutExpired:
            results.append({"key": key, "label": spec.label, "status": "timeout",
                            "note": spec.note})
            print("    TIMEOUT", file=sys.stderr, flush=True)
            continue
        if line is None:
            reason = _blocked_reason(proc)
            results.append({"key": key, "label": spec.label, "status": "blocked",
                            "note": spec.note, "reason": reason})
            print("    BLOCKED: %s" % reason, file=sys.stderr, flush=True)
            continue
        results.append(json.loads(line[len("RESULT "):]))

    _table(results, args)
    if args.json:
        with open(args.json, "w") as fh:
            json.dump({"config": vars(args), "results": results}, fh, indent=2)
        print("\n  wrote %s" % args.json, file=sys.stderr)
    return 0


def _blocked_reason(proc):
    tail = (proc.stderr or proc.stdout or "").strip().splitlines()
    for ln in reversed(tail):
        s = ln.strip()
        if s and not s.startswith("[") and "warning" not in s.lower():
            return s[:160]
    return "no RESULT (see stderr)"


def _table(results, args):
    print()
    print("  IK solvers benchmarked from ONE Python script (cppyy + moveit_kit)")
    print("  Robot: MoveIt Panda | targets: %d (seed %d) | per-solve timeout: %.0f ms"
          % (args.n, args.seed, args.timeout * 1000))
    print("  " + "=" * 92)
    print("  %-26s %8s %9s %13s %13s" %
          ("solver", "success", "solve/s", "pos err (mm)", "ori err (deg)"))
    print("  " + "-" * 92)
    for r in results:
        if r.get("status") != "ok":
            print("  %-26s %8s   %s"
                  % (r["label"], r.get("status", "?").upper(),
                     r.get("reason", "")[:52]))
            continue
        pe = r["pos_err"]["median"]
        oe = r["ori_err"]["median"]
        pe_s = "%.3f" % (pe * 1000) if pe is not None else "-"
        oe_s = "%.3f" % (oe * 57.29578) if oe is not None else "-"
        hz = r["hz_median"]
        print("  %-26s %6.1f%% %9.0f %13s %13s"
              % (r["label"], r["success_pct"], hz, pe_s, oe_s))
    print("  " + "-" * 92)
    print("  success = FK(solution) within %.0f mm / %.2f deg of target (verified"
          % (args.pos_tol * 1000, args.ori_tol * 57.29578))
    print("  independently, not the solver's own verdict). Errors: median over hits.")
    # near-limit breakdown, if present
    any_cat = any(r.get("status") == "ok" and r.get("by_category") for r in results)
    if any_cat:
        print("  near-limit targets (subset):")
        for r in results:
            if r.get("status") != "ok":
                continue
            nl = r.get("by_category", {}).get("near_limit")
            if nl:
                print("    %-24s %d/%d" % (r["label"], nl["ok"], nl["total"]))
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--worker", default=None,
                    choices=["gen_targets", "moveit", "python"])
    ap.add_argument("--solver", default=None, help="worker: solver key")
    ap.add_argument("--solvers", default=None,
                    help="comma list to run (default: all registered)")
    ap.add_argument("--n", type=int, default=200, help="number of targets")
    ap.add_argument("--seed", type=int, default=61)
    ap.add_argument("--repeats", type=int, default=3, help="timed passes (median)")
    ap.add_argument("--timeout", type=float, default=0.05, help="per-solve timeout (s)")
    ap.add_argument("--pos-tol", type=float, default=1e-3, help="success pos tol (m)")
    ap.add_argument("--ori-tol", type=float, default=1e-2, help="success ori tol (rad)")
    ap.add_argument("--targets", default=DEFAULT_TARGETS)
    ap.add_argument("--regen", action="store_true", help="force target regeneration")
    ap.add_argument("--json", default=None, help="also write results JSON here")
    ap.add_argument("--gen-timeout", type=float, default=300.0)
    ap.add_argument("--solver-timeout", type=float, default=600.0)
    args = ap.parse_args()

    if args.worker == "gen_targets":
        worker_gen_targets(args.n, args.seed, args.targets)
        return 0
    if args.worker == "moveit":
        worker_moveit(args.solver, args.targets, args.timeout, args.repeats,
                      args.pos_tol, args.ori_tol)
        return 0
    if args.worker == "python":
        worker_python(args.solver, args.targets, args.timeout, args.repeats,
                      args.pos_tol, args.ori_tol)
        return 0
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
