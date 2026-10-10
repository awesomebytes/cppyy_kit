#!/usr/bin/env python
"""Run in the checkout's locked ompl Pixi environment. Children contain probes."""
import argparse
import gc
import importlib.util
import json
import os
from pathlib import Path
import platform
import resource
import statistics
import subprocess
import sys
import time
import weakref

HERE = Path(__file__).resolve().parent


def load_policy(path):
    spec = importlib.util.spec_from_file_location("extension_policy", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def bootstrap():
    started = time.perf_counter()
    import cppyy
    import ompl_kit
    ob, _ = ompl_kit.bringup_ompl()
    bringup_s = time.perf_counter() - started
    started = time.perf_counter()
    cppyy.include(str(HERE / "native.hpp"))
    parse_s = time.perf_counter() - started
    return cppyy, ompl_kit, ob, cppyy.gbl.extension_example, bringup_s, parse_s


class Session:
    """One synchronous engine. Own the override until after native detach."""
    def __init__(self, native, policy_class, bias=0.0, fail_at=None):
        self.engine = native.Engine()
        self.policy = policy_class(self.engine.space_information(), bias, fail_at)
        self.engine.attach(self.policy)

    def close(self):
        if self.engine is not None:
            self.engine.close()
            self.engine = None
            self.policy = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def worker(args):
    cppyy, kit, ob, native, bringup_s, parse_s = bootstrap()
    kit.set_seed(args.seed)
    PyPolicy = load_policy(args.policy).make_policy(ob)
    cppyy.gbl.ompl.msg.setLogLevel(cppyy.gbl.ompl.msg.LOG_NONE)
    points = [(0.1, 0.1), (0.5, 0.5), (0.25, 0.5), (0.75, 0.5),
              (0.9, 0.9), (-0.01, 0.1), (1.01, 0.9)]
    t = time.perf_counter()
    flat = cppyy.gbl.std.vector["double"]([v for point in points for v in point])
    conversion_s = time.perf_counter() - t
    t = time.perf_counter()
    engine = native.Engine(args.resolution)
    policy = (PyPolicy(engine.space_information(), args.bias)
              if args.worker == "python"
              else native.NativePolicy(engine.space_information(), args.bias))
    engine.attach(policy)
    construction_s = time.perf_counter() - t
    t = time.perf_counter()
    engine.check(flat, 1)
    dispatch_first_s = time.perf_counter() - t
    # Warm only the call wrapper. A closed engine throws before reaching OMPL,
    # so this does not consume random samples or alter the measured plan.
    probe = native.Engine()
    probe.close()
    t = time.perf_counter()
    try:
        probe.solve(0.0)
    except Exception as error:
        assert "engine is closed" in str(error)
    solve_wrapper_first_s = time.perf_counter() - t
    del probe
    samples = []
    for _ in range(args.repetitions):
        policy.calls = 0
        t = time.perf_counter()
        result = engine.check(flat, args.repeats)
        total_s = time.perf_counter() - t
        count = int(policy.calls)
        assert count == len(points) * args.repeats
        samples.append({"native_seconds": float(result.seconds),
                        "wall_seconds": total_s, "calls": count})
    decisions = list(result.decisions)
    accepted = int(result.accepted)
    policy.calls = 0
    t = time.perf_counter()
    solved = engine.solve(5.0)
    solve_wall_s = time.perf_counter() - t
    solve_calls = int(policy.calls)
    # The only Python call was Engine.solve. OMPL initiated every override.
    output = {
        "kind": args.worker, "seed": args.seed, "bias_x": args.bias,
        "validity_resolution": args.resolution,
        "points": points, "decisions": decisions, "accepted": accepted,
        "repeats": args.repeats, "samples": samples,
        "exact": bool(solved.exact), "path_xy": list(solved.xy),
        "path_length": float(solved.length), "solve_calls": solve_calls,
        "solve_native_seconds": float(solved.seconds), "solve_wall_seconds": solve_wall_s,
        "calls_per_solve_second": solve_calls / float(solved.seconds),
        "bringup_seconds": bringup_s, "adapter_header_parse_seconds": parse_s,
        "construction_first_use_seconds": construction_s, "first_dispatch_seconds": dispatch_first_s,
        "solve_wrapper_first_use_seconds": solve_wrapper_first_s,
        "input_conversion_seconds": conversion_s,
        "max_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "versions": {"python": platform.python_version(), "cppyy": cppyy.__version__,
                     "ompl": json.loads(next((Path(os.environ["CONDA_PREFIX"]) / "conda-meta").glob("ompl-*.json")).read_text())["version"]},
    }
    before = int(policy.calls)
    engine.close()
    engine.close()
    assert int(policy.calls) == before
    del engine, policy
    gc.collect()
    print("RESULT " + json.dumps(output))


def lifecycle_worker(args):
    cppyy, _, ob, native, _, _ = bootstrap()
    PyPolicy = load_policy(args.policy).make_policy(ob)
    cppyy.gbl.ompl.msg.setLogLevel(cppyy.gbl.ompl.msg.LOG_NONE)
    released = 0
    for _ in range(25):
        session = Session(native, PyPolicy)
        ref = weakref.ref(session.policy)
        calls = session.policy.calls
        # Session keeps the policy alive across Python collection.
        gc.collect()
        assert ref() is not None
        vector = cppyy.gbl.std.vector["double"]([0.1, 0.1, 0.5, 0.5])
        answer = session.engine.check(vector, 1)
        assert list(answer.decisions) == [1, 0]
        assert session.policy.calls == calls + 2
        engine = session.engine
        policy = session.policy
        calls = policy.calls
        session.close()
        session.close()
        assert policy.calls == calls
        try:
            engine.check(vector, 1)
        except Exception as error:
            assert "engine is closed" in str(error)
        else:
            raise AssertionError("closed engine accepted dispatch")
        assert policy.calls == calls
        del policy, engine, session
        gc.collect()
        assert ref() is None, "Python policy survived explicit native detach"
        released += 1
    errors = []
    for operation, fail_at in [("check", 2), ("solve", 7)]:
        with Session(native, PyPolicy, fail_at=fail_at) as session:
            try:
                if operation == "check":
                    session.engine.check(vector, 3)
                else:
                    session.engine.solve(5.0)
            except Exception as error:
                assert "scripted validity policy failure" in str(error)
                assert session.policy.calls == fail_at
                errors.append({"operation": operation, "type": type(error).__name__,
                               "message": str(error), "calls": session.policy.calls})
            else:
                raise AssertionError("callback exception did not reach caller")
    special = [0.25, 0.5, 0.75, 0.5, 0.5, 0.75, 0.5, 0.25,
               0.0, 0.0, 1.0, 1.0, float("nan"), 0.1,
               float("inf"), 0.1, 0.1, -float("inf")]
    special_decisions = []
    for policy_class in [PyPolicy, lambda si, bias, fail: native.NativePolicy(si, bias)]:
        with Session(native, policy_class) as session:
            answer = session.engine.check(cppyy.gbl.std.vector["double"](special), 1)
            special_decisions.append(list(answer.decisions))
    assert special_decisions == [[0, 0, 0, 0, 1, 1, 0, 0, 0]] * 2
    invalid_bias_rejections = 0
    for policy_class in [PyPolicy, native.NativePolicy]:
        for bias in [float("nan"), float("inf"), -float("inf")]:
            engine = native.Engine()
            try:
                policy_class(engine.space_information(), bias)
            except Exception as error:
                assert "bias must be finite" in str(error)
                invalid_bias_rejections += 1
            else:
                raise AssertionError("nonfinite bias was accepted")
            finally:
                engine.close()
    print("RESULT " + json.dumps({"kind": "lifecycle", "released": released,
                                 "exceptions": errors, "boundary_nonfinite_decisions": special_decisions,
                                 "invalid_bias_rejections": invalid_bias_rejections}))


def run_child(args, kind, bias=None):
    cmd = [sys.executable, str(HERE / "demo.py"), "--worker", kind,
           "--policy", str(Path(args.policy).resolve()), "--repeats", str(args.repeats),
           "--repetitions", str(args.repetitions), "--seed", str(args.seed),
           "--bias", str(args.bias if bias is None else bias),
           "--resolution", str(getattr(args, "resolution", 0.001))]
    child_env = dict(os.environ, CPPYY_KIT_NO_AUTOPCH="1")
    started = time.perf_counter()
    child = subprocess.run(cmd, capture_output=True, text=True, timeout=120, env=child_env)
    elapsed = time.perf_counter() - started
    lines = [line[7:] for line in child.stdout.splitlines() if line.startswith("RESULT ")]
    if child.returncode or len(lines) != 1:
        raise RuntimeError(f"{kind} probe exit {child.returncode}\n{child.stdout}\n{child.stderr}")
    result = json.loads(lines[0])
    result["process_seconds"] = elapsed
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", choices=["python", "native", "lifecycle"])
    parser.add_argument("--policy", default=str(HERE / "policy.py"))
    parser.add_argument("--repeats", type=int, default=30000)
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--bias", type=float, default=0.05)
    parser.add_argument("--resolution", type=float, default=0.001)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.repeats < 1 or args.repetitions < 1:
        parser.error("repeat counts must be positive")
    if args.worker == "lifecycle":
        lifecycle_worker(args)
    elif args.worker:
        worker(args)
    else:
        results = [run_child(args, kind) for kind in ["python", "native", "lifecycle"]]
        from check import check_results
        check_results(results)
        output = {"results": results, "checks": "passed"}
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(output, indent=2) + "\n")
        for result in results[:2]:
            median_s = statistics.median(s["native_seconds"] for s in result["samples"])
            count = result["samples"][0]["calls"]
            print(f"{result['kind']}: exact={result['exact']} solve_calls={result['solve_calls']} "
                  f"solve_ms={result['solve_native_seconds'] * 1000:.3f} "
                  f"batch_ns/call={median_s / count * 1e9:.1f}")
        print("PASS: independent policy/path checks, native parity, errors, 25 releases")


if __name__ == "__main__":
    main()
