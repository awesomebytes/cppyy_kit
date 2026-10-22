"""Measure startup, warmup, throughput and instrumented per-call latency."""
import importlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time

NUMPY_START = time.perf_counter_ns()
import numpy as np
NUMPY_IMPORT_MS = (time.perf_counter_ns() - NUMPY_START) / 1e6

HERE = Path(__file__).resolve().parent
BUILD = HERE.parents[2] / "build/runtime-cost-native"
MODULES = {"native": "eigen_step_native", "nanobind": "eigen_step_nanobind",
           "rust": "joint_step_rust"}
CALLS = 10000
INDIVIDUAL_CALLS = 50000


def worker(mode):
    started = time.perf_counter_ns()
    if mode in MODULES:
        sys.path.insert(0, str(BUILD))
        module = importlib.import_module(MODULES[mode])
    else:
        module = importlib.import_module("cppyy" if mode == "plain" else "cppyy_kit")
    import_ms = (time.perf_counter_ns() - started) / 1e6
    started = time.perf_counter_ns()
    if mode in MODULES:
        function = module.joint_step
    else:
        cppyy = module if mode == "plain" else module.cppyy
        source = (HERE.parent / "eigen_startup/step.cpp").read_text()
        include = str(Path(os.environ["CONDA_PREFIX"]) / "include/eigen3")
        if mode == "plain":
            cppyy.add_include_path(include)
            cppyy.cppdef(source)
        else:
            module.cppdef_cached(
                source, decls="void joint_step(const double*, const double*, double, double*);",
                include_paths=[include], name="runtime_joint_step",
            )
        function = cppyy.gbl.joint_step
    prepare_ms = (time.perf_counter_ns() - started) / 1e6
    jacobian = np.random.default_rng(7).normal(size=(6, 7))
    delta = np.array([.01, -.02, .03, .005, -.004, .002])
    output = np.empty(7)
    started = time.perf_counter_ns()
    function(jacobian, delta, .05, output)
    first_ms = (time.perf_counter_ns() - started) / 1e6
    started = time.perf_counter_ns()
    for _ in range(1000):
        function(jacobian, delta, .05, output)
    warmup_ms = (time.perf_counter_ns() - started) / 1e6
    stages = dict(numpy_import_ms=NUMPY_IMPORT_MS, binding_import_ms=import_ms,
                  prepare_ms=prepare_ms, first_call_ms=first_ms, warmup_1000_ms=warmup_ms)
    print(json.dumps({"ready": stages}), flush=True)

    samples = []
    for _ in range(7):
        start = time.perf_counter_ns()
        for _ in range(CALLS):
            function(jacobian, delta, .05, output)
        samples.append((time.perf_counter_ns() - start) / CALLS)
    latencies = np.empty(INDIVIDUAL_CALLS, dtype=np.int64)
    timer = np.empty(INDIVIDUAL_CALLS, dtype=np.int64)
    for index in range(INDIVIDUAL_CALLS):
        start = time.perf_counter_ns()
        function(jacobian, delta, .05, output)
        latencies[index] = time.perf_counter_ns() - start
    for index in range(INDIVIDUAL_CALLS):
        start = time.perf_counter_ns()
        timer[index] = time.perf_counter_ns() - start
    path = HERE / "latency_samples" / f"{mode}-{os.getpid()}.npz"
    path.parent.mkdir(exist_ok=True)
    np.savez_compressed(path, latency_ns=latencies, timer_ns=timer)
    probes = {}
    if mode in ("native", "nanobind"):
        for name, args in (("scalar_probe", (3.0,)),
                           ("array_probe", (jacobian, delta, .05, output))):
            probe = getattr(module, name)
            for _ in range(1000):
                probe(*args)
            batches = []
            for _ in range(7):
                start = time.perf_counter_ns()
                for _ in range(CALLS):
                    probe(*args)
                batches.append((time.perf_counter_ns() - start) / CALLS)
            probes[name] = statistics.median(batches)
    reference = jacobian.T @ np.linalg.solve(
        jacobian @ jacobian.T + .05**2 * np.eye(6), delta,
    )
    assert np.allclose(output, reference, rtol=1e-10, atol=1e-12)
    print(json.dumps({"mode": mode, "stages": stages, "batch_ns_per_call": samples,
                      "median_ns_per_call": statistics.median(samples),
                      "latency_file": str(path.relative_to(HERE)),
                      "probe_ns": probes,
                      "p95_ns": float(np.percentile(latencies, 95)),
                      "max_error": float(np.max(np.abs(output - reference)))}))


def main():
    env = {key: value for key, value in os.environ.items() if "PCH" not in key}
    env.update(CPPYY_KIT_NO_AUTOPCH="1", OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1",
               CPPYY_KIT_CACHE_DIR=str(HERE.parents[2] / "build/runtime-cost-functions"))
    modes = ["plain", "cached", *MODULES]
    results = {mode: [] for mode in modes}
    for repetition in range(3):
        for mode in modes[repetition:] + modes[:repetition]:
            started = time.perf_counter_ns()
            process = subprocess.Popen(
                [sys.executable, str(HERE / "speed.py"), mode], env=env,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            )
            ready = process.stdout.readline()
            ready_ms = (time.perf_counter_ns() - started) / 1e6
            stdout, stderr = process.communicate()
            if process.returncode:
                raise RuntimeError(stderr)
            assert "ready" in json.loads(ready)
            row = json.loads(stdout)
            row["launch_to_warm_ms"] = ready_ms
            results[mode].append(row)
    summary = {}
    for mode, rows in results.items():
        arrays = [np.load(HERE / row["latency_file"]) for row in rows]
        latency = np.concatenate([array["latency_ns"] for array in arrays])
        timer = np.concatenate([array["timer_ns"] for array in arrays])
        summary[mode] = {
            "median_ns_per_call": statistics.median(row["median_ns_per_call"] for row in rows),
            "individual_p50_ns": float(np.percentile(latency, 50)),
            "individual_p95_ns": float(np.percentile(latency, 95)),
            "individual_p99_ns": float(np.percentile(latency, 99)),
            "process_p95_ns": [row["p95_ns"] for row in rows],
            "timer_p50_ns": float(np.percentile(timer, 50)),
            "timer_p95_ns": float(np.percentile(timer, 95)),
            "launch_to_warm_ms": statistics.median(row["launch_to_warm_ms"] for row in rows),
            "stages_ms": {key: statistics.median(row["stages"][key] for row in rows)
                          for key in rows[0]["stages"]},
            "probes_ns": {key: statistics.median(row["probe_ns"][key] for row in rows)
                          for key in rows[0]["probe_ns"]},
        }
        for array in arrays:
            array.close()
    report = {
        "method": "Three fresh processes, rotated order, warm OS caches and prepared kit cache. "
                  "First call then 1000 warmup calls, seven batches of 10000 calls. "
                  "Additional 50000 individually timed calls per process; pooled p95 from 150000. "
                  "Timer control measured separately and not subtracted. Retained callable/arrays. "
                  "Common O3 Eigen object in pybind11/nanobind; Rust LLT versus Eigen LDLT. "
                  "Launch-to-warm includes interpreter, imports, input setup, preparation, first "
                  "call, 1000 calls and pipe notification. Shared workstation, no CPU pinning.",
        "summary": summary, "samples": results,
    }
    (HERE / "speed_results.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    if len(sys.argv) == 2:
        worker(sys.argv[1])
    else:
        main()
