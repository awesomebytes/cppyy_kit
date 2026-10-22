"""Compare equal-contract cloud operations and same-binary C++ call paths."""
import datetime
import importlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BUILD = ROOT / "build/runtime-cost-cloud"
MODULES = {"pybind11": "cloud_pybind11", "nanobind": "cloud_nanobind",
           "rust": "joint_step_rust"}
MODES = ["numpy", "cppyy_shared", "plain", "cached", *MODULES]
SIZES = [1, 32, 1000, 100000, 307200]
DECLARATION = """
#include <cstdint>
#include <cstddef>
void cloud_transform(std::uintptr_t, std::uintptr_t, std::uintptr_t, std::size_t);
"""


def validate(points, transform, output):
    """Same contract for every public path; conversion and aliasing are rejected."""
    for array in (points, transform, output):
        if not isinstance(array, np.ndarray) or array.dtype != np.dtype("float64"):
            raise TypeError("Expected float64 NumPy arrays")
        if not array.flags.c_contiguous or not array.flags.aligned:
            raise ValueError("Expected aligned C-contiguous arrays")
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Points must have shape (N, 3)")
    if transform.shape != (3, 4) or output.shape != (len(points), 4):
        raise ValueError("Expected transform (3, 4) and output (N, 4)")
    if not output.flags.writeable:
        raise ValueError("Output must be writable")
    if np.shares_memory(output, points) or np.shares_memory(output, transform):
        raise ValueError("Output must not alias either input")


def numpy_operation(points, transform, output, scratch):
    np.matmul(points, transform[:, :3].T, out=scratch)
    np.add(scratch, transform[:, 3], out=scratch)
    np.einsum("ij,ij->i", scratch, scratch, out=output[:, 3])
    output[:, :3] = scratch


def prepare(raw, points, transform, output, checked_each_call):
    """Retain owners; prepared buffers must not be resized or changed concurrently."""
    validate(points, transform, output)
    addresses = (points.ctypes.data, transform.ctypes.data, output.ctypes.data, len(points))
    scratch = np.empty_like(points) if raw is None else None

    def call():
        if checked_each_call:
            validate(points, transform, output)
        if raw is None:
            numpy_operation(points, transform, output, scratch)
        elif checked_each_call:
            raw(points.ctypes.data, transform.ctypes.data, output.ctypes.data, len(points))
        else:
            raw(*addresses)
    return call


def load(mode):
    if mode == "numpy":
        return None
    if mode in MODULES:
        sys.path[:0] = [str(BUILD), str(ROOT / "build/runtime-cost-native")]
        return importlib.import_module(MODULES[mode])._transform_addresses
    cppyy = importlib.import_module("cppyy")
    if mode == "cppyy_shared":
        cppyy.load_library(str(BUILD / "libcloud_kernel.so"))
        cppyy.cppdef(DECLARATION)
    elif mode == "plain":
        cppyy.cppdef((HERE / "kernel.cpp").read_text())
    else:
        kit = importlib.import_module("cppyy_kit")
        kit.cppdef_cached((HERE / "kernel.cpp").read_text(), decls=DECLARATION,
                          name="comparison_cloud_transform")
    return cppyy.gbl.cloud_transform


def worker(mode):
    started = time.perf_counter_ns()
    raw = load(mode)
    setup_ms = (time.perf_counter_ns() - started) / 1e6
    rss_kib = next(int(line.split()[1]) for line in Path("/proc/self/status").read_text().splitlines()
                   if line.startswith("VmRSS:"))
    print(json.dumps({"ready": True, "setup_ms": setup_ms}), flush=True)
    transform = np.array([[.8, -.6, 0, .2], [.6, .8, 0, -.3], [0, 0, 1, .4]])
    rows = []
    for count in SIZES:
        points = np.random.default_rng(42).normal(size=(count, 3))
        output = np.empty((count, 4))
        xyz = points @ transform[:, :3].T + transform[:, 3]
        reference = np.column_stack((xyz, (xyz * xyz).sum(axis=1)))
        for checked in (True, False):
            call = prepare(raw, points, transform, output, checked)
            for _ in range(10):
                call()
            assert np.allclose(output, reference, rtol=1e-12, atol=1e-12)
            error = float(np.max(np.abs(output - reference)))
            # Adapt call count to ~10 ms per batch, with bounded small-case work.
            started = time.perf_counter_ns()
            for _ in range(10):
                call()
            pilot_ns = (time.perf_counter_ns() - started) / 10
            repeats = max(5, min(20000, int(1e7 / max(pilot_ns, 1))))
            batches = []
            for _ in range(5):
                started = time.perf_counter_ns()
                for _ in range(repeats):
                    call()
                batches.append((time.perf_counter_ns() - started) / repeats)
            # Individual durations, not a percentile of batch averages.
            individual = []
            for _ in range(2000 if count <= 1000 else 200):
                started = time.perf_counter_ns()
                call()
                individual.append(time.perf_counter_ns() - started)
            assert np.allclose(output, reference, rtol=1e-12, atol=1e-12)
            rows.append(dict(points=count, contract="checked" if checked else "prepared",
                             median_ns=statistics.median(batches), batch_ns=batches,
                             individual_ns=individual, calls_per_batch=repeats, max_error=error))
    print(json.dumps(dict(mode=mode, setup_ms=setup_ms, startup_rss_mib=rss_kib/1024, rows=rows)))


def main():
    env = {key: value for key, value in os.environ.items() if "PCH" not in key}
    env.update(CPPYY_KIT_NO_AUTOPCH="1", OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1",
               CPPYY_KIT_CACHE_DIR=str(ROOT / "build/runtime-cost-functions"))
    results = {mode: [] for mode in MODES}

    def run(mode):
        started = time.perf_counter_ns()
        process = subprocess.Popen([sys.executable, str(HERE / "benchmark.py"), mode],
                                   env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        ready = process.stdout.readline()
        launch_ms = (time.perf_counter_ns() - started) / 1e6
        stdout, stderr = process.communicate()
        if process.returncode:
            raise RuntimeError(stderr)
        assert json.loads(ready)["ready"]
        result = json.loads(stdout)
        result["launch_to_ready_ms"] = launch_ms
        return result

    preparation = run("cached")
    for repetition in range(3):
        for mode in MODES[repetition:] + MODES[:repetition]:
            results[mode].append(run(mode))
    summaries = {}
    for mode, runs in results.items():
        rows = []
        for index, row in enumerate(runs[0]["rows"]):
            latencies = [value for run in runs for value in run["rows"][index]["individual_ns"]]
            rows.append(dict(points=row["points"], contract=row["contract"],
                             median_ns=statistics.median(run["rows"][index]["median_ns"] for run in runs),
                             process_medians_ns=[run["rows"][index]["median_ns"] for run in runs],
                             p95_ns=float(np.percentile(latencies, 95))))
        summaries[mode] = dict(rows=rows, launch_to_ready_ms=statistics.median(
            run["launch_to_ready_ms"] for run in runs), startup_rss_mib=statistics.median(
                run["startup_rss_mib"] for run in runs))
    report = dict(recorded_at=datetime.datetime.now().astimezone().isoformat(),
                  method="Three processes, rotated mode order, warm OS caches. Equal Python validation. "
                  "cppyy_shared/pybind11/nanobind call same shared C++ kernel. "
                  "Plain/cached compile same source; Rust ports same scalar formula. "
                  "Five adaptive batches per size/contract; per-call p95 pools 6000 samples for "
                  "N<=1000 or 600 for larger inputs. Fixed reused buffers; data and code caches warm. "
                  "No CPU affinity, shared workstation, single-thread BLAS. Raw timing includes timer overhead.",
                  summary=summaries, samples=results, cache_preparation=preparation)
    (HERE / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(summaries, indent=2))


if __name__ == "__main__":
    if len(sys.argv) == 2:
        worker(sys.argv[1])
    else:
        main()
