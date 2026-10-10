"""Compare equal predicates and counts; save preparation and end-to-end costs."""
import time
IMPORT_START = time.perf_counter()
import argparse
import gc
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import resource
import statistics
import sys

import numpy as np
from cppyy_kit import pydantic_structs as pyd
from model import Detection, validate
from processing import Batch, lanes, load
IMPORT_SECONDS = time.perf_counter() - IMPORT_START


def timed(function):
    start = time.perf_counter()
    result = function()
    return (time.perf_counter() - start)*1000, result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sizes", default="257,262147")
    parser.add_argument("--repeats", type=int, default=21)
    parser.add_argument("--output", default="build/benchmark.json")
    args = parser.parse_args()
    if args.repeats < 3:
        parser.error("at least 3 repeats required")
    load_ms, compile_metadata = timed(load)
    first_prepare_ms, warm = timed(lambda: Batch([dict(x=3., y=4., z=0., confidence=.7, frame=0)]))
    first_calls = {}
    methods = [("serial_aos", 1), ("serial_columns", 1), ("xsimd", 1),
               ("tbb", 1), ("tbb", 2), ("tbb", 4)]
    for method, workers in methods:
        elapsed, _ = timed(lambda: warm.run(method, workers=workers))
        first_calls[f"{method}_{workers}"] = elapsed
    result = {
        "versions": {p: importlib.metadata.version(p) for p in ("cppyy", "numpy", "pydantic", "pytest")},
        "python": platform.python_version(), "platform": platform.platform(),
        "logical_cpus": os.cpu_count(), "affinity_cpus": len(os.sched_getaffinity(0)),
        "cpu": next((line.split(":",1)[1].strip() for line in Path("/proc/cpuinfo").read_text().splitlines()
                     if line.startswith("model name")), "unknown"),
        "import_seconds": IMPORT_SECONDS, "native_load_ms": load_ms,
        "first_prepare_ms": first_prepare_ms, "first_native_calls_ms": first_calls,
        "compile": compile_metadata, "simd_lanes": lanes(), "repeats": args.repeats,
        "command": "python " + " ".join(sys.argv), "workloads": [],
    }
    for n in map(int, args.sizes.split(",")):
        rng = np.random.default_rng(712)
        gen_start = time.perf_counter()
        xyz = rng.uniform(-6., 6., size=(n, 3))
        confidence = rng.uniform(0., 1., size=n)
        frame = rng.integers(0, 64, size=n)
        rows = [dict(x=float(p[0]), y=float(p[1]), z=float(p[2]), confidence=float(c), frame=int(f))
                for p, c, f in zip(xyz, confidence, frame)]
        generation_ms = (time.perf_counter() - gen_start)*1000
        expected_flags = ((confidence >= .7) & (((xyz[:,0]*xyz[:,0] + xyz[:,1]*xyz[:,1])
                                                + xyz[:,2]*xyz[:,2]) <= 25.)).astype(np.uint8)
        expected_counts = np.bincount(frame[expected_flags.astype(bool)], minlength=64).astype(np.uint64)
        prepare_ms, batch = timed(lambda: Batch(rows))
        general_validation_ms, models = timed(lambda: validate(rows))
        general_aos_ms, vector = timed(lambda: pyd.cpp_vector(Detection, models))
        del vector, models
        work = {"n": n, "generation_ms": generation_ms, "prepare_both_ms": prepare_ms,
                "preparation_components_ms": batch.costs, "native_bytes_both": batch.native_bytes,
                "general_path_validation_ms": general_validation_ms, "general_aos_ms": general_aos_ms,
                "methods": {}}
        for method, workers in methods:
            name = f"{method}_{workers}"
            for _ in range(3):
                batch.run(method, workers=workers)
            measurements = []
            for _ in range(args.repeats):
                elapsed, (flags, counts, _) = timed(lambda: batch.run(method, workers=workers))
                assert np.array_equal(flags, expected_flags), name
                assert np.array_equal(counts, expected_counts), name
                measurements.append(elapsed)
            # Prepare only the layout required by this candidate. AoS bulk fill
            # retains source columns through the existing prototype's keep_alive.
            total_start = time.perf_counter()
            one_shot_prepare_ms, one_shot = timed(lambda: Batch(
                rows, layout="aos" if method in {"serial_aos", "tbb"} else "columns"))
            one_shot_run_ms, (flags, counts, _) = timed(lambda: one_shot.run(method, workers=workers))
            end_to_end_ms = (time.perf_counter() - total_start)*1000
            assert np.array_equal(flags, expected_flags)
            assert np.array_equal(counts, expected_counts)
            observed = 0
            if method == "tbb":
                _, _, observed = batch.run(method, workers=workers, instrument=True)
                assert observed <= workers
            work["methods"][name] = {
                "median_ms": statistics.median(measurements), "min_ms": min(measurements),
                "p95_ms": float(np.percentile(measurements,95)), "samples_ms": measurements,
                "one_shot_end_to_end_ms": end_to_end_ms, "one_shot_components_ms": one_shot.costs,
                "one_shot_prepare_ms": one_shot_prepare_ms, "one_shot_run_ms": one_shot_run_ms,
                "native_bytes_retained": one_shot.native_bytes, "observed_peak_tasks": observed,
            }
            del one_shot
            gc.collect()
        work["maxrss_mib_so_far"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
        result["workloads"].append(work)
        del batch, rows, xyz, frame, confidence
        gc.collect()
    output = Path(args.output)
    output.parent.mkdir(exist_ok=True, parents=True)
    output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
