"""Replay validated settings in Python and the independent C++ executable."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import resource
import shlex
import subprocess
import sys
import time

import numpy as np

from . import (
    ROOT, FilterConfig, PoseFilter, _arrays, build_native, driver_replay,
    export_config, make_episode, native_namespace,
)


def elapsed(operation):
    start = time.perf_counter()
    result = operation()
    return result, time.perf_counter() - start


def measure(operation, reset, repetitions=100):
    values = []
    for _ in range(repetitions):
        reset()
        _, seconds = elapsed(operation)
        values.append(seconds)
    return {"median_s": float(np.median(values)), "p95_s": float(np.percentile(values, 95)),
            "repetitions": repetitions}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--samples", type=int, default=500)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "build" / "demo")
    parser.add_argument("--benchmark", action="store_true", help="force native rebuild and measure separate phases")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    episode, generation_s = elapsed(lambda: make_episode(args.seed, args.samples))
    config = FilterConfig()
    paths, compilation_s = elapsed(lambda: build_native(force=args.benchmark))
    namespace, native_load_s = elapsed(native_namespace)
    filt, construction_s = elapsed(lambda: PoseFilter(config))
    with filt:
        output, first_replay_s = elapsed(lambda: filt.process(episode["timestamps_ns"], episode["positions_m"]))
        snapshot = filt.snapshot()
        driver_output, driver_replay_s = elapsed(lambda: driver_replay(
            config, episode["timestamps_ns"], episode["positions_m"], args.output_dir))
        np.testing.assert_array_equal(driver_output, output)
        observed_error = np.linalg.norm(episode["positions_m"] - episode["truth_m"], axis=1)
        filtered_error = np.linalg.norm(output - episode["truth_m"], axis=1)
        result = {
            "episode_id": episode["episode_id"], "synthetic": True,
            "samples": args.samples, "config": config.model_dump(),
            "input_position_rmse_m": float(np.sqrt(np.mean(observed_error**2))) if args.samples else None,
            "filtered_position_rmse_m": float(np.sqrt(np.mean(filtered_error**2))) if args.samples else None,
            "driver_max_abs_difference_m": float(np.max(np.abs(output-driver_output))) if args.samples else 0.,
            "gap_resets": snapshot["gap_resets"], "native_library": str(paths["library"]),
            "native_driver": str(paths["driver"]),
            "timings": {
                "episode_generation_s": generation_s,
                "compilation_s" if args.benchmark else "cached_build_check_s": compilation_s,
                "native_loading_s": native_load_s, "first_construction_s": construction_s,
                "first_python_validated_replay_s": first_replay_s,
                "driver_export_process_read_s": driver_replay_s,
            },
        }
        if args.benchmark:
            result["timings"]["warmed_python_validated_batch"] = measure(
                lambda: filt.process(episode["timestamps_ns"], episode["positions_m"]), filt.reset)
            native = namespace.PoseEstimator(namespace.Config())
            t, x = episode["timestamps_ns"], episode["positions_m"].reshape(-1)
            out = np.empty_like(x)
            if args.samples:
                result["timings"]["warmed_direct_native_batch"] = measure(
                    lambda: native.process(t, x, args.samples, out), native.reset)
            result["timings"]["validation_contiguous"] = measure(
                lambda: _arrays(t, episode["positions_m"]), lambda: None)
            # Padding makes both slices noncontiguous without changing the samples.
            padded_t = np.repeat(t, 2)[::2]
            padded_x = np.repeat(episode["positions_m"], 2, axis=0)[::2]
            result["timings"]["validation_with_noncontiguous_copy"] = measure(
                lambda: _arrays(padded_t, padded_x), lambda: None)
            import cppyy
            import cppyy_kit
            import pydantic
            import pytest
            result["native_estimator_size_bytes"] = cppyy.sizeof("reverse_demo::PoseEstimator")
            result["process_peak_rss_kib"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            result["versions"] = {
                "python": platform.python_version(), "numpy": np.__version__, "pydantic": pydantic.__version__,
                "pytest": pytest.__version__, "cppyy": cppyy.__version__, "local_cppyy_kit": cppyy_kit.__file__,
                "compiler": subprocess.check_output([*shlex.split(os.environ.get("CXX", "c++")), "--version"], text=True).splitlines()[0],
                "platform": platform.platform(),
            }
    export_config(config, args.output_dir / "resolved.cfg")
    destination = args.output_dir / "results.json"
    destination.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    print(f"results_path={destination}")


if __name__ == "__main__":
    main()
