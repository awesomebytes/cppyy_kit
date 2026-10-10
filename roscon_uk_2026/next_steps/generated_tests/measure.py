"""Record dependency/native provenance and narrowly defined local timings."""

import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shlex
import statistics
import subprocess
import sys
import time

import numpy as np
from roscon_uk_2026.next_steps.reverse_core import PoseFilter, build_native

ROOT = Path(__file__).resolve().parent
CHECKOUT = ROOT.parents[2]


def timed(function):
    start = time.perf_counter()
    result = function()
    return result, time.perf_counter() - start


def main():
    paths, build_s = timed(build_native)
    # Compile identical sources into our own ignored directory. Preserve the
    # shared fixture's build cache and measure compilation outside cppyy startup.
    compiler = shlex.split(os.environ.get("CXX", "c++"))
    probe_dir = ROOT / "build" / "compile_probe"
    probe_dir.mkdir(parents=True, exist_ok=True)
    flags = ["-std=c++17", "-O3", "-Wall", "-Wextra", "-Werror"]
    _, compile_library_s = timed(lambda: subprocess.run([
        *compiler, *flags, "-fPIC", "-shared", str(paths["include_dir"] / "pose_filter.cpp"),
        "-I", str(paths["include_dir"]), "-o", str(probe_dir / "libreverse_pose.so")], check=True))
    _, compile_driver_s = timed(lambda: subprocess.run([
        *compiler, *flags, str(paths["include_dir"] / "driver.cpp"),
        "-I", str(paths["include_dir"]), "-L", str(probe_dir), "-lreverse_pose",
        "-Wl,-rpath,$ORIGIN", "-o", str(probe_dir / "pose_filter_driver")], check=True))
    instance, startup_s = timed(PoseFilter)
    try:
        # Shared library mapping proves the process loaded the compiled fixture.
        mapped = str(paths["library"].resolve()) in Path("/proc/self/maps").read_text()
        assert mapped
        count = 10_000
        times = np.arange(count, dtype=np.int64) * 10_000_000
        points = np.random.default_rng(17).normal(size=(count, 3))
        strided = np.repeat(points, 2, axis=1)[:, ::2]
        copy_times = [timed(lambda: np.ascontiguousarray(strided))[1] for _ in range(30)]
        _, first_s = timed(lambda: instance.process(times, points))
        contiguous_times, strided_times = [], []
        for _ in range(30):
            instance.reset()
            contiguous_times.append(timed(lambda: instance.process(times, points))[1])
            instance.reset()
            strided_times.append(timed(lambda: instance.process(times, strided))[1])
    finally:
        instance.close()
    import cppyy_kit
    report = {
        "python": sys.version.split()[0],
        "packages": {name: importlib.metadata.version(name) for name in (
            "cppyy", "cppyy-backend", "cppyy-cling", "hypothesis", "numpy", "pydantic", "pytest")},
        "checkout_cppyy_kit": str(Path(cppyy_kit.__file__).resolve().relative_to(CHECKOUT)),
        "library": str(paths["library"].relative_to(CHECKOUT)),
        "library_sha256": hashlib.sha256(paths["library"].read_bytes()).hexdigest(),
        "library_mapped": mapped,
        "source_sha256": {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in sorted(paths["include_dir"].glob("*")) if path.is_file()},
        "cached_build_lookup_s": build_s,
        "library_compilation_s": compile_library_s,
        "driver_compilation_s": compile_driver_s,
        "compiler": subprocess.check_output([*compiler, "--version"], text=True).splitlines()[0],
        "first_construction_s": startup_s,
        "first_batch_s": first_s,
        "samples_per_batch": count,
        "repetitions": 30,
        "contiguous_batch_median_s": statistics.median(contiguous_times),
        "strided_batch_median_s": statistics.median(strided_times),
        "contiguity_copy_median_s": statistics.median(copy_times),
        "notes": "Existing compiled cache. Construction includes cppyy load/declaration binding. "
                 "Batch timings include validation, allocation, native call, and output. "
                 "Copy timing is only ascontiguousarray on a strided positions buffer.",
    }
    directory = ROOT / "build"
    directory.mkdir(exist_ok=True)
    (directory / "measurement.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
