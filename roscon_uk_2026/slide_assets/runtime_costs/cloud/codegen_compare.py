"""Bounded check: can offline GCC optimization recover the JIT advantage?"""
import importlib.util
import json
import os
from pathlib import Path
import shlex
import shutil
import statistics
import subprocess
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BUILD = ROOT / "build/runtime-cost-cloud"
VARIANTS = ("offline_original", "offline_native", "offline_vectorized", "jit")


def worker(variant):
    spec = importlib.util.spec_from_file_location("cloud_benchmark", HERE / "benchmark.py")
    benchmark = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(benchmark)
    if variant == "jit":
        raw = benchmark.load("plain")
    else:
        sys.path.insert(0, str(BUILD if variant == "offline_original" else BUILD / variant))
        raw = importlib.import_module("cloud_nanobind")._transform_addresses
    points = np.random.default_rng(42).normal(size=(307200, 3))
    transform = np.array([[.8, -.6, 0, .2], [.6, .8, 0, -.3], [0, 0, 1, .4]])
    output = np.empty((len(points), 4))
    call = benchmark.prepare(raw, points, transform, output, True)
    xyz = points @ transform[:, :3].T + transform[:, 3]
    reference = np.column_stack((xyz, (xyz * xyz).sum(axis=1)))
    for _ in range(30):
        call()
    batches = []
    for _ in range(7):
        start = time.perf_counter_ns()
        for _ in range(100):
            call()
        batches.append((time.perf_counter_ns() - start) / 100)
    assert np.allclose(output, reference, rtol=1e-12, atol=1e-12)
    print(json.dumps(dict(variant=variant, batch_ns=batches, median_ns=statistics.median(batches),
                         max_error=float(np.max(np.abs(output-reference))),
                         kernel_mappings=[line for line in Path('/proc/self/maps').read_text().splitlines()
                                          if 'libcloud_kernel.so' in line])))


def main(clang_comparison=False):
    compiler = shlex.split(os.environ.get("CXX", "c++"))
    variants = VARIANTS
    repetitions = 3
    configurations = [
        ("offline_native", compiler, ["-march=native", "-mtune=native"], "-O3"),
        ("offline_vectorized", compiler,
         ["-march=nocona", "-mtune=haswell", "-fvect-cost-model=unlimited"], "-O3"),
    ]
    if clang_comparison:
        clang = [str(ROOT / "build/runtime-cost-tools/.pixi/envs/default/bin/clang++")]
        portable = ["-march=nocona", "-mtune=haswell"]
        configurations = [
            ("offline_gcc_o2", compiler, portable, "-O2"),
            ("offline_clang_o2", clang, portable, "-O2"),
            ("offline_clang_o3", clang, portable, "-O3"),
            ("offline_clang_native", clang, ["-march=native", "-mtune=native"], "-O3"),
        ]
        variants = ("offline_original", *(item[0] for item in configurations), "jit")
        repetitions = 5
        os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    commands = []
    for variant, executable, flags, optimization in configurations:
        destination = BUILD / variant
        destination.mkdir(exist_ok=True)
        command = [*executable, "-std=c++17", optimization, "-ffp-contract=off", "-fPIC", "-shared",
                   *flags, str(HERE / "kernel.cpp"), "-o", str(destination / "libcloud_kernel.so")]
        subprocess.run(command, check=True, capture_output=True, text=True)
        for wrapper in BUILD.glob("cloud_nanobind*.so"):
            shutil.copy2(wrapper, destination / wrapper.name)
        commands.append(command)
    environment = {key: value for key, value in os.environ.items() if "PCH" not in key}
    environment.update(CPPYY_KIT_NO_AUTOPCH="1", OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    samples = {variant: [] for variant in variants}
    for repetition in range(repetitions):
        for variant in variants[repetition:] + variants[:repetition]:
            env = dict(environment)
            completed = subprocess.run([sys.executable, str(HERE / "codegen_compare.py"), variant],
                                       env=env, capture_output=True, text=True)
            if completed.returncode:
                raise RuntimeError(completed.stderr)
            row = json.loads(completed.stdout)
            if variant.startswith("offline_") and variant != "offline_original":
                assert row["kernel_mappings"] and all(str(BUILD / variant) in line
                                                      for line in row["kernel_mappings"])
            samples[variant].append(row)
    summary = {variant: dict(median_ms=statistics.median(row["median_ns"] for row in rows)/1e6,
                             process_medians_ms=[row["median_ns"]/1e6 for row in rows])
               for variant, rows in samples.items()}
    report = dict(method=f"{repetitions} processes per variant, rotated order. Same shared Python validation, "
                  "307200 points, 30 warmup calls and seven batches of 100 calls. "
                  "All offline variants use unchanged nanobind wrapper; loader mappings verified. "
                  "No fast-math; new variants disable FMA contraction. Shared workstation, warm caches.",
                  cpu_affinity=sorted(os.sched_getaffinity(0)),
                  build_commands=commands, summary=summary, samples=samples)
    filename = "clang_results.json" if clang_comparison else "codegen_results.json"
    (HERE / filename).write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--clang":
        main(clang_comparison=True)
    elif len(sys.argv) == 2:
        worker(sys.argv[1])
    else:
        main()
