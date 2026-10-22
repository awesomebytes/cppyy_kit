"""Measure resident memory and stage latency in fresh Linux Python processes."""
import datetime
import importlib
import json
import os
from pathlib import Path
import platform
import shutil
import statistics
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
NATIVE_BUILD = HERE.parents[2] / "build/runtime-cost-native"
TOOLS = HERE.parents[2] / "build/runtime-cost-tools"
COMPILED = {
    "native": "eigen_step_native",
    "nanobind": "eigen_step_nanobind",
    "rust": "joint_step_rust",
}


def memory():
    fields = {}
    for line in Path("/proc/self/smaps_rollup").read_text().splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == "kB":
            fields[parts[0].rstrip(":")] = int(parts[1]) / 1024
    return {
        "rss_mib": fields["Rss"],
        "pss_mib": fields["Pss"],
        "private_mib": fields["Private_Clean"] + fields["Private_Dirty"],
    }


def worker(mode):
    rows = []

    def record(stage, started):
        rows.append(dict(stage=stage, elapsed_ms=(time.perf_counter() - started) * 1000,
                         **memory()))

    record("python_baseline", time.perf_counter())
    cppyy = None
    if mode not in COMPILED:
        started = time.perf_counter()
        module = importlib.import_module("cppyy" if mode == "plain" else "cppyy_kit")
        cppyy = module if mode == "plain" else module.cppyy
        record("cppyy_ready", started)

        started = time.perf_counter()
        cppyy.cppdef("double runtime_probe(double x) { return x * x; }")
        record("small_definition", started)
        started = time.perf_counter()
        assert cppyy.gbl.runtime_probe(3.0) == 9.0
        record("small_first_call", started)

    started = time.perf_counter()
    np = importlib.import_module("numpy")
    record("numpy_import", started)
    source = (HERE.parent / "eigen_startup" / "step.cpp").read_text()
    include = str(Path(os.environ["CONDA_PREFIX"]) / "include/eigen3")
    started = time.perf_counter()
    cache = None
    if mode in COMPILED:
        sys.path.insert(0, str(NATIVE_BUILD))
        native = importlib.import_module(COMPILED[mode])
    elif mode == "plain":
        cppyy.add_include_path(include)
        cppyy.cppdef(source)
    else:
        cache = module.cppdef_cached(
            source, decls="void joint_step(const double*, const double*, double, double*);",
            include_paths=[include], name="runtime_joint_step",
        )
    record("eigen_load", started)
    jacobian = np.random.default_rng(7).normal(size=(6, 7))
    delta = np.array([.01, -.02, .03, .005, -.004, .002])
    output = np.empty(7)
    started = time.perf_counter()
    if mode in COMPILED:
        native.joint_step(jacobian, delta, .05, output)
    else:
        cppyy.gbl.joint_step(jacobian, delta, .05, output)
    record("eigen_first_call", started)
    started = time.perf_counter()
    if mode in COMPILED:
        native.joint_step(jacobian, delta, .05, output)
    else:
        cppyy.gbl.joint_step(jacobian, delta, .05, output)
    record("eigen_second_call", started)
    reference = jacobian.T @ np.linalg.solve(
        jacobian @ jacobian.T + .05**2 * np.eye(6), delta,
    )
    assert np.allclose(output, reference, rtol=1e-10, atol=1e-12)
    print(json.dumps({
        "mode": mode, "stages": rows, "cache": cache,
        "cppyy_version": cppyy.__version__ if cppyy else None,
        "compiler_mappings": [
            line for line in Path("/proc/self/maps").read_text().splitlines()
            if any(name in line.lower() for name in ("cling", "llvm", "libclang"))
        ],
        "pch": os.environ.get("CLING_STANDARD_PCH"),
        "max_error": float(np.max(np.abs(output - reference))),
    }))


def main():
    env = {key: value for key, value in os.environ.items() if "PCH" not in key}
    env.update(CPPYY_KIT_NO_AUTOPCH="1",
               CPPYY_KIT_CACHE_DIR=str(HERE.parents[2] / "build/runtime-cost-functions"))

    def run(mode):
        completed = subprocess.run(
            [sys.executable, str(HERE / "measure.py"), mode], env=env,
            capture_output=True, text=True, check=True,
        )
        result = json.loads(completed.stdout)
        result["stderr"] = completed.stderr
        return result

    started = time.perf_counter()
    build_log = []
    for command in (
        ["cmake", "-S", str(HERE / "native"), "-B", str(NATIVE_BUILD),
         "-DCMAKE_BUILD_TYPE=Release", f"-DPython_EXECUTABLE={sys.executable}",
         f"-Dnanobind_DIR={TOOLS / 'python/nanobind/cmake'}"],
        ["cmake", "--build", str(NATIVE_BUILD), "--clean-first", "--parallel", "2"],
    ):
        completed = subprocess.run(command, env=env, capture_output=True, text=True, check=True)
        build_log.append(completed.stdout + completed.stderr)
    native_build_s = time.perf_counter() - started
    rust_bin = TOOLS / ".pixi/envs/default/bin"
    rust_env = dict(env, PATH=str(rust_bin) + os.pathsep + env["PATH"],
                    PYO3_PYTHON=sys.executable,
                    CARGO_TARGET_DIR=str(NATIVE_BUILD / "rust-target"))
    started = time.perf_counter()
    rust_build = subprocess.run(
        [str(rust_bin / "cargo"), "build", "--release",
         "--manifest-path", str(HERE / "rust/Cargo.toml")],
        env=rust_env, capture_output=True, text=True, check=True,
    )
    rust_build_s = time.perf_counter() - started
    shutil.copy2(NATIVE_BUILD / "rust-target/release/libjoint_step_rust.so",
                 NATIVE_BUILD / "joint_step_rust.so")
    preparation = run("cached")
    samples = {mode: [] for mode in ("plain", "cached", *COMPILED)}
    for _ in range(3):
        for mode in samples:
            samples[mode].append(run(mode))
    medians = {}
    for mode, runs in samples.items():
        medians[mode] = [
            dict(stage=stage["stage"], **{
                field: statistics.median(run["stages"][index][field] for run in runs)
                for field in ("elapsed_ms", "rss_mib", "pss_mib", "private_mib")
            })
            for index, stage in enumerate(runs[0]["stages"])
        ]
    result = {
        "recorded_at": datetime.datetime.now().astimezone().isoformat(),
        "python": sys.version, "platform": platform.platform(),
        "method": "Three interleaved fresh processes per path, standard existing cppyy PCH; "
                  "kit startup hook disabled; OS caches warm. Stage times exclude memory "
                  "sampling. RSS includes shared pages. PSS apportions shared pages. "
                  "Private is Private_Clean + Private_Dirty. No compiler child peak measurement.",
        "preparation": preparation, "samples": samples, "medians": medians,
        "native_build_s": native_build_s, "native_build_log": build_log,
        "rust_build_s": rust_build_s,
        "rust_build_log": rust_build.stdout + rust_build.stderr,
        "native_artifacts": {
            path.name: path.stat().st_size for path in NATIVE_BUILD.glob("*.so")
        },
    }
    (HERE / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(medians, indent=2))


if __name__ == "__main__":
    if len(sys.argv) == 2:
        worker(sys.argv[1])
    else:
        main()
