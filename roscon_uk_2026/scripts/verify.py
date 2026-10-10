"""Independent parity checks and computation timings for saved agent solutions."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import statistics
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def provenance():
    """Record the package and solution bytes used by this current check."""
    import cppyy_kit
    package = Path(cppyy_kit.__file__).resolve()
    if package != ROOT.parent / "cppyy_kit/__init__.py":
        raise RuntimeError(f"expected this checkout's cppyy_kit, found {package}; run the rehearsal Pixi task")
    return {"package_source": str(package), "package_mode": "checkout",
            "solutions_sha256": {
                path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sorted((ROOT / "solutions").glob("*.py"))},
            "pixi_lock_sha256": hashlib.sha256((ROOT / "pixi.lock").read_bytes()).hexdigest()}


def write_report(report, path):
    path = Path(path).resolve()
    if path.is_relative_to(ROOT / "evaluation") or path.is_relative_to(ROOT / "next_steps_evaluation"):
        raise ValueError("historical evaluation directories are read-only evidence; choose a new output path")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n")


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def timed(fn, *args, **kwargs):
    started = time.perf_counter()
    result = fn(*args, **kwargs)
    first = 1000*(time.perf_counter()-started)
    durations = []
    for _ in range(7):
        started = time.perf_counter()
        fn(*args, **kwargs)
        durations.append(1000*(time.perf_counter()-started))
    return result, {"first_call_ms": first, "warm_median_ms": statistics.median(durations)}


def oracle_sweep(baseline, t, states, thresholds, hold=.02):
    counts = np.zeros((len(thresholds),2),np.int32)
    seconds = np.zeros((len(thresholds),2))
    for k, threshold in enumerate(thresholds):
        for hand, offset in enumerate((0,6)):
            mask = baseline.motion_mask(t,states[:,offset:offset+3],threshold,threshold/2,hold)
            counts[k,hand] = np.count_nonzero(np.diff(np.r_[0,mask]) == 1)
            seconds[k,hand] = np.sum(np.diff(t)*mask[1:])
    return counts, seconds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "build/current-checkout/measurements.json")
    args = parser.parse_args()
    baseline = load(ROOT / "solutions/python_baseline.py")
    native = load(ROOT / "solutions/python_native.py")
    mcap = load(ROOT / "solutions/mcap_native.py")
    ros_baseline = load(ROOT / "examples/03_ros/node.py")
    ros_native = load(ROOT / "solutions/ros_native.py")
    rng = np.random.default_rng(273)
    thresholds = np.linspace(.02,.20,41)
    for _ in range(30):
        n = int(rng.integers(1,300))
        t = np.r_[0.,np.cumsum(rng.uniform(.005,.04,n-1))]
        states = np.cumsum(rng.normal(0,.002,(n,12)),axis=0)
        for hold in (0.,.02,.12):
            expected = oracle_sweep(baseline,t,states,thresholds,hold)
            actual = mcap.sweep(t,states,thresholds,hold)
            online = ros_native.rolling_query(t,states,thresholds,hold)
            for output in (actual,online):
                np.testing.assert_array_equal(output[0],expected[0])
                np.testing.assert_allclose(output[1],expected[1],atol=1e-12,rtol=1e-12)
        np.testing.assert_array_equal(native.motion_mask(t,states[:,:3]),baseline.motion_mask(t,states[:,:3]))
    t, xyz = baseline.synthetic()
    expected, pytime = timed(baseline.motion_mask,t,xyz)
    actual, ctime = timed(native.motion_mask,t,xyz)
    np.testing.assert_array_equal(actual,expected)
    started = time.perf_counter()
    ns, t, states = mcap.load_poses(ROOT / "data/hiw_pillow_episode_0002.mcap")
    decode_ms = 1000*(time.perf_counter()-started)
    expected, mpy = timed(oracle_sweep,baseline,t,states,thresholds,.02)
    actual, mcpp = timed(mcap.sweep,t,states,thresholds,.02)
    np.testing.assert_array_equal(actual[0],expected[0])
    np.testing.assert_allclose(actual[1],expected[1],atol=1e-12)
    window_py, rp = timed(ros_baseline.rolling_query,t[-256:],states[-256:],thresholds)
    window_cpp, rc = timed(ros_native.rolling_query,t[-256:],states[-256:],thresholds)
    np.testing.assert_array_equal(window_cpp[0],window_py[0])
    np.testing.assert_allclose(window_cpp[1],window_py[1],atol=1e-12)
    report = {"platform": platform.platform(), "processor": platform.processor(),
              "random_windows": 30, "hold_values": [0,.02,.12],
              "python_samples": 250000, "python_baseline": pytime, "python_native": ctime,
              "python_speedup": pytime["warm_median_ms"]/ctime["warm_median_ms"],
              "mcap_samples": len(t), "decode_ms": decode_ms, "hold_seconds": .02,
              "mcap_python": mpy, "mcap_native": mcpp,
              "mcap_speedup": mpy["warm_median_ms"]/mcpp["warm_median_ms"],
              "counts_at_0_02_mps": actual[0][0].tolist(),
              "counts_at_0_0785_mps": actual[0][13].tolist(),
              "ros_window_python": rp, "ros_window_native": rc,
              "ros_window_speedup": rp["warm_median_ms"]/rc["warm_median_ms"],
              "cache_condition": "shared cppyy initialization and existing kernel artifacts; first call is not a clean-cache build"}
    report["provenance"] = provenance()
    write_report(report, args.output)
    print(json.dumps(report,indent=2))


if __name__ == "__main__":
    main()
