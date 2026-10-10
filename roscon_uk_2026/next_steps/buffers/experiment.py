"""Measure compile, buffer preparation, warmed kernels and end-to-end costs."""
import time
_START = time.perf_counter()
import argparse
import gc
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import resource
import statistics
import subprocess
import sys
import weakref

import numpy as np
import buffers as b
_IMPORT_SECONDS = time.perf_counter() - _START


def median_seconds(operation, repeats=21):
    samples = []
    for _ in range(repeats):
        start = time.perf_counter()
        operation()
        samples.append(time.perf_counter() - start)
    return statistics.median(samples)


def numpy_pipeline(points, rotation, translation, radius):
    transformed = points @ rotation.T + translation
    selected = transformed[np.sum(transformed * transformed, axis=1) <= radius * radius]
    return transformed, selected, float(np.sum(selected * selected))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="measurement.json")
    parser.add_argument("--points", type=int, default=100000)
    args = parser.parse_args()
    native_load = b.load_native()
    small = np.ones((2, 3))
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    translation = np.array([0.5, -0.2, 0.3])
    first = {}
    for name, operation in [
        ("transform", lambda: b.transform_kernel(small, rotation, translation, np.empty_like(small))),
        ("filter", lambda: b.filter_kernel(small, 4., np.empty_like(small))),
        ("energy", lambda: b.energy_kernel(small)),
        ("pointer", lambda: b.pointer_kernel(small)),
    ]:
        start = time.perf_counter()
        operation()
        first[name] = time.perf_counter() - start
    points = np.random.default_rng(20261004).normal(size=(args.points, 3))
    radius = 1.4
    lease = b.BorrowedPoints(points)
    expected_t, expected_p, expected_e = numpy_pipeline(points, rotation, translation, radius)
    result = b.process(lease, rotation, translation, radius)
    np.testing.assert_allclose(result.transformed, expected_t, rtol=2e-13, atol=2e-13)
    np.testing.assert_allclose(result.points, expected_p, rtol=2e-13, atol=2e-13)
    assert np.isclose(result.energy, expected_e, rtol=2e-13, atol=2e-13)

    def prepare(source, copy=False):
        with b.BorrowedPoints(source, copy=copy):
            pass
    strided = points[::2]
    strided_reference = strided.copy()
    copy_lease = b.BorrowedPoints(strided, copy=True)
    readonly = points.view()
    readonly.flags.writeable = False
    readonly_lease = b.BorrowedPoints(readonly)

    transformed = np.empty_like(points)
    storage = np.empty_like(points)
    def kernels():
        b.transform_kernel(points, rotation, translation, transformed)
        count = int(b.filter_kernel(transformed, radius * radius, storage))
        return b.energy_kernel(storage[:count])
    kernels()
    conversion = {
        "borrow_validate_and_close_full_input_s": median_seconds(lambda: prepare(points)),
        "readonly_borrow_validate_and_close_full_input_s": median_seconds(lambda: prepare(readonly)),
        "strided_reject_s": None,
        "strided_copy_validate_and_close_half_input_s": median_seconds(lambda: prepare(strided, True)),
        "float32_copy_validate_and_close_full_input_s": None,
        "rigid_parameters_s": median_seconds(lambda: b.rigid_parameters(rotation, translation, radius)),
    }
    float32 = points.astype(np.float32)
    conversion["float32_copy_validate_and_close_full_input_s"] = median_seconds(lambda: prepare(float32, True))
    def reject():
        try:
            prepare(strided)
        except TypeError:
            return
        raise AssertionError("strided input accepted without copy")
    conversion["strided_reject_s"] = median_seconds(reject)
    runtime = {
        "transform_reused_output_s": median_seconds(lambda: b.transform_kernel(points, rotation, translation, transformed)),
        "filter_reused_output_s": median_seconds(lambda: b.filter_kernel(transformed, radius * radius, storage)),
        "energy_selected_view_s": median_seconds(lambda: b.energy_kernel(result.points)),
        "three_kernels_reused_outputs_s": median_seconds(kernels),
        "public_process_retained_lease_s": median_seconds(lambda: b.process(lease, rotation, translation, radius)),
        "public_process_new_lease_s": median_seconds(lambda: b.process(points, rotation, translation, radius)),
        "numpy_vectorized_pipeline_s": median_seconds(lambda: numpy_pipeline(points, rotation, translation, radius)),
    }
    def dlpack_import():
        with b.dlpack_points(points):
            pass
    conversion["dlpack_import_borrow_validate_and_close_s"] = median_seconds(dlpack_import)
    producer = np.arange(30, dtype=np.float64).reshape(10, 3).copy()
    producer_ref = weakref.ref(producer)
    producer_pointer = int(producer.ctypes.data)
    dlpack = b.dlpack_points(producer)
    sharing = {
        "borrow_input_native_pointer_equal": int(b.pointer_kernel(lease.array)) == points.ctypes.data,
        "readonly_native_pointer_equal": int(b.pointer_kernel(readonly_lease.array)) == points.ctypes.data,
        "strided_copy_pointer_different": copy_lease.array.ctypes.data != strided.ctypes.data,
        "strided_copy_shares_memory": bool(np.shares_memory(copy_lease.array, strided)),
        "filter_prefix_native_pointer_equal": int(b.pointer_kernel(result.points)) == result.storage.ctypes.data,
        "filter_prefix_shares_storage": bool(np.shares_memory(result.points, result.storage)),
        "transformed_shares_input": bool(np.shares_memory(result.transformed, points)),
        "filtered_shares_transformed": bool(np.shares_memory(result.points, result.transformed)),
        "cpu_dlpack_device": producer.__dlpack_device__(),
        "cpu_dlpack_import_pointer_equal": dlpack.array.ctypes.data == producer_pointer,
        "cpu_dlpack_native_pointer_equal": int(b.pointer_kernel(dlpack.array)) == producer_pointer,
        "cpu_dlpack_import_writeable": bool(dlpack.array.flags.writeable),
    }
    del producer
    gc.collect()
    sharing["cpu_dlpack_producer_survives_gc"] = producer_ref() is not None
    assert b.energy_kernel(dlpack.array) == float(np.sum(np.arange(30, dtype=np.float64) ** 2))
    dlpack.close()
    gc.collect()
    sharing["cpu_dlpack_producer_released_after_close"] = producer_ref() is None
    np.testing.assert_array_equal(copy_lease.array, strided_reference)
    compiler = subprocess.run([os.environ.get("CXX", "c++"), "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    import cppyy
    evidence = {
        "date": "2026-10-04", "python": platform.python_version(),
        "packages": {name: importlib.metadata.version(name) for name in ("numpy", "cppyy", "pytest")},
        "compiler": compiler, "eigen_header_semver_integer": int(cppyy.gbl.buffer_demo.eigen_version()),
        "native_cplusplus": int(cppyy.gbl.buffer_demo.cpp_standard()), "core_package_path": str(b.CHECKOUT / "cppyy_kit"),
        "platform": platform.platform(), "cpu_count": os.cpu_count(),
        "blas_thread_environment": {key: os.environ.get(key) for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS")},
        "input": {"rows": args.points, "dtype": points.dtype.str, "shape": points.shape, "strides_bytes": points.strides,
                  "coordinate_unit": "metres", "filter_radius_metres": radius, "retained_rows": len(result.points)},
        "startup": {"module_import_s": _IMPORT_SECONDS, "native_header_load_s": native_load,
                    "first_wrapper_call_compile_plus_small_execution_s": first},
        "conversion": conversion, "runtime_median_21_calls": runtime,
        "sharing": sharing,
        "correctness": {"max_transform_absolute_error": float(np.max(np.abs(result.transformed - expected_t))),
                        "max_selected_absolute_error": float(np.max(np.abs(result.points - expected_p))),
                        "energy_absolute_error": abs(result.energy - expected_e)},
        "memory": {"input_bytes": points.nbytes, "each_full_native_output_bytes": result.storage.nbytes,
                   "retained_view_logical_bytes": result.points.nbytes,
                   "process_peak_rss_kib_includes_cling_and_benchmarks": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss},
        "check_sha256": hashlib.sha256(Path(__file__).with_name("test_buffers.py").read_bytes()).hexdigest(),
    }
    for item in (lease, copy_lease, readonly_lease):
        item.close()
    Path(args.output).write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2))


if __name__ == "__main__":
    main()
