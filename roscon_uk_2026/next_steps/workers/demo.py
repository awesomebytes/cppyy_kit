"""Run a copied-input summary job and print measured boundary costs."""
import json
import platform
import resource
import threading
import time

import cppyy
import numpy as np
from .worker import LOAD_METRICS, SummaryWorker, result_dicts


def main():
    started = time.perf_counter()
    worker = SummaryWorker(queue_capacity=256, result_capacity=256)
    construction = time.perf_counter() - started
    with worker:
        inputs = [i / 100.0 for i in range(512)]
        worker.submit(0, inputs)
        deadline = time.monotonic() + 5
        while worker.snapshot()["completed"] != 1:
            if time.monotonic() >= deadline:
                raise RuntimeError("priming job did not finish")
            time.sleep(0.001)
        worker._set_test_gate(True)
        started = time.perf_counter()
        for timestamp in range(1, 129):
            assert worker.submit(timestamp, inputs)[1]
        conversion_submission = time.perf_counter() - started
        observations = []
        supervisor_results = []

        def supervisor():
            deadline = time.monotonic() + 5
            while not worker.snapshot()["drainers"]:
                if time.monotonic() > deadline:
                    raise RuntimeError("drain did not start")
                time.sleep(0.001)
            # These Python operations occur while main waits in native drain.
            version = worker.update_scale(2.0)
            supervisor_results.extend(worker.take_results())
            for _ in range(20):
                observations.append((version, worker.snapshot()["drainers"]))
            worker._set_test_gate(False)

        thread = threading.Thread(target=supervisor)
        thread.start()
        started = time.perf_counter()
        results = worker.drain()
        drain_and_conversion = time.perf_counter() - started
        thread.join(timeout=5)
        assert not thread.is_alive()
        assert len(observations) == 20 and all(n == 1 for _, n in observations)
        assert len(supervisor_results) == 1 and supervisor_results[0].sequence == 0
        assert len(results) == 128 and all(r.parameter_version == 0 for r in results)
        assert worker.submit(129, inputs)[1]
        changed = worker.drain()
        assert changed[0].parameter_version == 1
        worker.update_scale(1.0)
        worker._set_test_gate(True)
        numeric_before = worker.snapshot()["processing_ns"]
        started = time.perf_counter()
        for timestamp in range(130, 258):
            assert worker.submit(timestamp, inputs)[1]
        warmed_conversion_submission = time.perf_counter() - started
        started = time.perf_counter()
        worker._set_test_gate(False)
        warmed = worker.drain()
        warmed_drain_conversion = time.perf_counter() - started
        warmed_native_numeric = (worker.snapshot()["processing_ns"] - numeric_before) / 1e9
        started = time.perf_counter()
        matrix = np.tile(np.asarray(inputs, dtype=np.float64), (128, 1))
        numpy_conversion = time.perf_counter() - started
        started = time.perf_counter()
        baseline_mean = matrix.mean(axis=1)
        baseline_rms = np.sqrt(np.mean(matrix * matrix, axis=1))
        numpy_numeric = time.perf_counter() - started
        np.testing.assert_allclose([r.mean for r in warmed], baseline_mean, rtol=1e-12)
        np.testing.assert_allclose([r.rms for r in warmed], baseline_rms, rtol=1e-12)
    print(json.dumps({
        "python": platform.python_version(), "cppyy": cppyy.__version__,
        "numpy": np.__version__,
        "load": LOAD_METRICS, "construction_seconds": construction,
        "conversion_submission_seconds": conversion_submission,
        "drain_and_result_conversion_seconds": drain_and_conversion,
        "native_numeric_processing_seconds": worker.snapshot()["processing_ns"] / 1e9,
        "warmed_128_jobs": {
            "conversion_submission_seconds": warmed_conversion_submission,
            "drain_result_conversion_seconds": warmed_drain_conversion,
            "native_numeric_seconds": warmed_native_numeric,
            "numpy_matrix_conversion_seconds": numpy_conversion,
            "numpy_batch_numeric_seconds": numpy_numeric,
        },
        "supervisor_observations_during_native_drain": len(observations),
        "supervisor_results_retrieved_during_native_drain": len(supervisor_results),
        "max_rss_kib_process": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "first_result": result_dicts(results[:1])[0],
        "updated_result": result_dicts(changed)[0], "snapshot": worker.snapshot(),
    }, indent=2))


if __name__ == "__main__":
    main()
