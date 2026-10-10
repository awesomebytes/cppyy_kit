"""Independent lifecycle, numerical, accounting, and concurrency checks."""
import importlib
import math
import os
import subprocess
import sys
import threading
import time

import pytest

MODULE = os.environ.get("WORKERS_MODULE", "roscon_uk_2026.next_steps.workers.worker")
SummaryWorker = importlib.import_module(MODULE).SummaryWorker


def check_accounting(state, delivered=0):
    assert state["attempted"] == state["accepted"] + state["rejected"]
    assert state["accepted"] == (state["completed"] + state["failed"] +
                                 state["cancelled"] + state["queued"] +
                                 int(state["active"]))
    assert state["completed"] == delivered + state["retained_results"] + state["result_evicted"]


def test_serial_parity_order_copy_and_parameter_snapshots():
    with SummaryWorker(queue_capacity=32, result_capacity=32) as worker:
        worker._set_test_gate(True)
        expected = []
        for index in range(20):
            scale = -0.5 if index >= 7 else 1.0
            if index == 7:
                assert worker.update_scale(scale) == 1
            values = [math.sin(index + j) for j in range(11)]
            independent = [v * scale for v in values]
            expected.append((math.fsum(independent) / 11,
                             math.sqrt(math.fsum(v * v for v in independent) / 11)))
            assert worker.submit(10_000 + index * 100, values) == (index, True)
            values[:] = [1000.0] * len(values)
        assert worker.snapshot()["queued"] == 20
        worker._set_test_gate(False)
        results = worker.drain()
        assert [r.sequence for r in results] == list(range(20))
        for index, result in enumerate(results):
            assert result.timestamp_ns == 10_000 + index * 100
            assert result.parameter_version == int(index >= 7)
            assert result.count == 11
            assert result.mean == pytest.approx(expected[index][0], rel=1e-12, abs=1e-14)
            assert result.rms == pytest.approx(expected[index][1], rel=1e-12, abs=1e-14)
        check_accounting(worker.snapshot(), 20)
    assert worker.snapshot()["closed"] and not worker.snapshot()["running"]


def test_deterministic_drop_newest_and_stop_drains_gate():
    worker = SummaryWorker(queue_capacity=2, result_capacity=4).start()
    worker._set_test_gate(True)
    assert worker.submit(1, [1.0]) == (0, True)
    assert worker.submit(2, [2.0]) == (1, True)
    assert worker.submit(3, [3.0]) == (2, False)
    assert worker.submit(4, [4.0]) == (3, False)
    check_accounting(worker.snapshot())
    worker.stop()
    results = worker.take_results()
    assert [r.sequence for r in results] == [0, 1]
    assert [r.mean for r in results] == [1.0, 2.0]
    check_accounting(worker.snapshot(), 2)
    worker.stop()
    with pytest.raises(Exception, match="accepting"):
        worker.submit(5, [5.0])
    with pytest.raises(Exception, match="closed"):
        worker.start()


def test_bounded_storage_and_result_eviction_over_4000_jobs():
    delivered = 0
    with SummaryWorker(queue_capacity=4, result_capacity=3, max_samples=5) as worker:
        for batch in range(1000):
            worker._set_test_gate(True)
            for index in range(4):
                timestamp = batch * 4 + index
                assert worker.submit(timestamp, [float(timestamp)] * 5)[1]
            state = worker.snapshot()
            assert state["queued"] == 4 and state["retained_results"] == 0
            check_accounting(state, delivered)
            worker._set_test_gate(False)
            results = worker.drain()
            assert [r.sequence for r in results] == [batch * 4 + i for i in (1, 2, 3)]
            delivered += len(results)
            check_accounting(worker.snapshot(), delivered)
        assert worker.snapshot()["completed"] == 4000
        assert worker.snapshot()["result_evicted"] == 1000
        assert delivered == 3000


def test_repeated_lifecycle_validation_and_outer_exception_cleanup():
    for _ in range(30):
        worker = SummaryWorker(queue_capacity=1, result_capacity=1, max_samples=2)
        with pytest.raises(RuntimeError, match="outer"):
            with worker:
                worker.start()
                assert worker.submit(1, [2.0])[1]
                raise RuntimeError("outer")
        assert not worker.snapshot()["running"]
        assert worker.snapshot()["completed"] == 1
        worker.stop()
    with SummaryWorker(max_samples=2) as worker:
        for timestamp, values in [(-1, [1]), (0.5, [1]), (0, []),
                                  (0, [1, 2, 3]), (0, [math.nan]),
                                  (0, [math.inf]), (2**63, [1])]:
            with pytest.raises(Exception):
                worker.submit(timestamp, values)
        assert worker.submit(0, [1, 2])[1]
        with pytest.raises(Exception, match="increasing"):
            worker.submit(0, [1])
        with pytest.raises(Exception, match="finite"):
            worker.update_scale(math.inf)
        worker.drain()
        check_accounting(worker.snapshot(), 1)


def test_native_drain_releases_gil_for_supervisor_and_times_out():
    observed = []
    errors = []
    with SummaryWorker(queue_capacity=1, result_capacity=1) as worker:
        worker.submit(0, [3.0])
        deadline = time.monotonic() + 3
        while worker.snapshot()["completed"] != 1:
            assert time.monotonic() < deadline, "priming job did not finish"
            time.sleep(0.001)
        worker._set_test_gate(True)
        worker.submit(1, [2.0])

        def supervise():
            try:
                deadline = time.monotonic() + 3
                while not worker.snapshot()["drainers"]:
                    if time.monotonic() >= deadline:
                        raise AssertionError("Python could not observe blocking drain")
                    time.sleep(0.001)
                assert worker.update_scale(3.0) == 1
                ready = worker.take_results()
                assert len(ready) == 1 and ready[0].sequence == 0 and ready[0].mean == 3.0
                for _ in range(100):
                    observed.append(worker.snapshot()["drainers"])
                worker._set_test_gate(False)
            except BaseException as error:
                errors.append(error)

        thread = threading.Thread(target=supervise, daemon=True)
        thread.start()
        results = worker.drain(timeout_ms=4000)
        thread.join(timeout=4)
        assert not thread.is_alive() and not errors
        assert observed == [1] * 100
        assert results[0].mean == 2.0 and results[0].parameter_version == 0
        worker._set_test_gate(True)
        worker.submit(2, [2.0])
        with pytest.raises(Exception, match="timed out"):
            worker.drain(timeout_ms=1)
        assert worker.snapshot()["drainers"] == 0
    assert worker.take_results()[0].mean == 6.0


def test_native_failure_in_subprocess():
    script = '''
import importlib, os
Worker = importlib.import_module(os.environ["WORKERS_MODULE"]).SummaryWorker
worker = Worker(queue_capacity=3, result_capacity=3).start()
worker._set_test_gate(True)
worker.submit(0, [1e308])
worker.submit(1, [2.0])
worker.submit(2, [3.0])
worker._set_test_gate(False)
try:
    worker.drain()
except Exception as error:
    assert "overflow" in str(error), str(error)
else:
    raise AssertionError("native failure was not propagated")
try:
    worker.stop()
except RuntimeError as error:
    assert "overflow" in str(error)
state = worker.snapshot()
assert state["failed"] == 1 and state["cancelled"] == 2, state
assert state["accepted"] == 3 and state["completed"] == 0, state
assert state["closed"] and not state["running"] and not state["active"], state
assert state["queued"] == 0 and state["drainers"] == 0, state
# The throwing nogil call restored the GIL. A new native worker still works.
with Worker() as fresh:
    fresh.submit(0, [2.0])
    assert fresh.drain()[0].rms == 2.0
print("NATIVE_FAILURE_JOINED_AND_RECOVERED")
'''
    env = dict(os.environ, WORKERS_MODULE=MODULE)
    proc = subprocess.run([sys.executable, "-c", script], env=env,
                          capture_output=True, text=True, timeout=30)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "NATIVE_FAILURE_JOINED_AND_RECOVERED" in proc.stdout
