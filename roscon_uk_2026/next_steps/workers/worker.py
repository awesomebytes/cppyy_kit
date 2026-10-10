"""Python supervisor for the bounded timestamped summary experiment."""
from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
import threading
import time

import cppyy
from cppyy_kit import nogil
from cppyy_kit._compile import compile_shared, compiler

_ROOT = Path(__file__).resolve().parent
_LOAD_LOCK = threading.Lock()
_LOADED = False
LOAD_METRICS = {}


def _load():
    global _LOADED
    with _LOAD_LOCK:
        if _LOADED:
            return
        started = time.perf_counter()
        digest = hashlib.sha256()
        for name in ("worker.hpp", "worker.cpp"):
            digest.update((_ROOT / name).read_bytes())
        digest.update((compiler() + " c++17 -O2 -pthread").encode())
        library = _ROOT / "build" / ("worker_" + digest.hexdigest()[:16] + ".so")
        cold = not library.exists()
        if cold:
            compile_shared(str(_ROOT / "worker.cpp"), str(library),
                           extra_flags=["-pthread"])
        compiled = time.perf_counter()
        cppyy.load_library(str(library))
        cppyy.include(str(_ROOT / "worker.hpp"))
        _LOADED = True
        LOAD_METRICS.update(cold_compile=cold,
                            compile_or_lookup_seconds=compiled - started,
                            load_declarations_seconds=time.perf_counter() - compiled)


@dataclass(frozen=True)
class Result:
    sequence: int
    parameter_version: int
    timestamp_ns: int
    count: int
    mean: float
    rms: float


class SummaryWorker:
    """Copy finite samples into a drop-newest native queue.

    Settings apply to submissions accepted after update_scale returns. Results
    retain the newest result_capacity summaries. Stop drains then joins.
    """
    def __init__(self, queue_capacity=16, result_capacity=64, max_samples=4096):
        for value in (queue_capacity, result_capacity, max_samples):
            if type(value) is not int or value <= 0:
                raise ValueError("capacities must be positive integers")
        _load()
        self._native = cppyy.gbl.timestamp_worker.Worker(
            queue_capacity, result_capacity, max_samples)
        self._max_samples = max_samples
        self._stop_lock = threading.Lock()

    def start(self):
        self._native.start()
        return self

    def submit(self, timestamp_ns, values):
        if type(timestamp_ns) is not int or not 0 <= timestamp_ns < 2**63:
            raise ValueError("timestamp_ns must be a nonnegative int64 integer")
        # Bound temporary conversion storage too. This API accepts sized inputs.
        count = len(values)
        if not 1 <= count <= self._max_samples:
            raise ValueError("sample count outside configured bounds")
        native_values = cppyy.gbl.std.vector["double"]()
        native_values.reserve(count)
        for index in range(count):
            value = float(values[index])
            if not math.isfinite(value):
                raise ValueError("samples must be finite")
            native_values.push_back(value)
        submitted = self._native.submit(timestamp_ns, native_values)
        return int(submitted.sequence), bool(submitted.accepted)

    def update_scale(self, scale):
        return int(self._native.update_scale(float(scale)))

    def snapshot(self):
        state = self._native.snapshot()
        names = ("attempted", "accepted", "rejected", "completed", "failed",
                 "cancelled", "result_evicted", "parameter_version", "queued",
                 "retained_results", "drainers", "processing_ns")
        result = {name: int(getattr(state, name)) for name in names}
        for name in ("active", "running", "closed", "gated"):
            result[name] = bool(getattr(state, name))
        result["error"] = str(state.error)
        return result

    def take_results(self):
        return [Result(int(r.sequence), int(r.parameter_version),
                       int(r.timestamp_ns), int(r.count), float(r.mean), float(r.rms))
                for r in self._native.take_results()]

    def drain(self, timeout_ms=5000):
        if type(timeout_ms) is not int or not 0 <= timeout_ms <= 60000:
            raise ValueError("timeout_ms must be an integer from 0 to 60000")
        # Native std::function captures only the still-owned native worker.
        nogil(self._native.drain_call(timeout_ms))
        return self.take_results()

    def stop(self):
        with self._stop_lock:
            nogil(self._native.stop_call())
        error = self.snapshot()["error"]
        if error:
            raise RuntimeError(error)

    def _set_test_gate(self, closed):
        """Control dequeueing for deterministic tests. Stop always opens this gate."""
        self._native.set_gate(bool(closed))

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.stop()
        except Exception:
            if exc_type is None:
                raise
        return False


def result_dicts(results):
    return [asdict(result) for result in results]
