"""Missing implementation. Read ../README.md and ../PROMPT.md explicitly."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Result:
    sequence: int
    parameter_version: int
    timestamp_ns: int
    count: int
    mean: float
    rms: float


class SummaryWorker:
    def __init__(self, queue_capacity=16, result_capacity=64, max_samples=4096):
        raise NotImplementedError("construct bounded native storage")

    def start(self):
        raise NotImplementedError("start the native worker and return self")

    def submit(self, timestamp_ns, values):
        raise NotImplementedError("copy samples and return sequence, accepted")

    def update_scale(self, scale):
        raise NotImplementedError("synchronize configuration and return version")

    def snapshot(self):
        raise NotImplementedError("return counters and lifecycle state as a dict")

    def take_results(self):
        raise NotImplementedError("retrieve retained Result records")

    def drain(self, timeout_ms=5000):
        raise NotImplementedError("wait natively without GIL and take results")

    def stop(self):
        raise NotImplementedError("close, finish or cancel, and join without GIL")

    def _set_test_gate(self, closed):
        raise NotImplementedError("control native dequeueing; stop opens gate")

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, traceback):
        raise NotImplementedError("join even when the context body raises")
