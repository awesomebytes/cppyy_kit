"""Independent scalar reference for the public contract, with no native imports."""

import math

import numpy as np


class ReferenceFilter:
    def __init__(self, config):
        self.config = dict(config)
        self.reset()

    def reset(self):
        self.last = None
        self.position = None
        self.samples = 0
        self.gaps = 0

    def process(self, timestamps, positions):
        if not isinstance(timestamps, np.ndarray) or not isinstance(positions, np.ndarray):
            raise TypeError("NumPy arrays required")
        if timestamps.dtype != np.dtype("int64") or positions.dtype != np.dtype("float64"):
            raise TypeError("int64 timestamps and float64 positions required")
        if timestamps.ndim != 1 or positions.shape != (len(timestamps), 3):
            raise ValueError("expected N timestamps and N Cartesian triples")
        if not np.isfinite(positions).all():
            raise ValueError("positions must be finite")
        prior = self.last
        for raw in timestamps:
            timestamp = int(raw)
            if prior is not None and timestamp <= prior:
                raise ValueError("timestamps must increase")
            prior = timestamp

        output = []
        for timestamp, observation in zip(timestamps, positions):
            timestamp = int(timestamp)
            sample = list(map(float, observation))
            if self.last is None:
                self.position = sample
            else:
                # Python integer subtraction avoids NumPy int64 wraparound.
                dt = (timestamp - self.last) / 1_000_000_000
                if dt > self.config["max_gap_s"]:
                    self.position = sample
                    self.gaps += 1
                else:
                    weight = 1 - math.exp(-dt / self.config["tau_s"])
                    self.position = [old + weight * (new - old)
                                     for old, new in zip(self.position, sample)]
            self.last = timestamp
            self.samples += 1
            output.append(self.position.copy())
        return np.asarray(output, dtype=np.float64).reshape(-1, 3)

    def snapshot(self):
        return {
            "initialized": self.last is not None,
            "last_timestamp_ns": self.last,
            "filtered_position_m": None if self.position is None else self.position.copy(),
            "samples_processed": self.samples,
            "gap_resets": self.gaps,
            "config": dict(self.config),
        }


def assert_snapshot(actual, expected):
    actual = dict(actual)
    expected = dict(expected)
    left = actual.pop("filtered_position_m")
    right = expected.pop("filtered_position_m")
    assert actual == expected
    if right is None:
        assert left is None
    else:
        np.testing.assert_allclose(left, right, rtol=1e-12, atol=1e-9)
