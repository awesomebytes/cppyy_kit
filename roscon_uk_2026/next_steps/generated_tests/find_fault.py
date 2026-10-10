"""Find and shrink a reset failure in the separate faulty fixture."""

import json
from pathlib import Path
import time

import numpy as np
from hypothesis import find, settings, strategies as st

from faulty_fixture import MissingResetFilter

TRIPLE = st.tuples(*(st.integers(-10, 10) for _ in range(3)))
CASE = st.tuples(TRIPLE, TRIPLE, st.integers(0, 1_000_000_000),
                 st.integers(1, 500_000_000))
CONFIG = {"tau_s": 0.08, "max_gap_s": 0.5, "frame_id": "world"}


def replay(case):
    initial, subsequent, start, step = case
    instance = MissingResetFilter(CONFIG)
    try:
        instance.process(np.array([start], dtype=np.int64),
                         np.array([initial], dtype=np.float64))
        instance.reset()
        return instance.process(np.array([start + step], dtype=np.int64),
                                np.array([subsequent], dtype=np.float64))
    finally:
        instance.close()


def main():
    started = time.perf_counter()
    minimized = find(CASE, lambda case: not np.allclose(replay(case), [case[1]],
                                                      rtol=1e-12, atol=1e-9),
                     settings=settings(max_examples=500, deadline=None,
                                       derandomize=True, database=None))
    initial, subsequent, start, step = minimized
    witness = {
        "fixture": "MissingResetFilter",
        "fault": "compiled C++ fixture reset does not forward to the native filter",
        "config": CONFIG,
        "operations": [
            {"method": "process", "timestamps_ns": [start], "positions_m": [initial]},
            {"method": "reset"},
            {"method": "process", "timestamps_ns": [start + step], "positions_m": [subsequent]},
        ],
        "expected_last_output_m": [subsequent],
        "observed_last_output_m": replay(minimized).tolist(),
    }
    path = Path(__file__).with_name("minimized_fault.json")
    path.write_text(json.dumps(witness, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"saved": path.name, "minimized_case": minimized,
                      "elapsed_s": time.perf_counter() - started}))


if __name__ == "__main__":
    main()
