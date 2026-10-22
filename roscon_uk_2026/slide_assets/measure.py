"""Separate warmed computation from process startup for the slide examples."""
import argparse
import json
import statistics
import time

import numpy as np


def run(function):
    parser = argparse.ArgumentParser()
    parser.add_argument('--tiny', action='store_true')
    args = parser.parse_args()
    if args.tiny:
        print(function(np.array([1., 2., 3.], dtype=np.float64)))
        return
    x = np.arange(1_000_000, dtype=np.float64) % 100
    expected = 3_283_500_000.0
    result = function(x)  # Resolve/compile and warm the callable before timing.
    if result != expected:
        raise ValueError(f'{result=} differs from {expected=}')
    durations = []
    for _ in range(7):
        start = time.perf_counter_ns()
        result = function(x)
        durations.append((time.perf_counter_ns() - start) / 1e6)
    print(json.dumps({'elements': x.size, 'result': result,
                      'samples_ms': durations,
                      'median_ms': statistics.median(durations)}))
