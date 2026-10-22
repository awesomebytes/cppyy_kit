"""Compare list-based Python, NumPy and native particle scoring."""
import datetime
import json
from pathlib import Path
import random
import statistics
import time

import numpy as np
from demo import inputs, numpy_scores, numpy_streamed_scores, score_particles
from plain_python import score_particles as python_scores

positions, landmarks, ranges, scores = inputs()
list_inputs = [array.tolist() for array in (positions, landmarks, ranges)]
calls = {
    'Python lists + math.hypot': lambda: python_scores(*list_inputs),
    'NumPy: broadcasting': lambda: numpy_scores(positions, landmarks, ranges),
    'NumPy: one landmark at a time': lambda: numpy_streamed_scores(positions, landmarks, ranges),
    'Eigen C++: 1 worker': lambda: score_particles(positions, landmarks, ranges, scores, 1),
    'Eigen C++: 4 workers': lambda: score_particles(positions, landmarks, ranges, scores, 4),
}
reference = numpy_scores(positions, landmarks, ranges)
for name, call in calls.items():
    for _ in range(5):
        output = call()
    np.testing.assert_allclose(scores if output is None else output, reference,
                               rtol=1e-12, atol=1e-10)
samples = {name: [] for name in calls}
rng = random.Random(19)
for _ in range(60):
    order = list(calls)
    rng.shuffle(order)
    for name in order:
        start = time.perf_counter_ns()
        calls[name]()
        samples[name].append((time.perf_counter_ns() - start) / 1e6)
rows = [{'name': name, 'median_ms': statistics.median(times),
         'p95_ms': float(np.percentile(times, 95)), 'samples_ms': times}
        for name, times in samples.items()]
slowest = max(row['median_ms'] for row in rows)
for row in rows:
    row['speedup'] = slowest / row['median_ms']
    print(row['name'], round(row['median_ms'], 3), 'ms', round(row['speedup'], 1), 'x')
Path(__file__).with_suffix('.json').write_text(json.dumps({
    'recorded_at': datetime.datetime.now().astimezone().isoformat(),
    'method': '20,000 particles, 32 landmarks, 5 warmups then 60 randomized interleaved calls per implementation. Input generation and list conversion excluded. Native output preallocated; Python and NumPy allocate output. OMP_DYNAMIC=FALSE OMP_WAIT_POLICY=PASSIVE OPENBLAS_NUM_THREADS=1. Same inputs; agreement rtol=1e-12 atol=1e-10.',
    'rows': rows}, indent=2)+'\n')
