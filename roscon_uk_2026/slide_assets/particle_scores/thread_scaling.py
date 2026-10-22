import json
import time
from pathlib import Path
import numpy as np
from demo import inputs, score_particles, numpy_scores
from benchmark import worker_count

counts = [1, 2, 4, 8, 12, 16]
rng = np.random.default_rng(19)
rows = []
for particles in (20000, 100000, 500000):
    args = inputs(particles)
    expected = numpy_scores(*args[:3])
    samples = {n: [] for n in counts}
    for n in counts:
        assert worker_count(n) == n
        for _ in range(5):
            score_particles(*args, n)
        np.testing.assert_allclose(args[-1], expected, rtol=1e-12, atol=1e-10)
    for _ in range(60):
        for n in rng.permutation(counts):
            start = time.perf_counter_ns()
            score_particles(*args, int(n))
            samples[int(n)].append((time.perf_counter_ns()-start)/1e6)
    row = {'particles': particles, 'landmarks': 32, 'threads': {
        str(n): {'median_ms': float(np.median(v)), 'p95_ms': float(np.percentile(v,95)), 'samples_ms': v}
        for n,v in samples.items()}}
    rows.append(row)
    print(particles, {n: (round(v['median_ms'],3), round(v['p95_ms'],3)) for n,v in row['threads'].items()}, flush=True)
Path(__file__).with_suffix('.json').write_text(json.dumps({
    'method': '60 calls per thread count, randomized interleaving after five warmups; 32 landmarks; OMP_DYNAMIC=FALSE OMP_WAIT_POLICY=PASSIVE OPENBLAS_NUM_THREADS=1; unpinned threads',
    'rows': rows}, indent=2)+'\n')
