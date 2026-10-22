"""Validate and time complete calls; save reproducible measurements."""
import datetime
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import threading
import time

import numpy as np
from cppyy_kit import cpp
from demo import EIGEN_INCLUDE, inputs, numpy_scores, numpy_streamed_scores, score_particles

HERE = Path(__file__).resolve().parent


@cpp(headers=['omp.h'], link_args=['-fopenmp'], nogil=True)
def worker_count(threads: int) -> int:
    """
    int count = 0;
    #pragma omp parallel num_threads(threads)
    {
        #pragma omp single
        count = omp_get_num_threads();
    }
    return count;
    """


def measure(call, repeats=60):
    for _ in range(5):
        call()
    samples = []
    for _ in range(repeats):
        start = time.perf_counter_ns()
        call()
        samples.append((time.perf_counter_ns() - start) / 1e6)
    return {'median_ms': statistics.median(samples),
            'p95_ms': float(np.percentile(samples, 95)), 'samples_ms': samples}


def gil_probe(function, args):
    ticks = []
    stop = threading.Event()
    ready = threading.Event()

    def heartbeat():
        ready.set()
        while not stop.is_set():
            ticks.append(time.perf_counter())
            stop.wait(.001)

    thread = threading.Thread(target=heartbeat)
    thread.start()
    ready.wait()
    start = time.perf_counter()
    function(*args)
    end = time.perf_counter()
    stop.set()
    thread.join()
    return {'call_ms': (end-start)*1000,
            'heartbeat_ticks_inside_call': sum(start+.001 < t < end-.001 for t in ticks)}


def main():
    workers = {str(n): int(worker_count(n)) for n in (1, 2, 4, 8)}
    assert workers == {str(n): n for n in (1, 2, 4, 8)}, workers
    rows = []
    for count in (1000, 20000, 100000):
        positions, landmarks, ranges, scores = inputs(count)
        expected = numpy_scores(positions, landmarks, ranges)
        row = {'particles': count, 'landmarks': len(ranges), 'paths': {}}
        for n in (1, 2, 4, 8):
            score_particles(positions, landmarks, ranges, scores, n)
            np.testing.assert_allclose(scores, expected, rtol=1e-12, atol=1e-10)
            row['paths'][f'cpp_{n}_threads'] = measure(
                lambda: score_particles(positions, landmarks, ranges, scores, n))
        row['paths']['numpy'] = measure(lambda: numpy_scores(positions, landmarks, ranges))
        np.testing.assert_allclose(numpy_streamed_scores(positions, landmarks, ranges), expected, rtol=1e-12, atol=1e-10)
        row['paths']['numpy_streamed'] = measure(lambda: numpy_streamed_scores(positions, landmarks, ranges))
        rows.append(row)
        print(count, {k: round(v['median_ms'], 3) for k,v in row['paths'].items()}, flush=True)

    # Edge cases and error propagation through the GIL-release wrapper.
    for count, observations in ((0, 4), (3, 0), (1, 1)):
        args = inputs(count, observations)
        score_particles(*args, 2)
        np.testing.assert_allclose(args[-1], numpy_scores(*args[:3]), rtol=1e-12, atol=1e-10)
    args = inputs(3, 2)
    for bad in ((*args, 0), (args[0][:-1], *args[1:], 2)):
        try:
            score_particles(*bad)
        except Exception as exc:
            assert 'Expected XY' in str(exc), str(exc)
        else:
            raise AssertionError('Invalid dimensions/thread count accepted')

    held = cpp(headers=['Eigen/Core', 'omp.h', 'stdexcept'],
               include_paths=[EIGEN_INCLUDE], link_args=['-fopenmp'],
               nogil=False)(score_particles._fn)
    large = inputs(250000, 128)
    held(*large, 1)
    score_particles(*large, 1)
    gil = {'held': gil_probe(held, (*large, 1)),
           'released': gil_probe(score_particles, (*large, 1))}
    assert gil['released']['heartbeat_ticks_inside_call'] > 5, gil

    # Fresh processes, after this benchmark has populated the cache.
    startup = []
    command = [sys.executable, '-c',
               'import time; t=time.perf_counter(); from demo import inputs, score_particles; '
               'u=time.perf_counter(); a=inputs(); v=time.perf_counter(); '
               'score_particles(*a,4); w=time.perf_counter(); '
               'import json; print(json.dumps(dict(import_ms=(u-t)*1000, first_call_ms=(w-v)*1000)))']
    for _ in range(5):
        start = time.perf_counter()
        proc = subprocess.run(command, cwd=HERE, capture_output=True, text=True, check=True,
                              env=dict(os.environ, CPPYY_KIT_CACHE_DIR=str(Path.cwd()/'build/cppyy_kit_cache')))
        sample = json.loads(proc.stdout)
        sample['process_ms'] = (time.perf_counter()-start)*1000
        startup.append(sample)
    result = {'recorded_at': datetime.datetime.now().astimezone().isoformat(),
              'machine': platform.platform(), 'cpu': subprocess.check_output(['lscpu'], text=True),
              'compiler': os.environ.get('CXX'), 'numpy': np.__version__,
              'method': 'float64, 32 landmarks, 5 warmups then 60 individual calls per path; '
                        'wall time includes Python marshaling. C++ preallocates scores; NumPy allocates output and intermediates. '
                        'No fast-math. Fixed path order; OS scheduling and CPU frequency are uncontrolled.',
              'workers_observed': workers, 'rows': rows, 'gil_probe': gil,
              'cached_startup_samples': startup}
    (HERE/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    print('GIL:', gil, flush=True)
    print('Cached startup median ms:', {k: statistics.median(r[k] for r in startup) for k in startup[0]})


if __name__ == '__main__':
    main()
