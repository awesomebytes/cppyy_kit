"""Measure separate processes with standard PCH, Eigen PCH, and function cache."""
import datetime
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import time

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
work = Path(tempfile.mkdtemp(prefix='eigen-slide-', dir=REPO / 'build'))
env = {k: v for k, v in os.environ.items() if 'PCH' not in k}
env.update(CPPYY_KIT_NO_AUTOPCH='1', CPPYY_KIT_CACHE_DIR=str(work / 'functions'))
rows = {'plain': [], 'cached': [], 'eigen_pch': []}
transcript = []


def run(label, mode, extra=None):
    command = [sys.executable, str(HERE / 'demo.py'), mode]
    start = time.perf_counter()
    proc = subprocess.run(command, env=dict(env, **(extra or {})),
                          capture_output=True, text=True, check=True)
    elapsed = time.perf_counter()-start
    row = json.loads(proc.stdout)
    row.update(process_s=elapsed, stderr=proc.stderr)
    rows[label].append(row)
    text = f'{label}: {elapsed:.3f} s process\n{proc.stdout}{proc.stderr}'
    transcript.append(text)
    print(text, flush=True)


# Pay and record the first cache build before interleaving repeat processes.
run('cached', 'cached')
for _ in range(3):
    run('plain', 'plain')
    run('cached', 'cached')

pch = str(work / 'eigen.pch')
code = """
from cppyy_kit.autopch import generate_pch
import os, sys
from pathlib import Path
generate_pch(sys.argv[1], ['Eigen/Core', 'Eigen/Cholesky'],
             include_paths=[str(Path(os.environ['CONDA_PREFIX']) / 'include/eigen3')])
"""
start = time.perf_counter()
proc = subprocess.run([sys.executable, '-c', code, pch], env=env,
                      text=True, capture_output=True, check=True)
build_seconds = time.perf_counter()-start
print(f'Eigen PCH preparation: {build_seconds:.3f} s', flush=True)
for _ in range(3):
    run('eigen_pch', 'plain', {'CLING_STANDARD_PCH': pch})

result = {'recorded_at': datetime.datetime.now().astimezone().isoformat(),
          'method': 'Same source, input, standard PCH and disabled auto hook. '
                    'Cached mode uses a fresh function-cache directory then three repeats. '
                    'Eigen-PCH mode uses a separately generated PCH and the plain JIT source. '
                    'All processes include numerical validation. OS caches remain warm.',
          'pch_preparation_s': build_seconds, 'samples': rows,
          'repeat_median_s': {
              name: statistics.median(r['process_s'] for r in samples[-3:])
              for name, samples in rows.items()}}
(HERE / 'results.json').write_text(json.dumps(result, indent=2)+'\n')
(HERE / 'terminal.txt').write_text('\n'.join(transcript))
