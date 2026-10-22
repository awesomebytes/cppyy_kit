"""Measure first-build and cached startup, with peak resident memory."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
CHILD = '''
import json, resource, time
rss = lambda: resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
baseline = rss()
t = time.perf_counter()
from demo import inputs, score_particles
u = time.perf_counter()
import_rss = rss()
a = inputs()
v = time.perf_counter()
score_particles(*a, 4)
w = time.perf_counter()
print(json.dumps(dict(import_ms=(u-t)*1000, first_call_ms=(w-v)*1000,
                      baseline_peak_mib=baseline, import_peak_mib=import_rss,
                      call_peak_mib=rss())))
'''

with tempfile.TemporaryDirectory(prefix='particle-cache-', dir=REPO/'build') as cache:
    rows = []
    for i in range(6):
        start = time.perf_counter()
        proc = subprocess.run([sys.executable, '-c', CHILD], cwd=HERE,
                              env=dict(os.environ, CPPYY_KIT_CACHE_DIR=cache),
                              capture_output=True, text=True, check=True)
        row = json.loads(proc.stdout)
        row.update(process_ms=(time.perf_counter()-start)*1000,
                   mode='first_build' if i == 0 else 'cached', stderr=proc.stderr)
        rows.append(row)
    (HERE/'startup.json').write_text(json.dumps(rows, indent=2)+'\n')
    print(json.dumps(rows, indent=2))
