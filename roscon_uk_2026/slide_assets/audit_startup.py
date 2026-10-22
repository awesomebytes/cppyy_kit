"""Audit startup with explicit PCH isolation and phase timings."""
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import time

ASSETS = Path(__file__).resolve().parent
REPO = ASSETS.parents[1]
CHILD = """
import json, os, time
start = time.perf_counter()
selected_at_start = os.environ.get('CLING_STANDARD_PCH')
import numpy as np
numpy_done = time.perf_counter()
{imports}
imports_done = time.perf_counter()
{definition}
definition_done = time.perf_counter()
x = np.array([1., 2., 3.], dtype=np.float64)
result = {call}
call_done = time.perf_counter()
assert result == 14.0
print(json.dumps(dict(result=result, pch_at_start=selected_at_start,
    pch_after_import=os.environ.get('CLING_STANDARD_PCH'),
    numpy_ms=(numpy_done-start)*1000,
    imports_ms=(imports_done-numpy_done)*1000,
    definition_ms=(definition_done-imports_done)*1000,
    first_call_ms=(call_done-definition_done)*1000)))
"""
plain = {'imports': 'import cppyy', 'definition': '''cppyy.cppdef("""
double sum_sq(const double* x, size_t n) {
    double total = 0;
    for (size_t i = 0; i < n; ++i) total += x[i] * x[i];
    return total;
}
""")''', 'call': 'cppyy.gbl.sum_sq(x, x.size)'}
kit = {'imports': 'from cppyy_kit import cpp\nfrom cppyy_kit.numpy_types import NDArray',
       'definition': '''@cpp
def sum_sq(x: NDArray[np.float64]) -> float:
    """
    double total = 0;
    for (size_t i = 0; i < x_size; ++i) total += x[i] * x[i];
    return total;
    """
''', 'call': 'sum_sq(x)'}
cache = tempfile.mkdtemp(prefix='startup-audit-', dir=REPO / 'build')
base = {k: v for k, v in os.environ.items() if 'PCH' not in k}
base['CPPYY_KIT_CACHE_DIR'] = cache
cases = {
    'cppyy_stock_pch': (plain, {'CPPYY_KIT_NO_AUTOPCH': '1'}),
    'cppyy_no_pch': (plain, {'CPPYY_KIT_NO_AUTOPCH': '1', 'CLING_STANDARD_PCH': 'none'}),
    'cppyy_with_kit_hook': (plain, {}),
    'kit': (kit, {}),
}
report = {'method': 'Remove inherited PCH variables before each child; opt out of kit hook for vanilla. '
                   'One initial round then three interleaved rounds. Existing OS caches. '
                   'Kit function cache initially empty. Process elapsed uses parent perf_counter.',
          'samples': {name: [] for name in cases}}
for repeat in range(4):
    for name, (body, settings) in cases.items():
        env = dict(base, **settings)
        start = time.perf_counter()
        p = subprocess.run([sys.executable, '-c', CHILD.format(**body)],
                           env=env, text=True, capture_output=True)
        elapsed = time.perf_counter() - start
        row = json.loads(p.stdout) if p.returncode == 0 else {'returncode': p.returncode, 'stdout': p.stdout}
        row.update(process_s=elapsed, stderr=p.stderr)
        report['samples'][name].append(row)
        print(name, repeat, json.dumps(row), flush=True)
report['repeat_median_s'] = {
    name: statistics.median(row['process_s'] for row in rows[1:])
    for name, rows in report['samples'].items()
    if all('result' in row for row in rows)}
(ASSETS / 'startup_audit.json').write_text(json.dumps(report, indent=2) + '\n')
