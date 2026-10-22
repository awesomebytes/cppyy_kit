"""The same Eigen joint-step operation through JIT or a cached implementation."""
import argparse
import json
import os
from pathlib import Path
import time

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['plain', 'cached'])
args = parser.parse_args()
source = Path(__file__).with_name('step.cpp').read_text()
declaration = 'void joint_step(const double*, const double*, double, double*);'
include = str(Path(os.environ['CONDA_PREFIX']) / 'include/eigen3')
start = time.perf_counter()
cache_info = None
if args.mode == 'plain':
    import cppyy
    cppyy.add_include_path(include)
    cppyy.cppdef(source)
else:
    from cppyy_kit import cppyy, cppdef_cached
    cache_info = cppdef_cached(source, decls=declaration,
                              include_paths=[include], name='slide_joint_step')
loaded = time.perf_counter()
J = np.random.default_rng(7).normal(size=(6, 7))
dx = np.array([.01, -.02, .03, .005, -.004, .002])
dq = np.empty(7)
first = time.perf_counter()
cppyy.gbl.joint_step(J, dx, .05, dq)
called = time.perf_counter()
reference = J.T @ np.linalg.solve(J @ J.T + .05**2 * np.eye(6), dx)
error = float(np.max(np.abs(dq-reference)))
if not np.allclose(dq, reference, rtol=1e-10, atol=1e-12):
    raise ValueError(f'Joint increment differs from reference: {error}')
print(json.dumps({'mode': args.mode, 'load_s': loaded-start,
                  'first_call_s': called-first,
                  'max_error': error, 'joint_increment': dq.tolist(),
                  'cache': cache_info,
                  'pch': os.environ.get('CLING_STANDARD_PCH')}))
