"""The same Eigen nearest-point operation through JIT or a cached implementation."""
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
declaration = 'int nearest_point(const double*, size_t);'
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
                              include_paths=[include], name='slide_nearest_point')
loaded = time.perf_counter()
points = np.random.default_rng(7).normal(size=(10000, 3))
first = time.perf_counter()
index = cppyy.gbl.nearest_point(points, len(points))
called = time.perf_counter()
reference = int(np.argmin(np.einsum('ij,ij->i', points, points)))
if index != reference:
    raise ValueError(f'Nearest point differs: {index} != {reference}')
print(json.dumps({'mode': args.mode, 'load_s': loaded-start,
                  'first_call_s': called-first, 'nearest_index': index,
                  'cache': cache_info,
                  'pch': os.environ.get('CLING_STANDARD_PCH')}))
