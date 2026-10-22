# Reuse an Eigen joint-step computation across Python processes

This repository example computes `dq = J.T @ solve(J @ J.T + damping² I, dx)`
for a 6-by-7 Jacobian. Python supplies the arrays. Eigen provides the matrix
operations and LDLT factorization. It is a small operation used in damped
least-squares task control.

From the repository root:

```bash
pixi run python roscon_uk_2026/slide_assets/eigen_startup/benchmark.py
```

The runner creates a dedicated cache under `build/eigen-slide-*`, executes both
implementations, prepares an Eigen PCH, and records three fresh processes per
prepared path. It verifies each output against NumPy with `rtol=1e-10` and
`atol=1e-12`. [results.json](results.json) contains all samples, PCH paths, cache
hit/miss metadata, and numerical errors. [terminal.txt](terminal.txt) contains
the captured output.

## Same native source, different preparation

The complete operation is [step.cpp](step.cpp). Both paths load that same source:

```python
source = Path('step.cpp').read_text()
include = str(Path(os.environ['CONDA_PREFIX']) / 'include/eigen3')
```

Vanilla cppyy:

```python
import cppyy
cppyy.add_include_path(include)
cppyy.cppdef(source)
```

With cppyy_kit:

```python
from cppyy_kit import cppyy, cppdef_cached
cppdef_cached(
    source,
    decls='void joint_step(const double*, const double*, double, double*);',
    include_paths=[include],
    name='slide_joint_step',
)
```

Both call `cppyy.gbl.joint_step(J, dx, .05, dq)`. The kit compiles a shared library
on the first cache miss. Subsequent processes load it and expose the small
function declaration to Cling. The Eigen implementation stays in the compiled
library. The cache reports `miss-built` on preparation and `cached: true` on all
three subsequent launches.

The separate PCH path uses the kit's `generate_pch` helper for `Eigen/Core` and
`Eigen/Cholesky`, then runs the vanilla code with that PCH selected before Python
starts. It isolates reuse of parsed headers from reuse of compiled code.

## Measured on this machine, 20 October 2026

| Path | Whole-process median, 3 launches |
|---|---:|
| Vanilla cppyy, its own existing standard PCH | 1.695 s |
| Same code with a prepared Eigen PCH | 1.512 s |
| cppyy_kit, prepared function cache and standard PCH | 0.319 s |

The first function-cache run took **3.948 s**, including JIT and shared-library
preparation. Separate Eigen PCH preparation took **1.743 s**. These preparation
costs are recorded separately from the prepared-path medians.

Every process imports NumPy, prepares input, loads or compiles the operation,
executes it, checks its result and exits. Maximum absolute difference from NumPy
was below `2e-16`. The kit cache reduces repeat process time by about **5.3×**
for this example. These are startup-to-result measurements, not warmed solver
throughput or controller-loop timings.

The runner removes inherited PCH variables and disables the kit startup hook
for all cases. Baseline and function-cache runs use cppyy's existing standard
PCH. The third path explicitly selects the separately prepared Eigen PCH.
OS caches remain warm. The source and input are identical across paths.

## Show it in a terminal

Enter `pixi shell` from the repository root, then run:

```bash
export CPPYY_KIT_NO_AUTOPCH=1
unset CLING_STANDARD_PCH
export CPPYY_KIT_CACHE_DIR="$PWD/build/eigen-stage-functions"
python roscon_uk_2026/slide_assets/eigen_startup/demo.py plain
python roscon_uk_2026/slide_assets/eigen_startup/demo.py cached
python roscon_uk_2026/slide_assets/eigen_startup/demo.py cached
```

Each command prints its load time, first-call time, numerical result and cache
status. Use `/usr/bin/time -f '%e s elapsed'` before `python` for whole-process
timing. The benchmark runner is the reproducible source of the slide figures.
