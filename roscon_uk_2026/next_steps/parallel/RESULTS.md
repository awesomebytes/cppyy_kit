# Validated detection processing results

Recorded on 4 October 2026 in this checkout. The implemented operation returns
exact selection flags and integer per-frame counts. Both inline template probes
passed. The saved implementation passed 29 independent checks. No generic kit
helper or combined TBB/SIMD path was added.

The measurements do not establish a repeatable benefit over the
compiler-vectorized column baseline. At 262147 records, explicit xsimd and that
baseline take about 0.32 to 0.34 ms after preparation. Input validation and layout
conversion take hundreds of milliseconds. oneTBB's four-participant method is
faster than the serial struct method in this run, but slower than serial columns.
The host was running other experiment agents. These are local observations,
not isolated CPU measurements or deployment guarantees.

## Environment and commands

The host is an Intel Core Ultra 9 285H with 16 logical CPUs in the process affinity
set. Linux is 6.17.0-1032-oem, glibc 2.39. The independent environment contains
Python 3.12.12, cppyy 3.5.0, NumPy 2.5.3, Pydantic 2.13.5, pytest 9.1.1,
GCC/G++ 14.3.0, libgcc/libstdcxx 15.2.0, oneTBB and tbb-devel 2023.1.0,
and xsimd 14.3.0. Exact builds and transitive versions are in
[pixi.lock](pixi.lock) and [evidence/environment.json](evidence/environment.json).

Actual commands require this repository checkout:

```bash
cd roscon_uk_2026/next_steps/parallel
pixi install --locked
pixi run python build.py --force
pixi run probe
pixi run check
pixi run startup
pixi run bench
DETECTION_SOURCE=skeleton/native.cpp pixi run python -m pytest tests -q -x
pixi run check
```

The locked install into the existing environment took 0.062 s. This used warm
dependency caches. A clean network installation was not timed. Native adapter
compilation took 1.068 s with C++17, `-O3`, `-march=native`, and
`-ffp-contract=off`. See [the install record](evidence/locked_install.json) and
[the compiler record](evidence/compile.json). No shared manifest was edited.

## Startup and first use

[The startup record](evidence/startup.json) measures two new Python subprocesses
with an already built adapter library. Only the wrapper cache changes.

| Operation | Fresh wrapper cache | Reused wrapper cache |
|---|---:|---:|
| Complete subprocess wall time | 1.327 s | 0.559 s |
| Python module imports | 0.432 s | 0.377 s |
| Native adapter loading | 0.030 s | 0.029 s |
| First one-record batch preparation | 0.080 s | 0.076 s |
| First AoS bridge call | 0.355 s | 0.006 s |
| First column bridge call | 0.357 s | 0.005 s |

The xsimd call reuses the column bridge. The TBB call reuses the AoS bridge.
These process totals exclude a fresh native adapter compilation and dependency
installation. The manifest disables the package's optional auto-PCH setup.

## Conversion and memory

Seed 712 generates raw dictionaries outside the input-to-result timer. The
workloads use 64 frame bins. The two row counts exercise SIMD tails and small
versus larger native workloads. All timed outputs are compared to an
independent NumPy reference. Conversion results are in
[evidence/benchmark.json](evidence/benchmark.json).

| Preparation component | 257 records | 262147 records |
|---|---:|---:|
| Pydantic validation | 0.248 ms | 307.172 ms |
| Validated models to contiguous columns | 0.058 ms | 58.742 ms |
| Columns to prototype native struct vector | 8.858 ms | 6.438 ms |
| Complete preparation of both layouts | 9.551 ms | 418.563 ms |
| General prototype model-to-vector fill, separately timed | 5.759 ms | 167.016 ms |

The small workload's bulk fill still includes first-use ctypes/layout setup.
Its later one-shot AoS fills are smaller, as recorded in the raw evidence.
Component sums omit some Python model destruction and preparation overhead;
complete preparation and one-shot totals include them. The general fill timing
starts from validated models and excludes validation. It is not a complete
input-to-result comparison.

Positions and confidence use double columns. The frame uses an int64 column.
The generated struct is 40 bytes. Retained column storage is also 40 bytes per
record. The prototype pins columns on its vector, so the AoS route retains both
layouts even when only AoS processing is requested. At 262147 rows this is
20971760 bytes, about 20 MiB. The column-only route retains 10485880 bytes,
about 10 MiB. Output flags add 262147 bytes and counts add 512 bytes. Maximum
process RSS during the larger benchmark reached 618.723 MiB, including raw
dictionaries, temporary validated models, reference arrays, and the interpreter.
This is process peak RSS, not native resident-memory attribution.

## Warmed and one-shot processing

Each candidate has three warmups and 21 measured calls. Warmed timings include
the Python boundary, fresh result allocation, predicates, and the histogram.
They exclude validation and conversion. Candidate order is fixed. The one-shot
column routes skip struct-vector filling; AoS routes retain their pinned source
columns. Startup and input generation are excluded from all one-shot totals.

| Candidate | 257 rows median | 262147 rows median | 262147 rows p95 | 262147 rows one-shot total |
|---|---:|---:|---:|---:|
| Serial struct vector | 0.010 ms | 1.260 ms | 1.960 ms | 481.357 ms |
| Serial compiler-vectorized columns | 0.013 ms | 0.338 ms | 0.389 ms | 410.049 ms |
| xsimd columns | 0.014 ms | 0.322 ms | 0.392 ms | 411.552 ms |
| oneTBB, 1 participant | 0.011 ms | 0.730 ms | 1.553 ms | 411.411 ms |
| oneTBB, 2 participants | 0.011 ms | 0.664 ms | 1.502 ms | 416.584 ms |
| oneTBB, 4 participants | 0.010 ms | 0.619 ms | 1.251 ms | 401.054 ms |

The one-shot totals are single observations. Different validation times dominate
their ordering. The raw record includes each one-shot preparation, runtime,
validation, column extraction, and vector-fill timer; no complete speedup is
inferred from the warmed comparison. The small one-shot totals range from
0.613 to 0.783 ms.

GCC reported the serial column loop vectorized with both 32-byte and 16-byte
vectors. This baseline was not compiled with vectorization disabled. The xsimd
batch has four double lanes on this host.
[evidence/vectorization.txt](evidence/vectorization.txt) records the actual
compiler decisions.

The TBB predicate writes disjoint ranges. Its histogram remains serial, which
limits scaling. Instrumented larger runs observed 1, 2, and 4 simultaneously
active range bodies under the corresponding limits. Small runs observed only
one because 257 rows are below the 4096-row grain size. Instrumentation is
disabled in the measured warmed calls. These observations bound active range
bodies, not all OS threads or other native libraries.

## Correctness and exercise status

[tests/test_processing.py](tests/test_processing.py) uses an independent Python
reference. It checks all methods, repeated runs, 1/2/4 participants, empty data,
every tail length up to three SIMD batches, exact threshold and sphere boundaries,
invalid/nonfinite/out-of-range inputs, revalidation of constructed models,
source mutation and garbage collection, concurrent calls, runtime isolation
from Python validation, and general versus bulk conversion. Flags and uint64
counts match exactly. No floating reduction tolerance is needed.

The final successful command and count are in [evidence/tests.txt](evidence/tests.txt).
The unchanged check file SHA256 is in [evidence/checks.sha256](evidence/checks.sha256).
The initial skeleton compiles and fails with its missing oneTBB operation, as
recorded in [evidence/skeleton_failure.txt](evidence/skeleton_failure.txt). The
saved solution was rebuilt and rechecked afterward.

Both subprocess template probes succeeded with no stderr. Their source, exit
codes, and elapsed times are retained in [evidence/probes.json](evidence/probes.json).
The initial wrapper recipe failures and fixes are retained in
[evidence/FAILURES.md](evidence/FAILURES.md). No dependency ABI failure occurred
with the pinned toolchain.

[The exact prompt](AGENT_PROMPT.md), [the guide](GUIDE.md), and
[the missing-logic skeleton](skeleton/native.cpp) are provided. A fresh-agent
exercise and installed-package proof remain unevaluated. No skills were
installed and no agent settings were changed by this scope.
