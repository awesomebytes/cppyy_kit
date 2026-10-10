# nanoflann experiment results

Measured on 4 October 2026 in this repository checkout on Linux x86-64,
kernel 6.17.0-1032-oem and glibc 2.39. Raw measurements are in
[measurement_cold.json](measurement_cold.json) and
[measurement_warm.json](measurement_warm.json). Timings are local observations
under shared machine load. They are not a real-time or general performance claim.

## Reproduction and dependencies

Commands below were run in this checkout. Pixi task commands set their working
directory to this experiment's manifest directory.

```bash
pixi install --locked --manifest-path roscon_uk_2026/next_steps/nanoflann/pixi.toml
pixi run --manifest-path roscon_uk_2026/next_steps/nanoflann/pixi.toml probe
pixi run --manifest-path roscon_uk_2026/next_steps/nanoflann/pixi.toml check
pixi run --manifest-path roscon_uk_2026/next_steps/nanoflann/pixi.toml demo
pixi run --manifest-path roscon_uk_2026/next_steps/nanoflann/pixi.toml benchmark
```

The final compiler-source change produced a new adapter cache key, so the first
benchmark run compiled the current adapter. The following run reused it. Their
outputs were copied from `build/benchmark.json` into the two measurement files.
The `cached: true` field in both records describes the **load after prebuild**.
The separate prebuild timing distinguishes the compilation run from the reuse run.

Pinned dependencies: Python 3.12.13, cppyy 3.5.0, cppyy-cling 6.32.8,
NumPy 2.5.3, SciPy 1.17.1, GCC/G++ 14.3.0, libgcc/libstdcxx 15.2.0.
Compiler identity was `x86_64-conda-linux-gnu-c++ (conda-forge gcc 14.3.0-20)`.
The environment contains published cppyy-kit 0.3.0. The scripts explicitly import
the current checkout's `cppyy_kit`, whose path is recorded in the JSON.
This is a checkout example, not an installed-package compatibility proof.
The [current pixi.lock](pixi.lock) was updated on 10 October for explicit source
activation and removes the unused installed 0.3 package. Its transitive package
hashes describe the current rehearsal, rather than the original measurement
environment. An exact original lock snapshot was not retained for this
experiment. The recorded versions and raw measurements above remain historical.
Current acceptance results are in
[CURRENT_REHEARSAL.md](../../CURRENT_REHEARSAL.md).

No nanoflann header resolved in the active or shared checkout Pixi environments
or system include directories. A broader Rattler-cache search found v1.6.1
copies in unrelated cached environments; those paths are outside the active
prefix and were not registered. This recipe pins v1.6.3.
`require` fetched the [v1.6.3 header](https://github.com/jlblancoc/nanoflann/blob/v1.6.3/include/nanoflann.hpp)
to ignored `build/vendor`. SHA256:

```text
e47f12ae2fc339cd2eb9ca148af837bd1025ff7e301ecf1860675ff1c73dd071
```

Resolved header content is rehashed on every run, including cache hits.
A cached `pixi install --locked` check took 0.04 s using `/usr/bin/time`.
Initial environment solving, package download, first header fetch and first
Cling PCH setup were successful but not independently timed. The compile-cold
measurement uses an already available header and Cling runtime/PCH.

## Correctness and native execution

`probe` passed native class construction and a filtered first query. It also
checked nanoflann's version macro as 0x163. `nm -C` on the adapter `.so` showed
nanoflann `buildIndex`, `divideTree` and
`searchLevel<recorded_nn::Index::Impl::Selection>` symbols. The retained C++
class executes the actual nanoflann tree, not a Python nearest-neighbor loop.

`check` returned:

```text
PASS: 64 oracle comparisons; eligibility, distance units, owned lifetime, empty batches, invalid inputs, repeated close and teardown
```

The independent oracle uses NumPy brute force and `lexsort`, without tree code.
Fixtures include exact integer-coordinate ties across multiple leaves, duplicate
points, radius zero, fewer than k eligible points, empty indexed/query arrays,
confidence/radius/frame exclusions, and the first eligible point beyond the
nearest 50 ineligible observations. It checks squared-distance values and
Euclidean distance conversion separately. Input mutation, resizing, deletion
and garbage collection leave results unchanged. Unaligned coordinate, frame and
confidence inputs are normalized to aligned storage. Output arrays survive another
query and index close. Invalid shape, finite/range values, frame dtype and k
are rejected. Repeated close and 30 construction/query/teardown cycles pass.

The benchmark checks 32 queries against the independent oracle and all 2,000
queries against both SciPy filtered compositions. Direct unfiltered kNN also
matches SciPy indices and squared distances for all 2,000 queries. These random
continuous positions have no deliberate ties; the dedicated oracle fixtures
establish the documented original-index tie policy.

The missing-logic skeleton fails the same checks with the expected
`NotImplementedError: Build and retain the native index`:

```bash
NN_SOLUTION=skeleton.py pixi run --manifest-path roscon_uk_2026/next_steps/nanoflann/pixi.toml check
sha256sum roscon_uk_2026/next_steps/nanoflann/acceptance.py
```

Acceptance SHA256 is
`cdccbef7fe2be5e15cb111b9d2d5af0433b0f973fd5d02ade0119211667923af`.
The coordinator recorded a fresh **gpt-6.1-sol**, high-reasoning completion on
4 October 2026. Its [report](../../next_steps_evaluation/nanoflann_sol_high_01/report.json)
records 513.4 s, unchanged acceptance checks, passing acceptance and zero human
logic repairs. The scope was this checkout's scaffolding and documented helpers,
with saved solutions excluded. The [exact evaluation prompt](../../next_steps_evaluation/nanoflann_sol_high_01/prompt.txt)
and [acceptance output](../../next_steps_evaluation/nanoflann_sol_high_01/acceptance.txt)
are preserved with the candidate and events. This evaluates Sol; Luna was not
evaluated. The existing recipe was used before the minor dtype and compiler
guidance improvements recorded below. No new completion run was needed for
those wording changes.

The agent corrected its initial `long long*` signature to `std::int64_t*` for
NumPy int64 buffers and located the existing compiler-selection helper after
searching installation paths. The recipe now specifies exact pointer types,
shows `_compile.compiler()` for diagnostics, and distinguishes that internal
helper from the recommended top-level setup/compile interfaces. The saved
solution and acceptance checks were not changed by this documentation update.

## Costs and useful comparison

The benchmark retains 50,000 random positions in [-1,1] m on each axis and
queries 2,000 positions, with k=8. Seed is 4026. Selection requires confidence
at least 0.6, absolute frame difference greater than 128, and distance at most
0.25 m. Native computation and SciPy use one calling worker. Query values below
are medians of five warmed runs from the warm measurement file.

| Operation | Measured time |
|---|---:|
| Compile-cold adapter import, including prebuild | 996.58 ms |
| Native adapter prebuild on compile-cold run | 703.61 ms |
| Adapter import with existing compiled artifact | 379.25 ms |
| Existing artifact prebuild check | 0.21 ms |
| Load library and register declarations | 33.55 ms |
| Header discovery plus checksum, cached header | 0.23 ms |
| Index input validation/conversion | 0.71 ms |
| Native owned input copy plus tree build | 5.39 ms |
| Constructor call, including first signature wrapper | 9.34 ms |
| SciPy copied-data tree build | 10.55 ms |
| First filtered native call, including wrapper | 7.07 ms |
| Warmed native query, selection and centroid computation | 3.18 ms |
| Warmed native call including boundary crossing | 3.19 ms |
| Query validation plus output allocation | 0.023 ms |
| Output conversion | 0 ms, direct native writes into NumPy arrays |
| SciPy adaptive batched kNN search | 7.31 ms |
| SciPy adaptive Python selection/centroid | 24.18 ms |
| SciPy adaptive composed total | 31.49 ms |
| SciPy batched radius search plus candidate lists | 46.70 ms |
| SciPy radius-based Python selection/centroid | 50.99 ms |
| SciPy radius-based composed total | 97.74 ms |
| Native unfiltered query call, with centroid | 1.95 ms |
| SciPy direct unfiltered kNN, without centroid | 2.32 ms |
| NumPy brute force for only 32 queries | 25.56 ms |

The native timer starts after C++ value validation. The call timer includes
that validation and the Python/native crossing. Python conversion timing includes
shape checks, layout normalization and output allocation. The fixture is already
contiguous aligned float64/int64, so it does not measure strided or unaligned
array copying. The warm measurement was rerun after adding aligned-input
normalization; the compile-cold record uses the same native source before that
Python normalization correction. Construction
copies input into native vectors and includes that copy in native build time.
The direct-output conversion value records absence of a conversion step, not
absence of memory allocation or writes.

SciPy's existing [cKDTree.query](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.cKDTree.query.html)
is a useful direct native binding. The adaptive comparison begins with 32
candidates per query, doubles for unresolved rows, and continues at a cutoff tie
until index tie resolution is possible. It applies the same metadata and spatial
policy and centroid rule. The radius comparison uses native
[query_ball_point](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.cKDTree.query_ball_point.html)
to gather candidates, then NumPy filtering and sorting. The adaptive baseline
avoids enumerating every point in the radius and was substantially faster here.
Both baselines include necessary Python result selection and centroid work.

The direct unfiltered comparison is close. The larger composed-operation
difference measures native filtering during traversal and fused centroid
calculation. It does not establish that nanoflann generally outperforms SciPy
or every possible optimized SciPy implementation. Metadata preprocessing and
different workloads could change the comparison. Timing medians of components
need not add to the median of their total.

Process RSS was 238,380 KiB before first native construction and 241,660 KiB
after it. Native input storage alone accounts for 1,953.125 KiB. The RSS delta
also includes allocations and first-call wrappers; it is not isolated tree
memory. Whole-process peak RSS was 282,900 KiB, including both libraries,
candidate arrays, outputs and oracle work.

## Failures and limits

The first direct Cling template probe failed with unresolved
`__emutls_v._ZSt11__once_call` and `__emutls_v._ZSt15__once_callable` symbols.
The saved implementation uses serial nanoflann builds through
`NANOFLANN_NO_THREADS`, native prebuild, and bodiless Cling declarations.
This keeps optional async implementation details out of Cling's call path.
Empty NumPy pointer arguments also failed conversion. Typed one-element
sentinels plus explicit zero lengths handle them without any native dereference.
An alignment review also identified that C-contiguous buffers may still be
unaligned. All numeric input normalization now uses NumPy `require` with C and
aligned requirements. These fixes are confined to this experiment. No core
package changes were needed.

Initial setup, header fetch and adapter compilation must run in one process.
The reused helper implementations write fixed shared source paths without a
cross-process lock. Concurrent cold setup is unsupported and was not tested.

Synthetic data establishes the operation and contract. Recorded-data provenance
has not been tested. Exactness means epsilon-zero KD-tree traversal using double
arithmetic. Equal computed distances use original indices; near-equal real
distances can round differently. No concurrent query/close, GIL release,
thread-safety, GPU, dynamic data updates, or production deployment is claimed.
The bounded sorted result vector has O(k) insertion cost. Large k and strongly
selective metadata can make tree traversal expensive. This is a new-library
recipe and custom composition example, not a full general-purpose nanoflann kit.
