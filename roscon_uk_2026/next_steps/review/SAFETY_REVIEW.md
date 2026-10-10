# Native integration reliability review

Reviewed on 4 October 2026. Final source hashes were recorded at 21:49 UTC.
The review covered `reverse_core`, `buffers`, `nanoflann`, and `typed_mcap`.
It examined ownership, array bounds, exception handling, shared build artifacts,
and teardown. Authors corrected their own files. The reviewer did not edit them.

Four reproducible defects were corrected: unaligned inputs in two adapters,
empty speed-query handling, and overflow in speed arithmetic. All four scope
check commands passed after the corrections. Concurrent cold nanoflann setup
remains unsupported and is now documented.

These results are reliability evidence for the declared Python wrappers. They
are not a Rust memory-safety guarantee. The raw C++ interfaces take pointers and
counts. C++ still relies on the caller supplying valid storage, sufficient
lengths, compatible declarations, and the documented concurrency conditions.

## Actual checks

These checkout commands were run from the repository root. Each command had a
process timeout. Environments came from the scope's existing Pixi manifest and
lock, except typed MCAP, which used the existing presentation ROS environment.

```bash
timeout 120s pixi run --manifest-path roscon_uk_2026/next_steps/reverse_core/pixi.toml check
timeout 90s pixi run --manifest-path roscon_uk_2026/next_steps/buffers/pixi.toml check
timeout 90s pixi run --manifest-path roscon_uk_2026/next_steps/nanoflann/pixi.toml check
timeout 90s pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros \
  python -m pytest roscon_uk_2026/next_steps/typed_mcap/test_extract.py -q
```

| Scope | Independently observed result |
|---|---|
| reverse_core | 31 passed in 0.54 s |
| buffers | 31 passed in 0.87 s |
| nanoflann | 64 oracle comparisons passed, plus ownership, empty, invalid, alignment, and teardown checks |
| typed_mcap, after speed-query corrections | 9 passed in 0.62 s |

The nanoflann alignment reproduction was rerun after correction and printed
`UNALIGNED_NORMALIZED`. A separate typed-MCAP ownership probe generated 20 poses
and two images in a temporary directory. After deleting batches and collecting
Python garbage, retained positions and pixel arrays matched saved copies. The
position view's NumPy base owned its memory; the pixel array owned its memory.

A buffers probe forced `owner.resize((100,3), refcheck=False)` before processing
a leased array. NumPy allowed the resize. The lease rejected the next call with
`RuntimeError: point storage or layout changed; close and create a new lease`.
This demonstrates a check before the call. It does not make forced reallocation
or mutation during a native call safe.

## Findings and corrections

### 1. Nanoflann passed unaligned arrays into typed native pointers

Before correction, `index.py` used `np.ascontiguousarray` for points, frames,
and confidence. It can preserve an unaligned C-contiguous array. The native
constructor and query dereference `double*` and `int64_t*` values. Correct values
on x86 do not establish valid aligned C++ accesses.

The reviewer ran this reproduction in the nanoflann Pixi environment:

```python
import numpy as np
from roscon_uk_2026.next_steps.nanoflann import index

points = np.ndarray((2, 3), dtype=np.float64,
                    buffer=bytearray(49), offset=1)
points[:] = [[0, 0, 0], [1, 0, 0]]
print(points.flags.aligned, index.positions(points).flags.aligned)
with index.Index(points, [0, 1], [1., 1.]) as tree:
    print(tree.query(points, [0, 1], k=1)["ids"])
```

Before correction, it printed `False False` and returned `[[0],[1]]`.
The author changed normalization to `np.require(..., requirements=['C','A'])`
and added unaligned point/frame/confidence cases. The reviewer reran acceptance
and verified aligned normalization. Current relevant code is
[index.py](../nanoflann/index.py#L49); the native reads are in
[native.cpp](../nanoflann/native.cpp#L18).

### 2. Typed speed queries also preserved unaligned storage

Before correction, `count_speed_above` used `np.ascontiguousarray` for timestamps
and positions. With aligned timestamps and the offset-one points above, it
passed an unaligned `double*` to native arithmetic and returned a count. With
offset-one uint64 timestamps, cppyy instead raised a buffer-conversion TypeError.

The author now checks one-dimensional uint64 timestamps and uses `np.require`
with alignment and contiguity requirements for both arrays. Regression coverage
uses offset-one arrays. The reviewer reran those tests. See
[extract.py](../typed_mcap/extract.py#L139) and
[test_extract.py](../typed_mcap/test_extract.py#L111).

### 3. An empty typed speed query raised a binding error

A valid batch with `uint64[0]` timestamps and `float64[0,3]` positions reached the
raw-pointer call. cppyy rejected the empty buffer, even though native count zero
would read no elements. The reviewer observed this TypeError in the ROS Pixi
environment. The author added a zero return after shape and finite validation.
The new regression and the full typed-MCAP checks passed.

### 4. Finite speed inputs could produce an incorrect count

The old native speed calculation squared double-precision coordinate differences.
The reviewer ran a two-record batch with timestamps `[0,10**9]`, positions
`[[0,0,0],[1e200,0,0]]`, and threshold `2e200` metres per second. It returned one
match; the correct count is zero. Squaring `1e200` overflowed to infinity.

The author now computes coordinate differences, time division, and `std::hypot`
in long double. Regression cases include the original reproducer and opposite
`-1e308`/`1e308` endpoints separated by `10**19` ns. The reviewer read the final
implementation and reran the passing cases. See
[native.cpp](../typed_mcap/native.cpp#L12).

### 5. Concurrent cold nanoflann setup remains outside the contract

Static inspection found no experiment-local lock around header fetch/checksum
and prebuild. The reused helpers write shared header and source paths before
publishing the shared library. A second cold process could see a partial header
or compile while another process rewrites the source. Atomic library replacement
alone does not serialize those earlier operations.

The reviewer did not delete or race the author's artifact directory. No actual
concurrent corruption or crash is claimed. The author documented single-process
initial setup in the [README](../nanoflann/README.md), library recipe, and results.
This limits the supported workflow; it does not fix or test concurrent cold setup.

## Ownership and bounds inspected

| Scope | Ownership and lifetime | Bounds and conditions |
|---|---|---|
| reverse_core | Python pins native estimator and normalized arrays for synchronous process. Output is a new NumPy allocation. Native Config is copied into the estimator. | Python validates dtype and `(N,)`/`(N,3)` shape and aligns arrays. Native validates every timestamp and coordinate before mutating state. One instance is single-threaded. |
| buffers | Lease retains NumPy owner and memoryview. Each native map lives only during its call. Selected output is a view of retained NumPy storage. | Typed marshaling checks alignment, dtype, layout, and output writability. Native checks flattened lengths before constructing Eigen maps. Alias mutation and resize during processing remain prohibited. |
| nanoflann | Constructor copies coordinates, frames, and confidence into native vectors. Tree references those immutable vectors. Query outputs use fresh NumPy allocations. | Wrapper checks shape, aligns input, allocates exact outputs, and uses sentinels for empty buffers. Coordinate and radius limits prevent squared-distance overflow. Query/close concurrency is unsupported. |
| typed_mcap | CDR reads message storage synchronously. Successful extraction copies data to native vectors and then to NumPy-owned storage. Failed extraction discards its local batch. | CDR scalar, string, and pixel accesses check available bytes. Image dimensions and step are checked against payload length. Unsupported schemas/layouts and decreasing log timestamps are rejected. Speed-query shapes and alignment are now checked. |

The reverse filter's invalid-batch tests verify both unchanged estimator state
and unchanged output storage. This matches source inspection: all validation
precedes its state-update loop. Typed extraction can append to its local result
before a later field fails, but no partial batch is returned or retained globally.

Reverse-core compilation uses an advisory file lock and atomic library/driver
replacement. Its in-process namespace load uses a Python lock. Typed-MCAP adapter
compilation also uses a file lock and atomic library replacement. These were
inspected; this review did not run simultaneous cold compilation for either.
Buffers uses in-memory declarations and typed `@cpp(cached=False)` wrappers.

None of these four examples intentionally starts a native background worker.
Nanoflann compilation defines `NANOFLANN_NO_THREADS`. The inspected callbacks in
the MCAP reader are native lambdas. There is no Python callback lifetime or
worker join protocol in these scopes. This is source evidence, not a claim that
all loaded dependencies create no internal threads. The separate workers scope
has its own shutdown and GIL checks in [RESULTS.md](../workers/RESULTS.md).

## Limits of the review

The checks validate concrete examples and caller contracts. They do not prove
arbitrary direct calls through raw cppyy pointers safe. Counts supplied directly
to the C++ interfaces are not independently tied to Python allocation lengths.
The safe wrapper path performs that connection.

No sanitizers, leak detector, general malformed-file fuzzer, fault-injected
allocator, concurrent cold-build stress test, free-threaded Python, or cross-ABI
deployment test was run. MCAP selected outputs scale with selected recording
size; extraction does not promise a fixed total-memory bound. Result views may
have writable aliases even when the particular returned view is read-only.
The dependency locks and checksums establish tested inputs, not an FFI type or
lifetime proof for arbitrary compiler/library changes.

## Reviewed source hashes

Hashes identify the final files inspected. Later author changes require another
review of their differences.

```text
5d8ff7ab3870f71a83bd124c6285b1fcb539b383d310c5fdcf5b99a1bbe18b0a  reverse_core/__init__.py
97e85221ae71d435a137fac98623ac3f9d7b82d6adfc9371d2ef3e3fc5191212  reverse_core/native/pose_filter.cpp
5ebdda100d75fe91ed53cb135f66f8790d81fab8025dd256853ad40ba9de0936  buffers/buffers.py
f3df6ae35659e91921fe12b78a4f90b39dea903cf816f50595a98c704f330c76  buffers/native.hpp
32ccb9961e2b2f762fd57fa1ad15d0104e583654e0e18a27e668398f7b6a5902  nanoflann/index.py
7d285be1f43411ad779d86175e2107c89693744ae3d655bb1a8275b4a9775745  nanoflann/native.cpp
c4690e8c78b073fbbe87f4ce7ed5cb2fb04a6b54052d2ea1808065b5c04e9097  typed_mcap/extract.py
2631ce627239ec0d3288bfc9fb690ce67996f6f46028b84ad7fc82cf93b5c72b  typed_mcap/native.cpp
```
