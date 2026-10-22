# Compare equal-contract point-cloud processing paths

Transform XYZ points into a robot frame and compute each transformed point's
squared distance from the origin. This operation represents a small perception
stage. Inputs are deterministic synthetic clouds, from one point to a 640×480
camera frame. It is a throughput experiment, not a detector or complete ROS
pipeline.

From this repository checkout, first install the optional tools using the
[parent guide](../README.md). Then run:

```bash
pixi run bash roscon_uk_2026/slide_assets/runtime_costs/cloud/build.sh
pixi run python roscon_uk_2026/slide_assets/runtime_costs/cloud/benchmark.py
```

## Equal public contract

Every path uses the same Python validation function in [benchmark.py](benchmark.py):

- float64 NumPy arrays with aligned, C-contiguous storage;
- points `(N, 3)`, transform `(3, 4)`, output `(N, 4)`;
- writable output, with no overlap with either input;
- no implicit array conversion or output allocation during a timed call.

The output columns are transformed X, Y, Z and squared distance. Input contents
can change between calls. The prepared-buffer contract requires shape, dtype,
storage and ownership to remain valid. Concurrent mutation is excluded and the
GIL remains held during native execution.

Two cases are timed separately:

1. **Checked each call:** validate all arrays and obtain native addresses on
   every call. This represents changing frame buffers.
2. **Prepared buffers:** validate once, retain the arrays and addresses, then
   repeatedly process those fixed buffers. Every path receives this option.

The private address entry points are benchmark internals. Call them only through
`prepare()`, which checks the arrays and retains their owners. This is a small
controlled adapter, not a proposed public pointer API for the kit.

## Separate interface cost from generated-code cost

The `cppyy_shared`, pybind11 and nanobind paths call the **same shared library**,
`libcloud_kernel.so`, compiled once from [kernel.cpp](kernel.cpp). They therefore
execute the same machine code. They differ in how Python reaches that function.
The bindings receive integer addresses after the shared validation, rather than
using different library-specific ndarray validators.

The shared Python validator is an experimental control, not the fastest possible
validator. A production binding could implement those checks natively. The
prepared-buffer case separately shows the cost when validation is amortized.

Two additional cppyy paths compile the same C++ source through JIT or the kit
cache. Those measure different compiler/code-generation choices as well as the
interface. The Rust/PyO3 path implements the same scalar formula in Rust, with
no alternative linear algebra factorization. It is still a different compiler
and implementation.

NumPy uses matrix multiplication, addition and a squared-norm operation, with
an additional preallocated contiguous scratch array. This performs several
passes and copies XYZ into the specified output layout. The native loops fuse
these operations. The scratch array improves this NumPy baseline relative to
writing matrix multiplication directly into the strided XYZ output view.

## Measurement conditions

Three fresh processes per implementation, rotated order. Each processes sizes
1, 32, 1,000, 100,000 and 307,200 in both contract modes. After ten warmup calls,
five batches target approximately 10 ms each. Reported time per call is the
median of the three process medians of batch averages.

Individual p95 uses 2,000 individually timed calls per process for sizes up to
1,000, and 200 for larger clouds: 6,000 or 600 samples pooled per cell. Individual
timings include timer overhead. Data buffers and OS caches are warm. BLAS is
single-threaded. Processes are not CPU-pinned on this shared workstation.

Numerical outputs are compared to a NumPy reference before and after timing
with `rtol=1e-12`, `atol=1e-12`. [results.json](results.json) preserves raw batches,
individual timings, process summaries, startup memory and cache preparation.
The startup measurement ends when the API is loaded, before its first call.
Startup RSS excludes workload arrays. It is not peak memory.

## Results and how to choose

Measured 20 October 2026. All durations below are per operation on the whole
input, not per point. The batch-derived averages summarize three process
medians. p95 comes from separate individual timings and includes timer overhead.

**Validate on every call, then process the cloud:**

| Implementation | 1 point (µs/call) | 1,000 points (µs/call) | 307,200 points (ms/frame) | Full-frame p95 (ms/frame) |
|---|---:|---:|---:|---:|
| NumPy, reusable scratch | 3.327 | 14.137 | 4.279 | 4.488 |
| cppyy, same shared C++ kernel | 2.061 | 3.562 | 0.449 | 0.476 |
| cppyy JIT, same C++ source | 2.031 | 3.085 | 0.361 | 0.398 |
| Kit cache, same C++ source | 2.125 | 3.574 | 0.448 | 0.622 |
| pybind11, same shared C++ kernel | 2.019 | 3.510 | 0.458 | 0.499 |
| nanobind, same shared C++ kernel | 2.022 | 3.556 | 0.438 | 0.482 |
| Rust/PyO3, same formula | 2.114 | 3.667 | 0.455 | 0.499 |

**Same binary, validation once and fixed buffers reused:**

| Interface to the shared C++ kernel | 1 point (µs/call) | 1,000 points (µs/call) | 307,200 points (ms/frame) |
|---|---:|---:|---:|
| cppyy, same shared C++ kernel | 0.066 | 1.439 | 0.434 |
| pybind11, same shared C++ kernel | 0.056 | 1.430 | 0.424 |
| nanobind, same shared C++ kernel | 0.043 | 1.427 | 0.441 |

The small-case boundary costs differ by tens of nanoseconds when buffers are
prepared. With per-call validation, roughly 2 µs of checking/address handling
largely dominates the one-point case. At full-frame size, computation and memory
traffic dominate. The shared-kernel medians differ by a few percent and their
process ranges overlap. They do not establish a useful interface throughput
winner. Some individual runs were slower, particularly the kit-cache p95; raw
process timings are retained rather than discarded.

**Cost before allocating the benchmark clouds:**

| Implementation | Launch through imports/API loading (ms) | Resident process memory (MiB) |
|---|---:|---:|
| NumPy, reusable scratch | 56.5 | 33.4 |
| cppyy, same shared C++ kernel | 232.9 | 198.2 |
| cppyy JIT, same C++ source | 214.6 | 199.3 |
| Kit cache, same C++ source | 224.8 | 200.7 |
| pybind11, same shared C++ kernel | 56.9 | 33.6 |
| nanobind, same shared C++ kernel | 57.3 | 33.5 |
| Rust/PyO3, same formula | 56.8 | 33.8 |

These launch measurements exclude the first operation call, build/install time
and frame allocation. The resident figures include Python and NumPy. Workload
storage is additional: points plus output take about 16.4 MiB for a full frame;
the NumPy scratch adds about 7.0 MiB. No compiled extension maps Cling as a result
of these APIs. The earlier Eigen experiment has separate first-call and warmup
measurements; its exact startup numbers should not be transferred to this case.

### Conclusions supported by this run

- **Lowest implementation effort for this operation:** NumPy uses a few ordinary
  array operations and needs no new extension build. Its measured 4.28 ms still
  fits inside a 33.3 ms frame period at 30 Hz, although a complete perception
  pipeline needs its own budget.
- **Stable deployment with lower CPU and memory cost:** a fused compiled kernel
  takes about 0.44 ms through either binding. That is roughly 9–10 times faster
  than this NumPy implementation. The C++ extensions load in a process using
  about 34 MiB before cloud allocation, compared with about 200 MiB for cppyy.
  Existing pybind11 code does not need migration for full-frame throughput.
- **Interactive changes and native API exploration:** cppyy retains competitive
  steady-state processing while offering the live session workflow. This run
  measures runtime cost, not development-time savings.
- **Fastest steady-state kernel observed:** JIT compilation of the C++ source
  produced about 0.36 ms/frame. The shared GCC-built kernel takes about 0.44 ms.
  Different compilers/settings can generate different code. This is not proof
  that runtime compilation or cppyy's binding is inherently faster. It suggests
  checking compiler optimization before changing binding libraries.
  [Machine-code inspection and a bounded follow-up](CODEGEN.md) found two-point
  SIMD in the JIT kernel versus a scalar original GCC loop. Two GCC adjustments
  did not recover the advantage. A subsequent offline **Clang 16.0.6** build
  did: about **0.360 ms/frame** through the unchanged nanobind wrapper, matching
  the JIT. No specialization for the cloud size or transform values was observed.
- **Rust:** about 0.46 ms/frame here, with startup/memory similar to the C++
  extensions. This supplies no performance reason to port an existing C++
  implementation. Ownership, deployment and maintenance preferences are separate
  reasons to choose Rust.

The practical path for this project is to explore in a retained session, capture
correctness cases, then compile the accepted operation when startup or memory
matters. The same-binary comparison shows that this export can retain the native
throughput. It does not require a claim that the kit is always the fastest path.

The maximum numerical difference across measured cases was `1.07e-14`.
[Contract checks](contract_checks.json) record rejection of wrong dtype, shape,
layout, readonly output and overlapping storage. The same validator serves all
paths. Compiler source, binary hashes and flags are in
[build_evidence.json](build_evidence.json).
