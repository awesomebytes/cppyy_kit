# Buffer interoperability results

Recorded on 4 October 2026. The implementation is local to this directory.
It imports `cppyy_kit` from this checkout. It does not establish an installed
package workflow or a reusable core API.

## Commands and checks

These commands were run from the repository root unless a directory change is
shown. The environment was created with the local manifest and lock.

```bash
pixi install -m roscon_uk_2026/next_steps/buffers/pixi.toml
pixi install --locked -m roscon_uk_2026/next_steps/buffers/pixi.toml
pixi run -m roscon_uk_2026/next_steps/buffers/pixi.toml probe
pixi run -m roscon_uk_2026/next_steps/buffers/pixi.toml check
pixi run -m roscon_uk_2026/next_steps/buffers/pixi.toml demo
cd roscon_uk_2026/next_steps/buffers
pixi run env BUFFERS_KERNEL_MODULE=skeleton pytest -q --tb=short test_buffers.py -k test_filter_boundary_order_and_all_rejected
```

The final implementation check passed: **31 passed in 0.94 seconds**. The
deliberately incomplete skeleton returned exit code 1: **1 failed, 30 deselected
in 0.85 seconds**, with `logic_error: implement transform_kernel`. This proves
that the exercise starts with missing logic. It is not a fresh-agent evaluation.
The guide's two-point example was also run and returned `[[0.5, 1.0, 0.0]]` and
`1.25`.

Saved evidence:

- [check_results.txt](check_results.txt) records the passing suite.
- [skeleton_failure.txt](skeleton_failure.txt) records the incomplete skeleton.
- [probe_results.json](probe_results.json) records compiler commands, exit codes,
  header versions, and language standard.
- [setup_results.json](setup_results.json) records a warm locked install.
- [measurement.json](measurement.json) records timing, sharing, correctness,
  memory, and the check file's SHA-256.

The checks cover independent NumPy reference calculations for empty, singleton,
small, and 10,003-point batches; a dense three-axis rotation; inclusive filter
boundaries; stable order; complete rejection; arithmetic overflow filtering;
invalid parameters and scalar counts; float32, endian, alignment, Fortran,
positive and negative strides; read-only input and writable output requirements;
owner survival and release; layout invalidation; output lifetime; and CPU DLPack.
No check intentionally reads freed storage or forcibly reallocates a live view.

## Environment and compatibility

| Component | Observed version or condition |
|---|---|
| Python | 3.12.13 |
| NumPy | 2.5.1 |
| cppyy | 3.5.0 |
| cppyy backend / Cling | 1.15.3 / 6.32.8, resolved in the lock |
| Eigen package and headers | 5.0.1; header semver integer `50001` |
| pytest | 7.4.4 |
| C++ compiler | conda-forge GCC 14.3.0 |
| libgcc / libstdcxx | 15.2.0, pinned |
| Direct compiler / Cling standard | `__cplusplus == 201703`, C++17 |
| Platform | Linux x86-64, glibc 2.39; 16 logical CPUs reported |
| BLAS and OpenMP thread environment | `OPENBLAS_NUM_THREADS=1`, `OMP_NUM_THREADS=1` |

The standalone compiler probe and the Cling child process both returned zero.
The standalone `-std=c++17 -O2` compile took **1.030 seconds**. The installed
Eigen headers accept const and mutable row-major `Map` declarations under both
paths. No span, mdspan, GPU, or stream behavior was tested.

The first core import built an environment-local PCH and installed the core's
existing auto-PCH startup hook in this directory's Pixi environment. Later
measurements used that installed environment. The initial cold solve, download,
installation, and PCH build times were not captured. The recorded warm locked
install took **0.039 seconds** with packages already present. This is not cold
setup time. No global pip environment, other checkout, skill, or agent setting
was changed.

## Sharing, outputs, and correctness

The demo used seed `20261004` and **100,000 synthetic point rows**. It did not run
an ML model. Inputs were native `float64`, shape `(100000, 3)`, with byte strides
`(24, 8)`. A +90-degree z rotation and translation `[0.5, -0.2, 0.3]` metres
preceded filtering at radius **1.4 metres**. The filter retained **36,994 rows**.
Dense rotations are covered by the independent checks.

The measured transform and selected-coordinate errors against NumPy were both
**0.0 metres** for this seeded demo. The summed squared-range difference was
**3.64e-11 square metres**, within the stated `2e-13` relative tolerance. This
floating reduction is not required to be bitwise identical to NumPy.

| Boundary | Measured sharing |
|---|---|
| ndarray to typed native const pointer | Equal pointers |
| read-only ndarray to typed native const pointer | Equal pointers |
| strided input to opt-in contiguous copy | Different pointers; no shared memory |
| input to transformed output | No shared memory; separate allocation |
| transformed output to compacted filter storage | No shared memory; rows copied |
| compacted prefix to downstream energy | Equal pointers; prefix shares filter allocation |
| NumPy CPU DLPack producer to imported ndarray to native pointer | Equal pointers at both boundaries |

The NumPy producer reported DLPack device `(1, 0)`, CPU. NumPy 2.5.1 returned a
writable imported view. The producer survived deletion of its ordinary Python
reference and garbage collection. It was released after the imported lease
closed. The test also passed for a returned output view after its result owner
was deleted. This demonstrates NumPy's CPU protocol and owner chain, not another
framework's interoperability.

The input occupied **2,400,000 bytes**. Each of the two native output allocations
also occupied **2,400,000 bytes**. The selected view's logical size was
**887,856 bytes**. Its base still retains the complete filter allocation. Peak
process RSS was **299,340 KiB**, including Cling, imports, references, copies,
and the timing workload. It does not isolate native operation memory.

## Timings

The final timing run started in a new Python process after the PCH was present.
Other repository work could be active on the host. These are local observations,
not throughput or speedup guarantees. Compilation and runtime are separated.

| Startup operation | Seconds |
|---|---:|
| Python module imports, including core/cppyy | 0.3595 |
| Native header loading and declaration processing | 0.5399 |
| First transform wrapper call | 0.0646 |
| First filter wrapper call | 0.0491 |
| First energy wrapper call | 0.0422 |
| First pointer wrapper call | 0.0062 |

First wrapper calls include Cling compilation and a two-point execution. They
are not pure compiler timings. The wrappers use `cached=False`; no persistent
compiled wrapper cache is claimed.

The following values are medians of **21 calls**, in milliseconds. Preparation
includes finite-coordinate validation and closing the lease. Copies are included
where shown. The strided source has half as many rows as the full input.

| Preparation or conversion | ms |
|---|---:|
| Borrow full input, validate, close | 0.0435 |
| Borrow read-only full input, validate, close | 0.0437 |
| Reject strided input | 0.0006 |
| Copy 50,000 strided rows, validate, close | 0.1605 |
| Convert 100,000 float32 rows, validate, close | 0.1185 |
| Validate rigid parameters | 0.0234 |
| CPU DLPack import, borrow, validate, close | 0.1115 |

| Warm execution | ms |
|---|---:|
| Transform with reused output | 0.1973 |
| Filter with reused output | 0.5081 |
| Downstream energy over selected view | 0.0202 |
| Three kernels with reused output allocations | 0.8368 |
| Public process with retained input lease | 2.1523 |
| Public process with a new input lease | 2.0764 |
| NumPy vectorized transform, filter, and reduction | 6.4508 |

Warm kernel calls include typed Python/native marshaling and validation. The
combined path includes a prefix view. Public process timings additionally include
rigid-parameter checks and two output allocations. The NumPy baseline returns
the same three results with vectorized operations. It uses the locked BLAS
thread settings. New-lease and retained-lease medians vary with allocation and
host activity; their ordering does not establish an ownership cost advantage.

## Exercise and limits

[AGENT_PROMPT.txt](AGENT_PROMPT.txt) is the exact missing-logic prompt.
[skeleton.py](skeleton.py) contains two unimplemented C++ bodies. The solution
is [buffers.py](buffers.py) plus [native.hpp](native.hpp). The final independent
check SHA-256 is:

```text
673f485a60a3be449bf8e92a7d18e90f7c26e16ab37e61ded4c922263f30e049
```

No fresh agent session was run, so no agent elapsed time or success rate is
reported. Implementation checks and the deliberate skeleton failure are the
available evidence. The owning coordinator can run the supplied exercise later
without changing the checks.

The lease policy requires stable storage and no concurrent mutation. Default
NumPy resize checks were verified; forcibly bypassing them is prohibited.
Metadata changes are caught before native calls. C++ retains no pointers after
return. Kernels are synchronous and keep the GIL. The lower-level kernel
functions are implementation entry points; callers must supply disjoint output
storage. `process` allocates it. Device transfer, native asynchronous ownership,
reallocation by an external producer, cross-framework DLPack, and GPU execution
remain outside the measured scope. The policies and primary sources are in
[GUIDE.md](GUIDE.md).
