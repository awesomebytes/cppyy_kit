# Measure the resident cost of interactive C++

For choosing a processing path, also read the
[equal-contract point-cloud comparison](cloud/README.md). It gives every path
the same validation and uses one shared C++ binary to isolate interface costs.
The Eigen measurements below compare the original interfaces, including their
different argument-checking costs.

From this repository checkout, run:

```bash
mkdir -p build/runtime-cost-tools
cp roscon_uk_2026/slide_assets/runtime_costs/toolchain.toml build/runtime-cost-tools/pixi.toml
pixi install --manifest-path build/runtime-cost-tools/pixi.toml
uv pip install --python .pixi/envs/default/bin/python \
  --target build/runtime-cost-tools/python nanobind==2.12.0
pixi run python roscon_uk_2026/slide_assets/runtime_costs/measure.py
```

The runner builds pybind11 and nanobind extensions with CMake and a Rust
extension with Cargo, then starts
three fresh processes per path, interleaved. It disables the
kit's auto-PCH hook and clears inherited PCH settings. cppyy uses its existing
standard PCH. A separate invocation prepares the function cache before the
measured cached runs. OS caches remain warm.

## Results on this machine

Measured 20 October 2026. Values are medians of three runs. All paths expose
`joint_step(jacobian, delta, damping, output)` and receive identical float64
NumPy inputs.

| Implementation | RSS after first call | Operation load/import | First call | Extension file |
|---|---:|---:|---:|---:|
| Eigen through vanilla cppyy | 367.1 MiB | 1251 ms | 44.73 ms | Runtime compilation |
| Eigen through prepared kit cache | 203.4 MiB | 14.68 ms | 3.05 ms | Cached implementation plus Cling |
| Eigen + pybind11 | **36.9 MiB** | 0.330 ms | 0.108 ms | 178.6 KiB |
| Eigen + nanobind | **36.7 MiB** | 0.270 ms | 0.012 ms | 99.5 KiB |
| Rust nalgebra + PyO3 | **37.0 MiB** | 0.268 ms | 0.113 ms | 415.0 KiB |

The cppyy import adds approximately **164 MiB RSS** to this harness. Its import
stage takes **148 ms**; accessing cppyy through the kit takes **155 ms**.
These are import-stage times, not process startup times.

First-call time includes binding lookup, dispatch and any deferred
work; this measurement does not attribute all of it to compilation.
The source and numerical reference are the existing
[Eigen joint-step example](../eigen_startup/README.md).

The cached path still loads Cling and exposes declarations through cppyy.
The result demonstrates reduced resident memory for this workload, not removal
of the runtime compiler.

## Conventional compiled extension

[CMakeLists.txt](native/CMakeLists.txt) compiles the existing
[step.cpp](../eigen_startup/step.cpp) together with a
[small pybind11 wrapper](native/bindings.cpp). The wrapper accepts contiguous
float64 NumPy arrays, checks their shapes and output writability, and passes
their storage to the original function. Conversion is disabled so the output
cannot silently become a temporary array.
See [pybind11's NumPy interface](https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html).

The Python call remains:

```python
import eigen_step_native
eigen_step_native.joint_step(jacobian, delta, 0.05, output)
```

The benchmark places the extension in `build/runtime-cost-native` and adds that
directory to the native worker's import path. CMake, pybind11 headers and Eigen
are already available in this checkout's default Pixi environment.

The original pybind11-only clean build, including CMake configuration, took
**3.62 s**. The current combined pybind11/nanobind clean build took **5.64 s**.
The stripped release extension occupies **182,912 bytes**, about **179 KiB**, including the diagnostic probe bindings.
That is the extension file size, not the Python/NumPy deployment size.
All three native runs matched the NumPy reference with maximum absolute error
`1.53e-16`. Their process maps contained no Cling, LLVM or libclang entries.

The extension path uses approximately **167 MiB less resident memory** than the cached kit path
in this run. Its compiler and binding generation run during the build rather
than in the Python process. This is the deployment baseline for the proposed
interactive-to-compiled workflow.

## Nanobind and Rust

The [nanobind wrapper](native/nanobind.cpp) compiles the same Eigen `step.cpp`.
Its array types constrain shape, float64 type, CPU location, contiguous layout
and output writability. Argument conversion is disabled. This follows
[nanobind's array interface](https://nanobind.readthedocs.io/en/latest/ndarray.html).
Its extension is smaller than pybind11's here, while total process RSS is
effectively the same.

The [Rust source](rust/src/lib.rs) implements
`J.transpose() * solve(J * J.transpose() + damping² I, delta)` using nalgebra
0.33.2, PyO3 0.23.5 and rust-numpy 0.23.0. Cargo dependencies are recorded in
[Cargo.lock](rust/Cargo.lock). It uses Cholesky LLT for the positive-definite
damped system, whereas Eigen uses LDLT. It copies the small inputs into fixed-size
Rust matrices and writes the result into the caller's NumPy output array.
This is a port of the mathematical operation, not the same Eigen implementation
behind a Rust wrapper. See [nalgebra's Cholesky documentation](https://docs.rs/nalgebra/0.33.2/nalgebra/linalg/struct.Cholesky.html).

The Rust build took **12.06 s** with Rust 1.99.0 and a new target directory,
including crate resolution/download and compilation. Toolchain installation is
excluded. C++ build times use already installed headers, so these build times
describe this run rather than a controlled compiler-speed comparison.

All three compiled extensions had no Cling, LLVM or libclang mappings in their
runtime processes. Rust's maximum absolute error against the NumPy reference
was `4.16e-17`; both C++ bindings had `1.53e-16`.
The tiny RSS differences among compiled bindings are not evidence of a useful
memory advantage. Python and NumPy dominate these approximately 37 MiB processes.
Single-call timings do not establish a steady-state performance ranking.

## Startup, warmup and per-call p95

After building the extensions, run from this checkout:

```bash
pixi run python roscon_uk_2026/slide_assets/runtime_costs/speed.py
```

Three fresh processes per implementation run in rotated order. OS caches and
the kit function cache are warm. Import times below measure only the indicated
module. Preparation includes callable lookup and any native initialization or
compilation. The kit imports lazily, so its compiler initialization appears in
preparation. NumPy import takes about 29 ms in every case and is separate.

**Table 1: startup. Every value is milliseconds of elapsed time.**
Import and preparation are separate stages. The last column measures the whole
launch and includes those stages; do not add it to the other columns.

| Implementation | Import binding module (ms) | Prepare native operation (ms) | Entire launch through 1,000 warmup calls (ms) |
|---|---:|---:|---:|
| Vanilla cppyy | 151.32 | 1194.24 | 1446.3 |
| Kit cached | 12.45 | 149.47 | 222.3 |
| pybind11 | 0.190 | <0.001 | 58.8 |
| nanobind | 0.165 | <0.001 | 58.7 |
| Rust/PyO3 | 0.147 | <0.001 | 59.1 |

**Table 2: first call versus the following 1,000 calls.**
Every value is microseconds (µs); 1 ms = 1,000 µs.
The middle column is the **sum of time for all 1,000 calls**. The last column
divides that total by 1,000 to give an average per call. These 1,000 calls are
our fixed warmup procedure, not an estimate of how many calls warming requires.

| Implementation | One first call (µs) | Total for next 1,000 calls (µs) | Average per call within those 1,000 (µs/call) |
|---|---:|---:|---:|
| Vanilla cppyy | 40,875.6 | 281.9 | 0.282 |
| Kit cached | 1,582.1 | 267.4 | 0.267 |
| pybind11 | 75.0 | 434.2 | 0.434 |
| nanobind | 6.59 | 347.3 | 0.347 |
| Rust/PyO3 | 80.3 | 265.3 | 0.265 |

For example, nanobind's first call took 6.59 µs. The following 1,000 calls took
347.3 µs in total, averaging 0.347 µs each. The first call was about 19 times
slower than that average, not faster.

Launch-to-warmed time is measured by the parent process. It includes interpreter
startup, harness and NumPy imports, input allocation, preparation, the first
call, another 1,000 calls and notification through a pipe. A fixed 1,000-call
warmup is the procedure, not a measured minimum number of calls needed to warm
up. The table reports medians of three runs, not startup p95. Build times are
separate. Unlike the memory run above, this run does not pause between stages
to read process memory maps.

After warmup, seven batches of 10,000 calls measure throughput. Another 50,000
calls per process are timed individually. The p95 below pools **150,000
individual calls per implementation**. It is not a percentile of batch averages.
The callable and input/output arrays are retained throughout.

**Table 3: warmed operation. Every value is microseconds per call.**
The batch-derived average and individual-call percentiles come from **separate
measurements**:

- **Batch-derived average:** time a batch of 10,000 calls once, divide by 10,000,
  take the median of seven batch averages in each process, then the median of
  those three process results. This reduces timer overhead per call. It is not
  the median of individually timed calls.
- **Individual p50:** 50% of the individually timed calls took this long or less.
  This is the median of individual call durations.
- **Individual p95:** 95% of the individually timed calls took this long or less;
  5% took longer. This is neither an average nor a worst-case value.

| Implementation | Batch-derived average (µs/call) | Individual-call p50 (µs/call) | Individual-call p95 (µs/call) |
|---|---:|---:|---:|
| Vanilla cppyy | 0.271 | 0.298 | 0.328 |
| Kit cached | 0.248 | 0.273 | 0.293 |
| pybind11 | 0.449 | 0.483 | 0.512 |
| nanobind | 0.346 | 0.375 | 0.407 |
| Rust/PyO3 | 0.258 | 0.278 | 0.298 |

Individual timing reads the clock around every call. Its p50 can therefore be
higher than the batch-derived average even without substantial variation in the
operation. Compare p50 with p95 to understand the individual-duration spread;
use the batch-derived average to compare repeated-call throughput.

The timer-only control has p50 25–26 ns and p95 27–28 ns. It is recorded, not
subtracted. Individually timed calls therefore include instrumentation overhead.
The workstation is shared and these processes are not CPU-pinned. The measured
p95 characterizes this run; it is not a worst-case latency or real-time guarantee.
[Raw samples and summaries](speed_results.json) include each process's p95 and
links to compressed individual timings.

### Correction to the initial nanobind comparison

The original build used the binding projects' defaults. nanobind applied `-Os`
to the Eigen source, while pybind11 used `-O3` and LTO. That confounded the
comparison: the original 0.538 µs versus 0.443 µs result was not a controlled
comparison of the wrappers. [Original samples](speed_defaults.json) are retained.

Both wrappers now link the **same compiled Eigen object**, built once at `-O3`.
The binding code retains each project's defaults. See the
[CMake file](native/CMakeLists.txt) and [recorded flags](native_build_flags.txt).
This follows nanobind's recommendation to compile computation separately from
its size-optimized bindings in the
[official CMake documentation](https://nanobind.readthedocs.io/en/latest/api_cmake.html).

With this correction, nanobind's warmed call takes about **23% less time** than
pybind11's. Additional diagnostic probes support a binding-side advantage here:

| Diagnostic, no matrix solve | pybind11 | nanobind |
|---|---:|---:|
| One scalar argument and scalar result | 23.8 ns | 14.0 ns |
| Three arrays and damping, with validation | 299.4 ns | 213.2 ns |

These probes include Python loop/call overhead and use tuple unpacking in both
paths. They help explain this interface; they are not a general ranking.
The cppyy raw-pointer API does not enforce array shapes, while the conventional
wrappers do. Rust also uses a different factorization and copies small inputs.
The five-way table compares these actual interfaces rather than isolated
language or binding overhead. The largest median difference is about 0.20 µs
per call, small relative to a 1 ms controller period.

## Interpretation

RSS is resident memory of the whole process. It includes shared library pages,
Python, bindings, compiler state and generated code. It is not an isolated
measurement of LLVM allocation. MiB means 1,048,576 bytes; the imported cppyy
process's approximately 183 MiB is approximately 192 MB.

The [raw results](results.json) include proportional set size (PSS), private
resident pages, all stage timings, versions, cache status and numerical errors.
PSS apportions shared pages among processes. Sharing changes with other running
processes, so RSS values should not simply be multiplied to estimate system
memory for many workers.

These are resident snapshots after each operation, not peak memory measurements.
Compiler subprocess peaks during cache preparation are excluded. Larger headers,
additional instantiations and different versions can produce different results.
The measurement harness itself imports standard-library modules before its
baseline. This is a repeatable example, not a universal cppyy memory constant.
The cppyy paths retain the earlier small-function probe, which adds about 4 MiB;
the conventional extensions have no corresponding JIT probe. The final comparison
includes NumPy and identical numerical inputs in every path. Input preparation
occurs between the load and first-call memory samples. These single-call timing
samples are startup diagnostics, not warmed-throughput benchmarks.
