# Score candidate robot positions with Eigen and OpenMP

Compute which candidate XY positions best explain measured distances to known landmarks. Each score is the negative sum of squared range residuals. Larger scores fit the observations better. This is the measurement-scoring step of range-only localisation. With equal Gaussian range noise, these scores are proportional to log likelihood, up to scale and an additive constant.

From this repository checkout:

```sh
pixi run env OMP_DYNAMIC=FALSE OMP_WAIT_POLICY=PASSIVE OPENBLAS_NUM_THREADS=1 python roscon_uk_2026/slide_assets/particle_scores/demo.py
```

Expected best candidate: approximately `[1.08074207, -2.0284263]`, near the simulated true position `[1, -2]`. The demo checks every score against NumPy.

[demo.py](demo.py) contains the complete decorator and C++ body. It uses Eigen from the existing Pixi environment, GCC 14.3 and OpenMP. No extra model or library download is needed on this machine.

```python
@cpp(headers=['Eigen/Core', 'omp.h', 'stdexcept'],
     include_paths=[EIGEN_INCLUDE], link_args=['-fopenmp'], nogil=True,
     preamble='#ifndef _OPENMP\n#error OpenMP compilation is required\n#endif')
```

The C++ body maps each candidate and landmark to Eigen vectors, computes distances and accumulates squared residuals. `#pragma omp parallel for num_threads(threads)` distributes independent candidates across native workers. `nogil=True` releases the GIL during that body, allowing other Python threads to run. These are separate mechanisms.

For the slide, show the decorator, the parallel-loop pragma, and the two nested loops. Explain the task as: “Which robot positions best explain these landmark measurements?” The complete source includes dimension and thread-count checks. The input arrays contain contiguous float64 XY pairs; there is one range per landmark and one output per candidate. Inputs remain unchanged during a call; the output must not overlap them or be shared between concurrent calls.

## Measurements

Recorded 21 October 2026 on an Intel Core Ultra 9 285H. Each cell below is **median / p95 milliseconds for one complete Python call**, from 60 calls following five warmups. Every case uses 32 landmarks and float64 arithmetic. C++ reuses a preallocated output. Both NumPy implementations allocate their outputs and temporary arrays.

| Method | 1,000 particles | 20,000 particles | 100,000 particles |
| --- | ---: | ---: | ---: |
| NumPy, broadcasting | 0.111 / 0.114 | 3.320 / 3.638 | 20.714 / 22.153 |
| NumPy, one landmark at a time | 0.128 / 0.131 | 1.193 / 1.214 | 6.690 / 6.725 |
| C++, 1 thread | 0.055 / 0.060 | 0.837 / 0.869 | 4.331 / 4.401 |
| C++, 2 threads | 0.039 / 0.042 | 0.460 / 0.475 | 2.305 / 2.756 |
| C++, 4 threads | 0.029 / 0.034 | 0.239 / 0.429 | 1.353 / 1.530 |
| C++, 8 threads | 0.047 / 0.064 | 0.912 / 1.251 | 2.208 / 2.513 |

For 20,000 particles, four native threads were about 5.0 times faster than the better NumPy implementation, and 3.5 times faster than one native thread. Eight threads were slower than four in this run. Worker counts were verified using `omp_get_num_threads()`. CPU frequency, scheduling and core placement were uncontrolled; this laptop has heterogeneous cores. The run does not establish the cause of the eight-thread slowdown.

The broadcast baseline uses in-place ufuncs to reduce allocation. The streamed baseline uses O(particles) temporary storage. C++ keeps each particle's accumulation in a scalar. Both NumPy versions and every native thread count agree within rtol 1e-12 and atol 1e-10. Native compilation uses the existing cache compiler defaults, including `-O2`, and `-fopenmp`; no fast-math flag is added.

A separate Python-thread probe used 250,000 particles and 128 landmarks, with one native worker. A 46.0 ms call with `nogil=True` allowed 41 heartbeat ticks inside the call. A 47.1 ms call holding the GIL allowed zero. Each tick waits approximately 1 ms. This demonstrates Python-thread progress, not guaranteed UI frame rate.

Reproduce:

```sh
pixi run env OMP_DYNAMIC=FALSE OMP_WAIT_POLICY=PASSIVE OPENBLAS_NUM_THREADS=1 python roscon_uk_2026/slide_assets/particle_scores/benchmark.py
pixi run env OMP_DYNAMIC=FALSE OMP_WAIT_POLICY=PASSIVE OPENBLAS_NUM_THREADS=1 python roscon_uk_2026/slide_assets/particle_scores/startup.py
```

[results.json](results.json) contains all 60 samples per case, hardware information, observed worker counts, and the GIL probe. Fixed path order and system load can affect timings.

## Startup and memory

[startup.json](startup.json) records one fresh function-cache build, followed by five fresh processes using that cache. The existing Cling PCH was available; OS caches were warm. The workload was 20,000 particles, 32 landmarks and four workers.

| Measurement | First function-cache build | Cached process median, 5 runs |
| --- | ---: | ---: |
| Import demo, NumPy and decorator | 56.4 ms | 51.5 ms |
| First scoring call | 829.3 ms | 176.9 ms |
| Whole subprocess, including exit | 936.9 ms | 271.9 ms |
| Peak RSS before imports | 18.7 MiB | 18.7 MiB |
| Peak RSS after imports | 36.6 MiB | 36.5 MiB |
| Peak RSS after first call | 233.1 MiB | 233.3 MiB |

The first call includes lazy runtime initialization and call-wrapper preparation. These times are separate from warmed per-call computation above. RSS is the Linux process high-water mark from `getrusage(RUSAGE_SELF)`; it includes the interpreter, NumPy, cppyy, native libraries and arrays. It excludes the compiler subprocess and is not a measurement of compiler memory alone.

## Core package changes and validation

- `@cpp(headers=...)` emits library includes outside the function and namespace.
- `preamble=` supplies file-scope C++; its text and headers affect symbol and cache identities.
- `link_args=` forwards driver options. GCC applies `-fopenmp` to compilation and linking in the existing single compiler invocation.
- Explicit link arguments select `cppdef_cached(compiled_only=True)`. Cache misses build and load the shared library before exposing declarations to Cling. Build failures propagate. Disabling caching raises an error for this path, rather than silently ignoring compiler options.
- Uncached header examples now receive their include paths before Cling compilation.

Validation: 103 existing decorator/typing tests and 20 existing cache tests passed. The benchmark verifies numerical agreement, empty inputs, dimension errors, invalid thread counts, actual OpenMP worker counts and GIL release. Additional direct checks verified headers in uncached mode and different preambles producing distinct callable implementations. No new test suite was added.

## Scaling to 16 workers

A follow-up run interleaved thread counts in a seeded random order, with 60 calls per count and five warmups. All cases used 32 landmarks. Each entry is median / p95 milliseconds per call. These are fresh measurements, separate from the table above.

| Workers | 20,000 particles | 100,000 particles | 500,000 particles |
| --- | ---: | ---: | ---: |
| 1 | 0.926 / 1.008 | 4.402 / 4.597 | 21.348 / 21.851 |
| 2 | 0.507 / 0.954 | 2.313 / 2.538 | 11.333 / 11.779 |
| 4 | 0.301 / 0.894 | 1.219 / 2.198 | 6.009 / 6.442 |
| 8 | 0.487 / 0.783 | 2.057 / 2.303 | 10.073 / 10.951 |
| 12 | 0.437 / 0.896 | 1.510 / 1.834 | 6.923 / 7.551 |
| 16 | 0.462 / 0.775 | 1.265 / 1.629 | 5.452 / 6.639 |

Sixteen workers gave the lowest median at 500,000 particles, about 3.9 times faster than one worker and 9% faster than four. Four workers gave the lowest median for the smaller workloads. Worker creation counts and numerical agreement were verified. Threads were not pinned; heterogeneous cores, scheduling, and parallel overhead are possible explanations for the non-monotonic scaling, not established causes.

Reproduce from this checkout:

```sh
pixi run env OMP_DYNAMIC=FALSE OMP_WAIT_POLICY=PASSIVE OPENBLAS_NUM_THREADS=1 python roscon_uk_2026/slide_assets/particle_scores/thread_scaling.py
```

All samples are in [thread_scaling.json](thread_scaling.json).

## Plain Python baseline and slide speedups

[plain_python.py](plain_python.py) implements the same scoring operation with Python lists, Python floats, and standard-library `math.hypot`. It has no NumPy dependency. `hypot` itself is a native standard-library function; the surrounding loops and arithmetic execute through Python. Its distance algorithm differs from the direct square-root calculation, but scores agree within rtol 1e-12 and atol 1e-10 for these inputs.

A fresh comparison used 20,000 particles, 32 landmarks, five warmups and 60 calls per method with randomized interleaving. Input generation and conversion to lists were outside timing. Native output was preallocated; Python and NumPy allocated their outputs. The slide uses this complete new comparison, rather than mixing its timings with earlier runs.

| Implementation | Median per call | Speedup over Python |
| --- | ---: | ---: |
| Python lists + math.hypot | 39.182 ms | 1.0× |
| NumPy: broadcasting | 4.957 ms | 7.9× |
| NumPy: one landmark at a time | 1.344 ms | 29.2× |
| Eigen C++: 1 worker | 0.904 ms | 43.3× |
| Eigen C++: 4 workers | 0.450 ms | 87.1× |

Speedup is the Python median divided by each implementation's median. These figures exclude compilation and startup. Plain Python may suit an occasional query; this workload alone exceeds a 33.3 ms frame budget at 30 Hz. CPU frequency, placement and background activity remain uncontrolled.

Reproduce from the repository checkout:

```sh
pixi run env OMP_DYNAMIC=FALSE OMP_WAIT_POLICY=PASSIVE OPENBLAS_NUM_THREADS=1 python roscon_uk_2026/slide_assets/particle_scores/comparison.py
```

All samples, including p95 values, are in [comparison.json](comparison.json). The slide displays only median time and speedup. Its speaker notes explain the Python implementation and comparison conditions.
