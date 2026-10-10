# Generated native-filter test results

Run on 4 October 2026 in this checkout on Linux x86-64. Commands below use
`roscon_uk_2026/next_steps/generated_tests/pixi.toml` and import the local checkout.
This tests the separately compiled demonstration in reverse_core.

## Correctness and fault detection

From this directory in this checkout:

```bash
pixi run --frozen check --hypothesis-show-statistics
pixi run --frozen find-fault
pixi run --frozen reproduce
pixi run --frozen reproduce --assert-correct
pixi run --frozen measure
```

The correct filter passed 18 pytest items in 1.98 s on the final run. Six generated properties completed
150 passing examples each, and the state machine completed 100 passing sequences
with a configured maximum of 35 actions per sequence. Direct native generated
checks reached `reverse_demo::PoseEstimator::process` and verified
`std::invalid_argument`, no output writes, unchanged state, and recovery after
rejection. The finite-range check includes explicit opposite maximum float64
observations and verifies finite output against an independent Decimal reference.

Hypothesis reduced the deliberate compiled C++ reset fault to these operations:

1. Process timestamp `0 ns` and position `[0, 0, 0] m`.
2. Reset.
3. Process timestamp `1 ns` and position `[0, 0, 1] m`.

The expected z output is `1 m`. The deliberate native fixture returned
`1.2499999921875e-08 m`. The saved [witness](minimized_fault.json) contains the
resolved settings and observations. A new interpreter running the standalone
reproducer confirmed that the correct native filter passes and the deliberate
fixture fails. `--assert-correct` exited 1 with the original numerical assertion.
The reproducer does not import Hypothesis. No native assertion was removed to
make the tests pass.

The native fault finder took 1.226 s on its recorded run. This includes first-use
fault fixture compilation/binding and Hypothesis generation and reduction.
The earlier Python-only reset fixture reduced to the same witness; it was
replaced by a compiled C++ fixture before the final reproduction.

## Environment and measured costs

| Dependency | Version |
|---|---|
| Python | 3.12.14 |
| cppyy | 3.5.0 |
| cppyy-backend | 1.15.3 |
| cppyy-cling | 6.32.8 |
| Hypothesis | 6.168.3 |
| NumPy | 2.5.3 |
| Pydantic | 2.13.5 |
| pytest | 8.4.2 |
| GCC and G++ | 14.3.0 |
| libgcc and libstdcxx | 15.2.0 |

The manifest and lock belong to this directory. They do not add Hypothesis to a
shared environment. The compiler uses the activated Conda sysroot. Startup
auto-PCH hooks are disabled by `CPPYY_KIT_NO_AUTOPCH=1`; cppyy's own standard PCH
may still be built and reused.

The measurement loaded `reverse_core/build/40694f2e84f189b8/libreverse_pose.so`.
Its SHA-256 was `c3acd8d562596865250ce2a431f86160e91bed8beae294711586952d1c8d8480`.
The Linux process map contained that exact library. Source hashes and measurement
values are saved in [evidence.json](evidence.json).

| Operation | Measured time |
|---|---:|
| Cached shared build lookup | 1.655 ms |
| Compile identical C++ library into owned build area | 2.895 s |
| Compile its standalone C++ driver | 0.488 s |
| First native construction, including cppyy startup/declarations | 343.389 ms |
| First 10,000-sample batch | 16.702 ms |
| Warm contiguous 10,000-sample batch, median of 30 | 115.531 us |
| Warm strided 10,000-sample batch, median of 30 | 124.554 us |
| Copy strided positions to contiguous storage, median of 30 | 8.728 us |

Batch times include Python validation, output allocation, the native call, and
result wrapping. Reset happens outside each timed batch. The copying measurement
isolates `np.ascontiguousarray` on the strided position input. This is a local cost
measurement with an existing standard PCH and compiled cache. Fresh dependency
downloads and a clean cppyy PCH build were not timed. No installed-package,
memory-leak, or speedup claim is made.

## Recorded failures and limits

The initial isolated environment used the default compiler solve and cppyy
startup exited 139 inside Cling `AddHostArguments`. A probe with system G++ then
failed to find system headers. Each probe ran in a separate process. Adding
`gxx_linux-64=14.3.0` supplied the compiler activation and sysroot. Explicit GCC/G++
14.3.0 and libgcc/libstdcxx 15.2.0 pins match the existing recipe compatibility
constraints. Startup and native tests then passed.

An early relative script command ran from the repository root and exited 2
because `reproducer.py` was not there. The documented Pixi task runs from its
manifest directory and reproduced the failure correctly.

Generated coverage is finite. The ordinary oracle uses coordinates bounded to
`[-1e6, 1e6] m`; a separate Decimal property covers arbitrary finite float64
magnitudes with scale-based tolerance. The state machine uses one configuration;
the batch properties use three validated configurations. Closed operations,
signed int64 extremes, invalid buffer representations, and unaligned buffers
have separate deterministic checks. Thread safety and concurrent use of one
instance are outside the contract. No native crash was reduced by Hypothesis.
A fresh-agent skeleton evaluation has not been run for this scope.

The final stateful statistics recorded every intended action category, including
reset (52.78%), smooth feed (62.96%), gap feed (60.19%), repeated timestamps across
calls (23.15%), decreasing timestamps across calls (36.11%), invalid shapes
(40.74%), invalid lengths (40.74%), dtype errors (53.70%), NaN (30.56%), positive
infinity (39.81%), and negative infinity (33.33%). Percentages describe generated
sequences containing an event, including invalid generated sequences. They do
not describe action counts or C++ line coverage. The state machine discarded
eight generated sequences; each property discarded zero to fourteen generated
inputs before reaching its 150 passing examples.
