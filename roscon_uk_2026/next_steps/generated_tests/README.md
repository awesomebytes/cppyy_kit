# Test a compiled pose filter with generated inputs

Check whether reset, chunk boundaries, and rejected batches preserve the filter's
documented state. Run these commands from this repository checkout:

```bash
cd roscon_uk_2026/next_steps/generated_tests
pixi install --frozen
pixi run --frozen check
pixi run --frozen find-fault
pixi run --frozen reproduce
```

Expected result: the correct native filter passes 18 pytest items. Hypothesis saves
a reduced reset failure in [minimized_fault.json](minimized_fault.json). The
reproducer reports `correct_native_passes: true` and `known_fault_reproduced: true`.
The deliberate native fixture outputs about `1.25e-8 m` on the last z coordinate;
the correct result after reset is `1 m`.

The first run compiles and loads the shared demonstration library through cppyy.
The fault finder also compiles a separate C++ forwarding fixture. It deliberately
omits the native reset call. Each processing call still reaches the real compiled
estimator. This fixture exists only to check that the tests detect a known fault.
The shared estimator is a demonstration implementation created for this experiment.

Read the guide explicitly with `pixi run --frozen guide`. No skill or agent setting
is installed. These are checkout commands. The manifest sets `PYTHONPATH` to this
checkout and uses a local environment, independently of the repository environment.
There is no installed-package proof here.

## What the tests check

Read [the public contract](../reverse_core/CONTRACT.md) before changing expectations.
The stateful test generates sequences of feed, reset, empty, and rejected calls.
It compares the entire public snapshot after every operation with an independent
Python model. A rejected batch includes a valid prefix and an invalid final row,
so validation after mutation would fail the check.

| Check | Purpose |
|---|---|
| Independent scalar reference | Verify the time-dependent smoothing result and counters. |
| Chunking | Detect state changes caused by batch boundaries, including empty chunks. |
| Reset isolation | Compare replay after reset with a fresh native instance. |
| Gap boundary | Check that equality smooths and one extra nanosecond resets. |
| Decimal reference | Check all finite float magnitudes, including opposite maximum doubles. |
| Direct native rejection | Bypass Python validation and check C++ exceptions, state, and output preservation. |
| Buffer and lifecycle checks | Verify readonly, strided, unaligned, owning output, and closed-instance rules. |

Six generated properties run 150 passing cases each. The state machine runs 100
passing sequences, with up to 35 operations per sequence. Ordinary numerical
comparisons use relative tolerance `1e-12` and absolute tolerance `1e-9 m`.
The full float-range property uses a separate Decimal reference and a tolerance
scaled to input magnitude to allow cancellation near zero. Chunking and reset
comparisons use exact equality because both sides execute the same native code.

These sample counts describe the completed run. They are not exhaustive coverage
of timestamps, configurations, or arbitrary call sequences. Configuration
validation and the C++ deployment driver have separate checks in reverse_core.

## Replay and investigate a failure

The saved reproducer does not import Hypothesis. Run this checkout command to
raise the original numerical assertion against the deliberate native fault:

```bash
pixi run --frozen reproduce --assert-correct
```

Expected result: exit status 1 and an `AssertionError` comparing approximately
`1.25e-8` with `1`. Ordinary `reproduce` succeeds only when the correct fixture
passes and the faulty one fails. The three operations are preserved as explicit
data; no random seed or example database is required to replay them.

Separate binding failures from algorithm failures. Shape and dtype checks target
the adapter. `test_native_rejection_is_atomic` calls the C++ class directly with
correctly sized buffers and expects `std::invalid_argument`. The deliberate reset
fault is in [the C++ fixture](native/faulty.cpp), under the production Python adapter.

Run crash-prone native probes as separate processes. A native crash can terminate
the interpreter before Hypothesis handles an assertion. Do not describe such a
crash as automatically reduced. Here the initial incompatible compiler environment
crashed in subprocesses; the assertion failure was reduced after startup worked.

## Maintainer and exercise files

[test_filter.py](test_filter.py) is the saved solution. [oracle.py](oracle.py) has
no native imports and does not copy the C++ implementation. It computes smoothing
with Python scalar arithmetic; the extreme-value property uses Decimal arithmetic.
[AGENT_PROMPT.md](AGENT_PROMPT.md) gives the exercise prompt, and
[skeleton.py](skeleton.py) contains two missing-logic functions. A fresh-agent
exercise evaluation has not been run for this scope.

The fault fixture replaces the adapter's private estimator field. This deliberate
test fixture must be reviewed if the adapter's internals change. Its compilation
cache belongs to this directory's ignored `build/`, and it links the exact shared
core library selected by `build_native()`.

Run these checkout commands for generation statistics and provenance:

```bash
pixi run --frozen check --hypothesis-show-statistics
pixi run --frozen measure
```

`measure` writes `build/measurement.json`. It verifies that the shared library is
mapped into the process, records source/library hashes, and separates compilation,
cppyy startup, buffer copying, and complete batch timings. It compiles an additional
copy into this directory's ignored build area to measure compilation without
clearing the shared build cache. Results from the completed run are in
[RESULTS.md](RESULTS.md).

Hypothesis API references: [stateful rules and invariants](https://hypothesis.readthedocs.io/en/latest/stateful.html),
[settings and find](https://hypothesis.readthedocs.io/en/latest/reference/api.html).
