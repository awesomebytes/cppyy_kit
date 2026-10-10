# OMPL extension experiment results

Recorded on 4 October 2026. The saved implementation passes all five tests.
Python and C++ policies produce identical native-planner outputs. The experiment
uses OMPL's existing geometric state-validity policy. It does not exercise a
sensor or estimator interface.

## Reproduction and evidence

Commands below require this repository checkout and run from its root:

```bash
pixi install --locked -e ompl
CPPYY_KIT_NO_AUTOPCH=1 pixi run --locked -e ompl pytest \
  roscon_uk_2026/next_steps/extensions/test_extensions.py -q
CPPYY_KIT_NO_AUTOPCH=1 pixi run --locked -e ompl python \
  roscon_uk_2026/next_steps/extensions/demo.py \
  --output roscon_uk_2026/next_steps/extensions/build/evidence.json
pixi run --locked -e ompl python \
  roscon_uk_2026/next_steps/extensions/check.py \
  roscon_uk_2026/next_steps/extensions/evidence.json
pixi list -e ompl
```

Observed test result: `5 passed in 18.52s`. The demo reported the independent
checks passed. [evidence.json](evidence.json) preserves the measured run.
Reproduction writes a separate ignored build artifact. The saved evidence
checker uses only the Python standard library and does not import the policy.

The root environment was already installed. A locked installation check took
`0.244 s`. This is an existing-environment check, not a fresh download or setup
measurement. No manifest, lock, core package, or shared environment dependencies
were changed. Auto-PCH was disabled for the probe processes.

## Correctness and scope

At seed 41, bias `0.05`, and motion-checking resolution `0.001`:

| Check | Observed result |
|---|---|
| Seven fixed-state decisions | `[1, 0, 0, 1, 1, 0, 0]` in both implementations |
| Batch accepted states | `90,000` out of `210,000` in each sample |
| Planner result | Exact solution in both implementations |
| Calls initiated by the native planner | `1,164` per implementation |
| Path length | `1.5051035285208356` dimensionless units in both |
| Returned coordinates | Exactly equal between Python and C++ |
| Independent path check | Every complete line segment remains outside the circle |
| Bias variations | `0`, `0.05`, and `-0.05` pass parity and geometry checks |
| Circle boundary and nonfinite states | Both implementations reject the tested cases |
| Invalid bias | NaN and both infinities rejected by both constructors |
| Callback exceptions | `ValueError` reaches caller at batch call 2 and planner call 7 |
| Lifetime | 25 forced-collection, dispatch, close, and weak-reference release cycles pass |
| Closed engine | Native call rejects before any further callback |
| Missing override logic | Skeleton fails with its `NotImplementedError` message |

The exception text includes the native method signature and the original Python
message. The lifecycle probe checks the exception type and exact dispatch count.
It does not convert callback exceptions into an invalid-state return value.

`native.hpp` JIT-compiles an ownership adapter and the equivalent C++ policy.
RRTConnect itself runs from the installed `libompl.so`. No virtual interface was
created for this experiment. The inspected base has one pure virtual,
`bool isValid(const State*) const`. The existing local `ompl_kit` supports both
this cross-inheritance path and a callback slot. `importlib.util.find_spec("ompl")`
returned `None` in the tested environment, so the official OMPL Python binding
was not available for a separate benchmark.

## Cost measurements

The batch loop traverses seven native points 30,000 times. Each implementation
runs five samples after one untimed warm call. Times below are medians inside
the native loop. Geometry, validity decisions, and callback counting are included.
Input conversion and wrapper startup are excluded from this loop timer.

| Measurement | Python override | Equivalent C++ override |
|---|---:|---:|
| Calls per batch sample | 210,000 | 210,000 |
| Warmed batch median | 85.354 ms | 1.191 ms |
| Batch time per call | 406.4 ns | 5.672 ns |
| Batch dispatch frequency | 2.46 million calls/s | 176.30 million calls/s |
| One native planner solve | 1.197 ms | 1.198 ms |
| Calls divided by native solve duration | 972,311 calls/s | 971,630 calls/s |
| Python wall time of the first successful solve call | 15.348 ms | 13.233 ms |
| Peak process RSS on Linux | 298,284 KiB | 296,664 KiB |

The batch operation was about 71.7 times faster with the equivalent C++ policy
in this run. That comparison includes Python state access and geometry, not only
virtual-dispatch machinery. The short planner solve did not show a speedup in
this sample. Its single duration is affected by first native solve work and
host scheduling. These are local measurements, not scheduling or throughput
guarantees. No real-time or multithreaded planner claim is made.

Startup and crossings were recorded separately:

| First-use measurement | Python process | Native process |
|---|---:|---:|
| cppyy and kit bringup | 834.808 ms | 792.119 ms |
| Adapter header parse | 34.543 ms | 33.853 ms |
| Engine construction, policy construction, and attach | 237.526 ms | 145.431 ms |
| First batch dispatch | 56.987 ms | 51.192 ms |
| First solve wrapper, using a closed-engine exception | 28.701 ms | 35.971 ms |
| First seven-point input conversion | 21.201 ms | 21.583 ms |
| Whole child process including startup and batches | 1.876 s | 1.313 s |

cppyy defers wrapper and method compilation until use. The header parse field
does not represent all compilation. Construction and first dispatch include
deferred JIT work. The closed-engine probe prepares the solve call wrapper
without sampling. The first successful solve still creates its first result
proxy, so its wall time includes deferred result conversion work. Native solve
duration and wall duration are both retained. No persistent compilation cache
was added and no fresh process is described as a warmed import.

## Initial failure and correction

The independent checker found a continuous-segment crossing at OMPL's default
motion-checking resolution `0.01`, seed 41, and bias `0`. OMPL returned an exact
solution because its intermediate sampled states passed. The closest point of
one segment had radius `0.2499880425838933`. The required radius is strictly
greater than `0.25`. The first test run had one failure and three passes.
[initial_failure.json](initial_failure.json) retains this initial output.

The adapter now defaults to resolution `0.001`. All three tested biases pass
the independent continuous-segment calculation at that resolution. The fifth
test retains the coarse case and requires the independent checker to reject it.
Reproduce that detection with this checkout command:

```bash
CPPYY_KIT_NO_AUTOPCH=1 pixi run --locked -e ompl python \
  roscon_uk_2026/next_steps/extensions/demo.py --bias 0 --resolution 0.01
```

Expected result: the command fails with `segment intersects keep-out circle`.
This is a recorded limitation of discrete motion checking for this geometry.
Reducing resolution does not guarantee continuous safety for arbitrary inputs.
OMPL documents the distinction in its
[motion validation reference](https://ompl.kavrakilab.org/stateValidation.html).

## Versions and evaluation status

Host: Linux `6.17.0-1032-oem`, x86-64, glibc `2.39`. Pixi `0.70.0`.
Relevant installed packages from the existing root lock:

| Package | Version |
|---|---|
| Python | 3.12.13 |
| cppyy | 3.5.0 |
| cppyy-backend | 1.15.3 |
| cppyy-cling | 6.32.8 |
| CPyCppyy | 1.13.0 |
| OMPL and ros-jazzy-ompl | 1.7.0 |
| GCC and G++ | 14.3.0 |
| libgcc and libstdcxx | 15.2.0 |
| Boost | 1.90.0 |
| Eigen | 5.0.1 |

The saved solution, missing-logic skeleton, explicit guide, and exact prompt
are complete. A fresh-agent evaluation and installed-package proof have not
been run. No agent-completion timing is claimed.

Independent-check SHA-256 values at this result:

```text
check.py
c99d0301753b468aff80205dd6b9f41f7feabc34aa19fcb04df7b59b37b9513f
test_extensions.py
14076b9e08a5ff4ccd863fd1570b83fb93f0662eb2aac2730fe761ba7b68e6cd
```
