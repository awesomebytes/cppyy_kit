# Native worker implementation evidence

Measured on 4 October 2026 in this repository checkout, on Linux x86-64.
Commands used the root Pixi environment and its existing lock. No shared package,
manifest, lock, or core implementation was changed for this scope.

## Versions and commands

Observed versions were Pixi 0.70.0, Python 3.12.13, cppyy 3.5.0, NumPy 2.5.1,
pytest 7.4.4, and the conda-forge C++ compiler 14.3.0-19. The root lock selected
gcc_linux-64 and gxx_linux-64 14.3.0, with libstdcxx 15.2.0. The compiler helper
used C++17, `-O2`, `-shared`, `-fPIC`, and `-pthread`.

These commands require this checkout and were run from its root:

```bash
pixi run python -m roscon_uk_2026.next_steps.workers.check
pixi run python -m roscon_uk_2026.next_steps.workers.demo
sha256sum roscon_uk_2026/next_steps/workers/tests/test_worker.py
pixi run python -c 'import pytest,sys; print(pytest.__version__); print(sys.version)'
pixi run bash -c '"$CXX" --version'
pixi --version
```

The saved [checks](tests/test_worker.py) have SHA-256:

```text
0c1c0274ada518150e5085bc0defc125b604850995927a4b7c8997b7e0060ff9
```

## Correctness and lifecycle

The last full implementation check reported **6 passed in 1.52 s**. It checked:

- Independent serial mean and RMS values, timestamps, ordering, copied input
  lifetime, and submission-time parameter versions for 20 jobs.
- A gated queue of capacity two accepted sequences 0 and 1, rejected sequences
  2 and 3, and preserved accepted results after stop opened the gate and joined.
- A 4000-job run held at most four queued inputs and three retained summaries.
  It completed all jobs, retrieved 3000 summaries, and counted 1000 evictions.
  Accounting equations were checked after each of 1000 batches.
- Thirty independent context-manager lifecycles joined after an outer exception.
  Repeated start while running, repeated stop, submission after close, and restart
  after close followed the declared rules. Invalid sizes, timestamps, nonfinite
  samples, and nonfinite settings were rejected.
- A native drain blocked on a controlled gate. A Python supervisor observed one
  active drainer 100 times, updated settings, and retrieved a ready summary before
  opening that gate. A separate 1 ms drain timeout left a usable worker. Stop
  opened the gate and completed that job with its snapshotted setting.
- A subprocess submitted finite `1e308`, which overflowed the sum of squares in
  the native worker. Drain raised the stored error. Stop joined. The failing job
  count was one and the queued cancellation count was two. The subprocess then
  successfully ran another worker, proving recovery after exception propagation
  through the GIL-release shim.

The native-failure subprocess has a 30 second timeout. The whole check process
has a 90 second deadline. No test uses a guessed sleep to force queue overload.
Short sleeps in the supervision check poll an observed state with a deadline.

## Compilation, conversion, and execution

The first native adapter build was exercised in a subprocess under a 60 second
shell timeout. It succeeded. A cold build of the final native source took
0.620484 s; parsing its declarations and loading the library took 0.038260 s.
The existing cppyy PCH and GIL-shim cache were available. This is a cold adapter
build, not a clean installation or an empty global cache measurement.

The final demo ran successfully and printed 258 attempted, accepted, and
completed inputs. It had zero rejected, failed, cancelled, or evicted inputs or
results. The final snapshot was closed, with no running thread, active input,
queued input, retained result, or drainer. It reported 20 Python observations
and one ready summary retrieved during the controlled native drain.

The final demo observed these local costs:

| Operation | Time |
|---|---:|
| Native adapter cache lookup | 0.000089 s |
| Load library and declarations | 0.028861 s |
| Construct first wrapper, including load | 0.033244 s |
| Convert and submit initial 128 jobs of 512 samples | 0.004246 s |
| First drain and result conversion, including first GIL-shim use and supervision | 0.090932 s |
| Convert and submit warmed 128 jobs of 512 samples | 0.004586 s |
| Warmed drain and result conversion | 0.000464 s |
| Native arithmetic across the warmed 128 jobs | 0.000141 s |
| Convert the same values to a NumPy 128 by 512 matrix | 0.000331 s |
| NumPy batch mean and RMS | 0.000318 s |

Native arithmetic time is measured inside each successful job, before acquiring
the output mutex. It includes sum, sum of squares, and summary construction.
It excludes queueing, locking, output storage, wakeups, and Python conversion.
It overlaps the wall-clock drain measurement. These columns must not be added
as separate serial stages.

The NumPy baseline produces equal results within `1e-12` relative tolerance.
Input conversion and supervision make this small worker job slower overall than
the measured NumPy batch. This experiment demonstrates ownership and supervision
behavior. It does not establish a computation speedup.

Peak process RSS in the final demo was 254276 KiB. That includes Python, NumPy,
cppyy, Cling, and their caches. The bounded-storage claim comes from declared
container limits plus the long-run checks. RSS does not isolate native queue
memory, and no leak detector or sanitizer was run.

## Missing-implementation harness

The exact [prompt](PROMPT.md) and [skeleton](skeleton/worker.py) are present.
The unimplemented skeleton was checked with:

```bash
WORKERS_MODULE=roscon_uk_2026.next_steps.workers.skeleton.worker \
  pixi run python -m roscon_uk_2026.next_steps.workers.check
```

It exited with status 1 and **6 failed in 0.06 s**, caused by the expected
`NotImplementedError`. The independent harness therefore does not accept the
empty implementation. The test hash was unchanged.

No fresh GPT-6 Luna evaluation was run in this scope. No agent-generated
candidate, agent elapsed-time claim, or manual candidate repair is reported.
The working files are a saved implementation with measured checks. An installed
package proof, free-threaded Python, external I/O cancellation, other platforms,
and real-time scheduling were not evaluated.
