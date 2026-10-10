# Summarize timestamped samples in a native worker

Use this experiment when Python needs to submit numeric samples, update settings,
and inspect summaries while a native worker runs. Each accepted input produces a
scaled mean and root mean square. Results retain timestamp and sequence identity.

## Run the example

These commands require this repository checkout. Run them from its root.
The root [Pixi manifest](../../../pixi.toml) and [lock](../../../pixi.lock)
supply the compiler, cppyy, NumPy, and pytest. No pip installation is needed.

```bash
pixi install --locked
cat roscon_uk_2026/next_steps/workers/README.md
pixi run python -m roscon_uk_2026.next_steps.workers.demo
pixi run python -m roscon_uk_2026.next_steps.workers.check
```

The demo prints JSON. It reports 258 completed jobs and a stopped worker. A Python
supervisor makes 20 observations, updates the scale, and retrieves one ready
summary while the main thread is waiting inside native `drain`. The mean changes
from 2.555 to 5.11 when scale changes from 1 to 2. The checks finish with six passing
tests. Timings vary by machine and by cache state.

The files are a checkout example. They are not an installed package interface.
For a small job, the native queue and input copies can cost more than a NumPy
batch reduction. The demo measures that baseline on the same 128 by 512 values.

## Submit samples

Run this Python code in the activated checkout environment:

```python
from roscon_uk_2026.next_steps.workers.worker import SummaryWorker

with SummaryWorker(queue_capacity=4, result_capacity=8, max_samples=100) as worker:
    sequence, accepted = worker.submit(1_000_000, [1.0, 2.0, 3.0])
    assert accepted
    results = worker.drain()
    assert results[0].sequence == sequence
    assert results[0].timestamp_ns == 1_000_000
    assert results[0].mean == 2.0
    assert abs(results[0].rms - (14.0 / 3.0) ** 0.5) < 1e-12
```

Timestamp values are nonnegative signed 64-bit integers, in nanoseconds. They
belong to one caller-defined clock. Every valid submission, including a dropped
submission, must have a strictly increasing timestamp. This example performs no
clock conversion. Samples must be finite. Inputs must support `len` and numeric
indexing. Empty inputs and inputs longer than `max_samples` are rejected.

Submission converts samples into a temporary C++ vector. Native submission copies
that vector into owned queue storage before returning. The native thread retains
no Python object or buffer. You can change or release the original input after
`submit` returns. Changes to an input during its conversion are unsupported.

## Read overload and lifecycle state

`submit` returns `(sequence, accepted)`. Sequences start at zero. Every valid
submission attempt consumes one sequence, so drops leave visible gaps. Invalid
inputs consume no sequence.

The input policy is **drop newest**. At most `queue_capacity` inputs wait in the
queue, plus one input being processed. A full queue rejects the new input and
increments `rejected`. Accepted inputs remain in submission order. `result_capacity`
sets the number of retained summaries. A full result ring evicts its oldest
summary and increments `result_evicted`. Losing a retained summary does not undo
successful computation. Choose capacities for the rate at which the supervisor
will retrieve results.

`take_results` retrieves the currently retained summaries and empties the result
ring. `drain(timeout_ms=5000)` waits until the queue is empty and no input is
active, then retrieves retained summaries. It can time out. A timeout leaves the
worker running. Stop producers before using drain as a completion boundary. A
concurrent producer can submit more work after drain observes the idle state.

`start` is idempotent while running. `stop` closes submission, opens the test gate,
finishes accepted inputs, and joins the native thread. `stop` is idempotent after
successful completion. Stopped workers cannot restart; construct a new instance.
The context manager calls start and stop. A worker error is reported again by
stop, including repeated stop calls. If the context body already raised, cleanup
preserves that exception and leaves any worker error in the snapshot.

`snapshot` returns a mutex-protected copy. Use these equations to check accounting:

```text
attempted = accepted + rejected
accepted = completed + failed + cancelled + queued + active
completed = retrieved_results + retained_results + result_evicted
```

Here `active` counts as zero or one. `retrieved_results` is the count tracked by
the supervisor across `drain` and `take_results`. The native snapshot does not
retain another unbounded history of retrieved results.

`update_scale` accepts a finite scale and returns a parameter version. The mutex
orders configuration updates with submission. Each accepted input snapshots the
scale and version at submission, so queued inputs preserve their old settings.
Negative scales change mean sign and leave RMS nonnegative.

Numeric overflow is a worker failure. The worker stores the exception message,
counts the failing input, cancels queued inputs, and closes submission. Drain
raises the stored error. Stop still joins before reporting that error. A successful
summary is counted only after its result has been stored.

## Inspect the implementation and checks

[worker.hpp](worker.hpp) is the native interface. [worker.cpp](worker.cpp) owns the
thread, mutexes, condition variable, queue, and result ring. [worker.py](worker.py)
owns the native object and implements the Python context manager. It uses the
checkout's [compile helper](../../../cppyy_kit/_compile.py) and
[nogil helper](../../../cppyy_kit/nogil.py). Blocking drain and join receive native
`std::function<void()>` objects. Those functions capture the still-owned C++
worker, with no Python callbacks. The native destructor can join while the GIL is
held because the worker needs no Python execution to finish. Explicit context
cleanup releases the GIL while joining.

The numerical operation is a serial double-precision sum and sum of squares.
It does not use a compensated sum. Checks compare it with independent `math.fsum`
references at relative tolerance `1e-12` and absolute tolerance `1e-14` on bounded
inputs. The demo also compares its warm batch with NumPy.

Queue memory is bounded by configured input count and sample count. Result storage
is bounded by its capacity. A result insertion can briefly hold one additional
summary before evicting the oldest. Submission and retrieval make temporary
copies. Python owns retrieved result lists, so applications must also bound the
history they retain.

[tests/test_worker.py](tests/test_worker.py) controls dequeueing through the private
`_set_test_gate` hook. It fills queues without guessing worker timing. The GIL
test observes a native drainer before the Python supervisor opens the gate. If
the blocking native call held the GIL, this test would time out. The 4000-job
test checks input and result limits and exact eviction counts at each batch.
Native failure runs in a subprocess with a 30 second timeout. The complete check
runner has a 90 second deadline.

The example uses one ordinary native thread. It does not need Boost.Asio or a
lock-free queue for this CPU operation. It makes no real-time scheduling,
free-threaded Python, external I/O cancellation, or performance guarantee.

## Try the missing implementation

Read [PROMPT.md](PROMPT.md) explicitly. [skeleton/worker.py](skeleton/worker.py)
contains the missing supervisor implementation. The checks accept a candidate
module through `WORKERS_MODULE`. The saved working solution is the implementation
in this directory. [RESULTS.md](RESULTS.md) separates implementation checks from
the fresh-agent evaluation, which has not been run here.
