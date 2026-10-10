# Exact missing-implementation prompt

Implement the bounded native summary worker described in
`roscon_uk_2026/next_steps/workers/README.md`.

Work in this existing repository checkout. Do not create a worktree or another
checkout. Use the root Pixi environment and its lock. Do not modify shared
packages, tests, manifests, or locks. Do not install skills or use extra agents.

Implement `roscon_uk_2026/next_steps/workers/skeleton/worker.py` and supporting
native source files in the skeleton directory. Read the native interface in
`roscon_uk_2026/next_steps/workers/worker.hpp`. You may copy that interface into the
skeleton directory. Do not read the saved implementations `worker.cpp` or
`worker.py` outside the skeleton directory. Use ordinary native threads. The
worker thread must run only C++ and own copied numeric input storage.

Supply scaled mean and RMS results with sequence, timestamp, count, and parameter
version. Bound both pending inputs and retained results. Implement drop-newest
input overload and counted oldest-result eviction. Snapshot parameter values on
accepted submission. Propagate native failures, cancel queued work on failure,
and join workers during context-manager teardown. Release the GIL for blocking
drain and join through a native nullary callable. Support the private dequeue gate
needed for deterministic checks. Do not use Python sleep in the native algorithm.

Run these unchanged checks from the repository root:

```bash
WORKERS_MODULE=roscon_uk_2026.next_steps.workers.skeleton.worker \
  pixi run python -m roscon_uk_2026.next_steps.workers.check
```

Record the commands, dependency versions, check hash, elapsed time, failures, and
manual repairs in the skeleton directory. A native crash or timeout is a failure.
Do not claim completion from output that was not actually observed.
