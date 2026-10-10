# Exact exercise prompt

```text
Test this C++ filter's reset and chunking behavior from Python. Generate edge
cases and save a small reproducer for any failure.

Work in this existing checkout. Read AGENTS.md and
roscon_uk_2026/next_steps/reverse_core/CONTRACT.md. Own only
roscon_uk_2026/next_steps/generated_tests. Do not edit the shared implementation,
create a checkout or worktree, install skills, or change shared dependencies.

Use the local Pixi manifest and actual separately compiled reverse_core library.
Complete skeleton.py with reset isolation and rejected-batch state preservation.
Build an independent Python oracle and Hypothesis properties and stateful tests.
Cover empty batches, reset, chunk boundaries, gap equality and reset, repeated and
decreasing timestamps, invalid shapes and dtypes, nonfinite rejection, finite
extreme values, snapshot counters, and cleanup. Native validation checks must
reach the compiled C++ class, not only the Python adapter.

Use a separate deliberately faulty fixture. Find and reduce a failing reset
sequence with Hypothesis. Save explicit data and a standalone reproducer that
does not import Hypothesis. Show that the correct compiled filter passes the
same witness. Distinguish binding errors from native algorithm errors. Probe
native crash risks in subprocesses; do not claim automatic crash shrinking.

Retain readable commands and actual results in README.md and RESULTS.md. Record
versions, native source/library provenance, test counts, limitations, and separate
compilation, startup, conversion, and execution costs. Preserve the exact prompt
and any failed commands or manual repairs. Do not spawn additional agents.
```

This is an exercise prompt. The saved implementation was completed as part of the
coordinated implementation session. A fresh-agent run has not been measured.
