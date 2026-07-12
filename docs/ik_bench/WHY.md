# Why ik_bench — one Python script benchmarks C++-only solvers

Inverse-kinematics solvers live in an awkward split. The best ones are **C++ MoveIt
plugins**, and some of the most interesting — **bio_ik** (evolutionary/memetic) and
**pick_ik** (gradient-descent + memetic global) — **are not packaged at all**: no
`pip`, no `conda`, no apt. Others (KDL, TRAC-IK) are packaged C++. And there is always
someone's pure-Python Jacobian solver. Benchmarking these against each other normally
means standing up a C++ test harness, launch files and a parameter server per solver —
enough friction that the comparison rarely gets done honestly.

**ik_bench does it in one Python file.** cppyy + `moveit_kit` load each C++ plugin
*in-process* through MoveIt's own pluginlib mechanism and call `RobotState::setFromIK`;
the two unpackaged solvers are built from source once and discovered by the *same*
lookup-by-name path as the packaged ones — cppyy never parses a line of their headers,
pluginlib just `dlopen`s the compiled `.so`. The pure-Python baseline is NumPy only.
`python ik_bench/run_bench.py` runs all five on the same Panda, the same seeded targets
and the same tolerances, and prints one table (solve-rate, verified success %, accuracy,
near-limit behaviour). The punchline: the fastest solver in the table, **bio_ik, is one
you cannot install** — it exists only as C++ source, yet here it is benchmarked,
configured and beaten-or-beating its packaged peers from a single script. That is the
new use case cppyy_kit unlocks: **cppyy as the harness that makes C++-only libraries
first-class citizens of a Python benchmark.**
