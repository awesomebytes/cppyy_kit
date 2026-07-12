"""ik_bench -- a cppyy_kit benchmark suite (M6c).

Benchmarks inverse-kinematics solvers on the MoveIt Panda from ONE Python script:
packaged C++ plugins (KDL, TRAC-IK), C++-only unpackaged plugins vendored-built
(bio_ik, pick_ik), and a pure-NumPy baseline. cppyy + moveit_kit load every C++
plugin in-process via pluginlib, so no C++ harness / launch files are needed.

This is a *benchmark suite*, not a full kit (no mirror-API package): see
``run_bench.py`` for the harness and ``docs/ik_bench/REPORT.md`` for the results.
"""
__all__ = ["panda", "solvers"]
