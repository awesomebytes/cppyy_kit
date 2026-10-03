# ik_bench: compare IK solvers from Python

KDL and TRAC-IK are packaged MoveIt plugins in this environment. bio_ik and pick_ik
are available here only as C++ source. Comparing these plugins with a Python solver
usually requires separate C++ test programs and MoveIt configuration.

`ik_bench/run_bench.py` loads the plugins through MoveIt pluginlib and calls
`RobotState::setFromIK`. The bio_ik and pick_ik plugins are built from source and
discovered by plugin lookup name. cppyy does not parse their headers. A NumPy solver
provides the Python comparison.

The script runs all five solvers on the same Panda model and seeded target set. It
reports solve rate, success rate checked with forward kinematics, position and
orientation error, and near-limit results. See [REPORT.md](REPORT.md) for the
measurement setup and results.

## Run this repository example

The benchmark is a repository example, not a separately published package. From this
checkout, run `pixi run -e ik bench-ik`; it prints a comparison of KDL, TRAC-IK,
bio_ik, pick_ik, and NumPy DLS on seeded Panda targets. For the setup and limits of
those results, see [REPORT.md](REPORT.md). The underlying MoveIt integration is
published as [`ros-jazzy-moveit-kit`](https://repo.prefix.dev/awesomebytes); see
[Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/) for
installation or source-development instructions.
