# Ruckig experiment results

Recorded on 4 October 2026 in this checkout. Persistent local Community generation
and a custom native tracking law work for 3 and 6 DoF. The independent checks pass.
The control-kit mock integration passes. These are numerical and integration
results, not physical robot or real-time results.

## Reproduce

All commands below require this checkout. Run from the repository root:

```bash
pixi install --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml
pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml build
pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml probe
pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml check
pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml demo
pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml benchmark
pixi run --manifest-path roscon_uk_2026/pixi.toml -e control python \
  roscon_uk_2026/next_steps/ruckig/mock_control.py
```

The own manifest and lock install Python 3.12.13, cppyy 3.5.0,
NumPy 2.5.3, pytest 8.4.2, and the official Ruckig
0.12.2 wheel. GCC/G++ are 14.3.0. libgcc/libstdcxx are pinned to 15.2.0.
The separate mock probe uses the existing presentation control environment,
control-kit 0.3.0, controller_manager 4.47.0, and hardware_interface 4.47.0. It
requires that environment; the own lightweight environment does not contain ROS.

The source archive SHA256 is
`08f69a165da3815d122ea3c5f377b5dd76c2e626ac026e5156d5e32dd4636b0f`.
The official cp312 Linux x86_64 wheel SHA256 in the own lock is
`ea082b3234a3f70d9329452e018bd01fcaf108d6cdfd819ea686aa74c5360ac6`.
The upstream [0.12.2 release files](https://pypi.org/project/ruckig/0.12.2/#files)
provide these artifacts. Native compilation excludes cloud, Python, and WebAssembly
entry points. The download is build input in `.build`, not another repository checkout.

## Correctness evidence

The final checker reports 40 passing tests. It covers two dimensions, goal
changes, direction reversals, Working/Finished, reset, nonzero terminal states,
random finite targets, invalid dimensions and periods, vector sizes, NaN and
infinity, nonpositive limits, and infeasible current/terminal states. Tightening
limits during motion is transactional. Rejected targets retain the previous goal.
Exception checks require `std::invalid_argument`, so an unrelated binding error
does not satisfy a rejection test.

For matching inputs, position, velocity, acceleration, time, result, and
new-calculation flags agree with the official Python binding. The absolute
state and limit tolerance is 2e-8 in consistent joint units. The tracking command
also agrees with an independently computed Python formula to 2e-12 rad.

Constraints are checked on the full [0, duration] trajectory for every new
calculation in the changing-target cases and 12 random target segments per
DoF configuration. Every constant-jerk phase is inspected. Acceleration extrema
are checked at endpoints. Velocity extrema also include the interior roots of
acceleration. Acceleration secants check jerk, and midpoint states check the
phase polynomial. These checks cover continuous motion between sampled ticks.
Jerk discontinuities at phase/target boundaries are allowed. They are not
interpreted as a snap constraint. Nonzero terminal acceleration extrapolation
beyond duration is tested separately and excluded from the bounded-segment claim.

Both subprocess probes return 0. The compiled adapter probe performs a step,
reads phase knots, and samples its trajectory. The direct template probe only
checks a declaration/construction. It does not establish a complete JIT-only
Ruckig integration. [probe_results.json](probe_results.json) retains the output.

An early phase-inspection implementation crashed with a segmentation violation
(exit 129) because a C++17 range loop referenced a subobject of a temporary
profile container. The implementation now retains that container. The extended
subprocess probe exercises this path. Tightening exception assertions also
exposed that upstream validation throws its own exception type. The adapter now
preserves the upstream reason while translating it to `std::invalid_argument`.
One initial target-feasibility assertion used the wrong acceleration sign. It
was corrected to test backwards feasibility at the terminal state.

The untouched skeleton is rejected by the unchanged checker at its first
trajectory test with `std::logic_error: complete the native step`. The exact
prompt is [prompt.txt](prompt.txt), and the saved solution is
[tracking.cpp](tracking.cpp). No fresh agent session has been evaluated. Agent
completion time and installed-package-only behavior were not measured.
The independent checker SHA256 is `de3014b037aafeaef69f1b0586e1be908ef7b7dcc247e2c0d913c668adc57f82`.

## Costs measured in this process

The retained full source/adapter compile took 7.721 s.
Its compiler was `x86_64-conda-linux-gnu-c++ (conda-forge gcc 14.3.0-20) 14.3.0`.
The warmed cppyy import took 0.210 s, and shared-library loading
plus parsing the adapter declarations took 0.017 s.
Archive verification with already downloaded sources took
0.0008 s. Initial download and environment-install
latency were not isolated. The import timing can include cached PCH use; it does
not measure a first-ever PCH build. Generated binaries are ignored.

Times below are p50 / p99 in microseconds. Routine native calls use 4,990 samples
per DoF. Conversion and official-binding cases use 3,000 samples after 100 warmup
calls. Recalculation has only 10 samples; its percentile is descriptive and
interpolated. Results are from one process on `Linux-6.17.0-1032-oem-x86_64-with-glibc2.39`.

| Operation | 3 DoF, µs | 6 DoF, µs |
|---|---:|---:|
| Native generation/tracking, internal timer | 0.321 / 0.376 | 0.377 / 0.426 |
| Native step with retained input vector | 0.803 / 0.924 | 0.832 / 0.939 |
| Native recalculation with retained input | 1.933 / 8.221 | 2.380 / 4.463 |
| Official binding plus Python tracking law | 1.142 / 1.446 | 1.513 / 1.786 |
| Python list to copied native vector | 0.775 / 0.903 | 0.848 / 1.010 |
| Four native output vectors to Python lists | 2.278 / 2.523 | 2.270 / 2.525 |
| Native step including input list conversion | 1.560 / 1.762 | 1.776 / 1.975 |

[measurements.json](measurements.json) contains p95, maximum, counts, versions,
compile command, and startup timings. Internal native timing includes generation,
tracking, output copies, and snapshot creation. Boundary timing includes cppyy
call/return overhead but retains its input vector and returns a native sample
proxy. List conversion copies inputs. Fully converting native result vectors
adds the reported output cost. The official baseline already uses a compiled
Ruckig Python extension; its custom tracking formula runs in Python. No blanket
end-to-end speedup follows from the smaller retained-input call time. Input
conversion alone makes this native call slower than the official composed
baseline in the measured run. Result conversion adds further cost.

Peak process RSS was 229,492 KiB on Linux. This includes Python,
cppyy/Cling, NumPy, pytest imported for a shared baseline fixture, the official
binding, and both sessions. It is not per-generator memory. The native adapter
uses dynamic vectors and creates returned samples; it makes no allocation-free,
thread-safe, deadline, or real-time guarantee.

## Mock control integration

The final separate probe ran 3200 cycles in 3.211157 s and recorded
3201 callbacks including activation. The three target-update ticks were
[0, 400, 900]. The last 250 samples had maximum reference/position
error 5.40568516e-09 rad. Python callback p50/p99 was
35.860/177.928 µs, including state reads, native step,
command writes, and recording. Activation, configure, and teardown completed.
[mock_results.json](mock_results.json) records these values.

Manager periods ranged from 0.000573332 to
0.060761469 s. The generator advances exactly 0.001 s per callback.
Thus trajectory time is nominal callback time, not measured wall time. The
manager also reported overruns during first use. An initial assertion that all
manager periods equal 1 ms failed and was replaced by an explicit recorded
period range and the stated fixed-step contract. This probe checks interfaces,
state transport, and lifecycle. It is not a timing-correct physical controller.
The upstream [GenericSystem source at 4.47.0](https://raw.githubusercontent.com/ros-controls/ros2_control/4.47.0/hardware_interface/src/mock_components/generic_system.cpp)
implements command-to-state mirroring when dynamics calculation is disabled,
as it is in this URDF.

## Scope and remaining limits

Local state-to-state generation is the only Ruckig functionality exposed here.
The pinned [upstream guide](https://raw.githubusercontent.com/pantor/ruckig/v0.12.2/README.md)
separates Community cloud waypoint support, local Pro waypoint support, and the
Pro Tracking interface. This custom law does not use those features. It has no
intermediate waypoints or cloud traffic. Tracking commands have a clipped
velocity correction but no acceleration/jerk guarantee. The reference maintains
its own kinematic state rather than estimating actual plant state. Extreme
numerical scales, physical dynamics, real-time scheduling, multi-threaded access,
and long-term deployment are not covered by these fixtures. The validation and
reference limits apply to the tested numerical scale and trajectory duration.
