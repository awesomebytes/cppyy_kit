# Retarget a trajectory without losing velocity and acceleration

Use a persistent Ruckig Community generator when a target changes during motion.
This experiment also computes a custom tracking command in the same native call.
The saved solution supports 3 or 6 joints. Joint positions use radians; velocity,
acceleration, and jerk use rad/s, rad/s², and rad/s³. The mathematical generator
can also use consistently scaled linear units.

These commands require this repository checkout and Pixi. Run from its root:

```bash
pixi install --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml
pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml guide
pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml probe
pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml check
pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml demo
```

Expect 40 passing checks and a demo that changes its target at ticks 0, 400, and
900. The generator advances 0.001 seconds per call. The demo reaches its final
rest target and settles the tracking error below 1e-6 rad with command mirroring.
This is a checkout experiment, not an installed cppyy_kit API.

For a short interactive example, start Python with the experiment as the current
directory and use its Pixi environment:

```python
from native import session, set_target, vector
engine = session(dofs=3, dt=.001)
set_target(engine, [.6, -.3, .2])
for _ in range(400):
    result = engine.step(vector([0., 0., 0.]))
before = engine.state()
set_target(engine, [-.2, .4, -.1])
after = engine.state()
assert list(before.velocity) == list(after.velocity)
assert list(before.acceleration) == list(after.acceleration)
result = engine.step(vector([0., 0., 0.]))
assert result.new_calculation
```

`target(p, v, a)` changes only the goal. It retains the current reference position,
velocity, acceleration, and accepted output. `reset(p, v, a)` explicitly replaces
the current state and clears the generator cache. It retains the goal and limits.
Use reset for a new episode. Resetting derivatives during a target reversal can
introduce a discontinuity; an acceptance check compares these two operations.

`step(measured)` advances the reference and calculates, for each joint:

```text
rate = clip(reference_velocity + kp * (reference_position - measured_position),
            -max_velocity, +max_velocity)
command_position = measured_position + dt * rate
```

The default kp is 12 s⁻¹. The returned sample contains the reference state, jerk,
command, result, trajectory time, trajectory duration, and native elapsed time.
Reference constraints apply to the Ruckig trajectory. The corrective command
rate is clipped. Its acceleration and jerk are not constrained by this law.
Measured positions affect the tracking command. They do not overwrite the
Ruckig reference state. The law is a custom example, separate from Ruckig Pro's
Tracking interface.

Invalid shapes, NaNs, infinities, nonpositive limits, and infeasible current or
target states raise `std::invalid_argument` before mutation. A rejected update
leaves the previous goal active. The next valid step continues it. The caller
must handle the error; the adapter does not issue a new hardware command for a
failed call. An unexpected solver error retains the accepted input/output and
clears the solver cache before raising `std::runtime_error`. This numerical
recovery is not a robot safety policy.

`Working` is 0 and `Finished` is 1. A rest target has zero terminal velocity and
acceleration and can be held. A nonzero terminal state continues moving after
Finished. Ruckig extrapolates constant acceleration after the trajectory ends.
Stop stepping or supply a new goal when that behavior is unwanted. The phase
constraint checks cover [0, duration], not indefinite extrapolation. At a target
update, position, velocity, and acceleration remain continuous. Jerk can jump;
there is no snap limit. `at_time(t)` and `knots()` inspect the last successful
calculation. A target setter alone does not replace that trajectory.

The independent checks compare matching inputs against the official Python
binding. They also split trajectories at every native jerk phase. They check
acceleration at both endpoints, jerk from acceleration secants, and velocity at
endpoints and every interior zero of acceleration. Midpoint checks verify the
constant-jerk polynomial. The absolute tolerance is 2e-8. Constraints are not
inferred from a coarse control-tick sample grid.

For the separate ros2_control integration, these commands require this checkout
and its existing control environment:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml build
pixi run --manifest-path roscon_uk_2026/pixi.toml -e control python \
  roscon_uk_2026/next_steps/ruckig/mock_control.py
```

This loads the actual control-kit ControllerManager and GenericSystem hardware.
A Python controller reads state and writes commands. Generation and the custom
law run inside one native step. GenericSystem mirrors commands into state; it
has no physical robot dynamics. The probe checks activation, three target
changes, settling, and teardown. Its reference advances 1 ms per callback,
independently of the variable period supplied by the manager and elapsed wall
time. The recorded period range and callback timings make this distinction
visible. It is an integration test, not a physical controller evaluation or a
real-time deployment result.

Run `pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml benchmark`
for timing distributions. Input vector creation, output conversion, native work,
Python/native boundary calls, recalculation, and the official-binding/Python-law
baseline are reported separately. [RESULTS.md](RESULTS.md) records the measured
run. The adapter allocates copied vectors and returned samples. It is single
caller code; shared concurrent access is unsupported.

Ruckig Community state-to-state generation runs locally. In the pinned release,
Community intermediate waypoints use a cloud service when that support is
built. This adapter excludes the cloud client and exposes no intermediate
waypoints. Pro has local waypoint and Tracking functionality. These are separate
features described in the [versioned upstream guide](https://raw.githubusercontent.com/pantor/ruckig/v0.12.2/README.md).

# Complete the native step from the skeleton

Read [prompt.txt](prompt.txt) explicitly. No skill or agent setting is installed.
The skeleton retains the working validation, state, and trajectory inspection
code, but removes `Session::step`. The complete saved solution is
[tracking.cpp](tracking.cpp). The independent checker is
[test_acceptance.py](test_acceptance.py).

These preparation commands require this checkout. Run in the experiment directory:

```bash
mkdir -p .build
cp skeleton/tracking.cpp .build/candidate.cpp
TRACKING_SOURCE="$PWD/.build/candidate.cpp" pixi run check
```

The untouched candidate fails because its step throws `std::logic_error`.
Complete `.build/candidate.cpp`, then rerun the same command. The candidate
build uses `.build/libcandidate.so`; the saved solution remains available.
This delivery has not been evaluated in a fresh agent session.

# Maintainer reference

[build.py](build.py) verifies the pinned source SHA256 before extraction. It
compiles the local algorithm sources and adapter with C++17 and `-O3`, excluding
Python, WebAssembly, and cloud entry points. The own Pixi lock pins the official
Python wheel and its hashes, cppyy, Python, NumPy, compiler, and runtimes. The
compiled adapter's narrow header avoids exposing Ruckig templates to cppyy.
[probe.py](probe.py) uses subprocesses for both the compiled path and an optional
direct-template declaration probe. It preserves stderr and return codes in
[probe_results.json](probe_results.json). Only the compiled adapter's full update
path is the supported implementation.
