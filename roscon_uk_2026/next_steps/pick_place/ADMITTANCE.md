# Push the hand with a Python admittance controller

For the conference RViz demo, run `./roscon_uk_2026/run_robot_demo.sh admittance` from the repository root. This starts the simulator, Python controller and RViz force handle. See [the three-demo guide](../robot_demos/README.md) for drag controls and shutdown.

This repository demo gives the left hand a virtual spring, mass and damper.
Hold a browser button to apply a virtual force. Release it and the hand returns
toward its initial pose. Python calculates the spring response. One native call
calculates arm kinematics and joint commands inside Gazebo's ros2_control manager.

## Run the demo

These commands require this repository checkout. Start in this document's
`pick_place` directory. Use the existing simulator `kit` environment because the
controller imports cppyy and NumPy inside Gazebo. The simulator `default`
environment does not provide cppyy.

First, prepare the reusable Python plugin:

```bash
(
  cd ../tiago_pro_jazzy
  pixi run -e kit env \
    PYTHONPATH="$PWD/../../..:$PWD/../../../control_kit" \
    python ../pick_place/admittance_demo.py --prepare-only
)
```

Stop any existing simulator. In another terminal, starting in `pick_place`, run:

```bash
(
  cd ../tiago_pro_jazzy
  pixi run -e kit env \
    AMENT_PREFIX_PATH="$HOME/.cache/cppyy_kit/control_plugin:$PWD/.pixi/envs/kit" \
    PYTHONPATH="$PWD/../../..:$PWD/../../../control_kit" \
    bash ../simulator_guard.sh -- ros2 launch "$PWD/playground.launch.py" \
    gui:=true rviz:=false moveit:=false navigation:=false
)
```

After `TIAGO_PRO_READY`, run from `pick_place`:

```bash
pixi run --manifest-path ../../../pixi.toml -e pick-place demo-control-admittance --interactive
```

Open the printed localhost URL, normally `http://127.0.0.1:8765/`. Hold an axis
button with the mouse, touch, or keyboard Space/Enter. Force is in newtons, in
`torso_lift_link` coordinates. The controls apply no torque. Start with 1 N;
the UI permits up to 5 N. The display shows force and measured hand position.

Release the button to remove the force. Commands expire after 0.3 s without
refreshes. Press Enter in the demo terminal or Ctrl-C to finish. The interactive
demo allows the spring to return, restores the stock arm controller, and unloads
the custom controller. The simulator remains running until you stop its terminal.

For a repeatable headless proof, use `gui:=false` in the launch command and run:

```bash
pixi run --manifest-path ../../../pixi.toml -e pick-place demo-control-admittance \
  --force 1 --axis x --duration 8
```

This applies +X force for eight wall seconds, releases it for eight wall seconds,
and writes `admittance_evidence.json` in the repository root. Expect `passed: true`
and `stock_controller_restored: true`. `--report` changes the output path.

## Read the controller

Start with [CartesianAdmittance.update](../../../control_kit/demos/python_admittance_controller.py).
The Python law integrates, independently along three axes:

```text
mass * acceleration + damping * velocity + stiffness * offset = force
```

Defaults are 2 kg, 30 N s/m and 80 N/m. A constant 1 N force therefore requests
a 12.5 mm equilibrium offset. The offset is relative to the hand pose recorded
on activation. The reference orientation stays fixed. Python limits the offset
norm to 30 mm and its velocity norm to 60 mm/s. Integration uses substeps no
larger than 2 ms. No force sensor is read in this version.

The line `self.chain.step(...)` calls the
[native kinematics kernel](../../../control_kit/demos/arm_kinematics.hpp).
It evaluates forward kinematics and the analytic geometric Jacobian, then solves
a damped least-squares system for joint velocity. It scales that vector to a
0.1 rad/s joint command rate and applies position bounds. These are bounds on
commands; they do not guarantee the physical joint velocity or tracking error.

The [Chain wrapper](../../../control_kit/demos/arm_kinematics.py) parses the URDF
once, compiles the Eigen kernel with `cppyy_kit.prebuild()`, and exposes it through
`cppyy_kit.cppdef_cached()`. It warms both native entry points before activation.
The compiled kernel is reused from the cache. This avoids passing Eigen function
bodies through Cling on a cold cache. Restart Gazebo after editing the Python law.

The reusable [Python plugin bridge](../../../control_kit/control_kit/python_controller.cpp)
reads ros2_control joint feedback, calls the Python law, and writes its returned
positions. The [demo process](admittance_demo.py) handles browser input, publishes
virtual forces, observes motion and restores controllers. It does not calculate
the control commands.

## Measured result and limits

The [recorded headless run](evidence/admittance/live.json) applied 1 N along +X:

| Measurement | Result |
|---|---:|
| Peak measured hand displacement | 10.86 mm |
| Position error after release | 1.45 mm |
| Final orientation error | 0.0000128 rad |
| Python updates inside Gazebo | 1,157 |
| Native step mean / maximum | 31.18 / 3,129.7 microseconds |

The callback PID matched Gazebo, and the stock controller was restored. Timing
includes the Python-to-native call on this run. It is not a comparative speedup
measurement or a real-time guarantee. Python allocation, GIL acquisition, native
allocation and scheduling remain in the path.

This is a local arm compliance example. It has no collision avoidance and does
not move the torso or base. The bounded Cartesian reference may be unreachable;
regularization and joint limits can produce tracking and orientation error.
The force is synthetic and is not a physical wrench applied to the simulation.
Measured wrist-force input and contact-based placement remain future extensions.
