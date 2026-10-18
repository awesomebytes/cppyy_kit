# Start and stop robot playgrounds

The TIAGo and Stretch experiments share the workstation's CPU, memory and
display. Run one simulator at a time, including headless checks. Other robot
playgrounds remain available; stop the current one before switching.

## TIAGo PRO

From this repository checkout:

```bash
cd cppyy_kit/roscon_uk_2026/next_steps/tiago_pro
./setup.sh
pixi run playground
```

`playground.launch.py` starts the home simulator, model publisher, native
controllers, MoveIt, Nav2 and RViz. Controllers activate before the selected
robot servers/viewers start. Ready joint values are initialized directly;
startup does not execute homing or motion tests. See the
[PRO guide](tiago_pro/README.md) for launch arguments and robot controls.

| Task | Starts |
|---|---|
| `playground`, `playground-home` | Full home stack with viewers |
| `playground-headless` | Full home backends without viewer windows |
| `view`, `headless` | Simulation and native controllers, with/without Gazebo GUI |
| `moveit`, `navigation`, `rviz` | Selected subsystem against an existing simulator |

Attach tasks use the same launch description with simulation disabled. They
must not duplicate servers already running in a full stack. Viewer closure
does not stop the backends. Ctrl-C in the owning launch terminal stops its
processes; wait for shutdown before starting another robot.

## Shared simulator guard

Simulator-starting PRO tasks already use `../simulator_guard.sh`. For another
playground's entrypoint that is not already guarded, wrap its complete command:

```bash
bash ../simulator_guard.sh -- pixi run sim
```

The guard uses `flock` at
`${XDG_CACHE_HOME:-$HOME/.cache}/cppyy_kit/simulator.lock`. A competing launch
exits with status 75. Nested guarded launchers reuse the inherited descriptor
after checking its ancestry. The lock persists through child shutdown;
SIGINT/SIGTERM reach the executed command.

The guard neither stops existing processes nor detects simulators launched
without it. Do not delete the lock file to bypass a running launch. Distinct
ROS domains and Gazebo master addresses are additional isolation, not a reason
to run several resource-heavy playgrounds together. PRO's saved defaults use
ROS domain 77, Fast DDS and Classic master port 11377.

The existing `test_simulator_guard.py` checks lock behavior with dummy processes;
it does not launch Gazebo or ROS and is not part of ordinary startup.
