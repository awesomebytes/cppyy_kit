# Stretch 4 local feasibility probe

Run the stock mobile manipulator, check commands and real sensor outputs, then
open the viewer. These commands require this repository checkout:

```bash
cd cppyy_kit/roscon_uk_2026/next_steps/robot_playground/stretch4_probe
bash setup.sh
pixi run --locked probe
pixi run --locked viewer
```

The probe writes images, depth arrays and measured states under `artifacts/`.
The viewer stays open until it is closed or Ctrl+C is pressed. Its launch on
`DISPLAY=:1` passed and a captured window image shows the robot in the stock
kitchen scene. See `artifacts/viewer.png`.
The user confirmed that the viewer opened and the camera could be moved.

Run one simulator at a time. Both tasks use the shared simulator guard and
refuse to start if another guarded conference simulator is running. Stop the
viewer before running the probe, and wait for its shutdown to finish.

## Scope and observed result

This is the unmodified upstream simulator at revision
`0b20cf4fb5faf514ac40581582f9b64aa8f96e50`, running locally through Pixi.
It uses Python 3.12 and MuJoCo 3.5.0. Optional RoboCasa, Molmo and digital-twin
packages are excluded. No kitchen assets are required for the stock scene.
The local Pixi environment occupies about 2.7 GB and the source about 101 MB.

On 2026-10-18, `pixi run --locked probe` passed with exit status 0:

| Check | Observed result |
| --- | --- |
| Base command | Approximately 0.232 m displacement in 3 seconds at commanded 0.08 m/s |
| Lift command | Approximately 0.598 m to 0.496 m, approaching target 0.498 m |
| Telescopic arm command | 0.09997 m to 0.19995 m, target 0.19997 m |
| Simulation clock | Advanced continuously; upstream status reported approximately real time |
| Gripper RGB | 270 × 480 × 3 |
| Head RGB | 965 × 1280 × 3 |
| Gripper depth | 270 × 480; positive finite depths 0.07363–0.23339 m |
| Gyroscope | Finite three-vector |
| Accelerometer | Finite three-vector, vertical acceleration approximately 9.81 m/s² |
| Left / right 3D lidar | 31,307 / 31,513 finite XYZ points in the final run |
| Shutdown | Upstream physics loop and child process terminated; exit status 0 |

These figures establish a working stock scene and interfaces. They do not
measure application performance, establish realistic sensor noise, or verify
ROS integration. Depth is simulated geometric depth; it is not evidence of a
stereo reconstruction algorithm. Sensor rendering uses EGL graphics, while
simulation and application processing remain ordinary CPU execution.

## Architectural fit

Stretch 4 is the selected starting simulator because it combines an actuated
mobile base, manipulation, images, depth and 3D lidar with an existing upstream
[Jazzy ROS integration](https://github.com/hello-robot/stretch4_ros2/tree/jazzy/stretch_simulation).
That ROS stack still needs its own dependency solve, launch and interface checks.
The separate Stretch 3 environment in the parent directory remains a working
Humble-oriented fallback.

The simulator starts a child process and exchanges data through Python
`multiprocessing.Manager` proxies. Keep that baseline visible when comparing
future native composition. This probe does not use cppyy_kit or ros2_control.

## Installation details

Use the checked-in lock. The environment uses conda-forge `evdev` because PyPI
evdev 2.0 and 1.9.2 failed to build against the environment's Linux input headers
with undeclared constants such as `KEY_SIDEVU_SONAR`. No system header changes
are needed. Upstream code generates URDF/MJCF assets at runtime; the source
checkout and generated assets are ignored rather than bundled into this repo.

Sources: [upstream simulator](https://github.com/hello-robot/stretch4_mujoco),
[dependencies](https://github.com/hello-robot/stretch4_mujoco/blob/0b20cf4fb5faf514ac40581582f9b64aa8f96e50/pyproject.toml),
[architecture](https://github.com/hello-robot/stretch4_mujoco/blob/0b20cf4fb5faf514ac40581582f9b64aa8f96e50/docs/architecture.md).
