# TIAGo wrist force/torque investigation

The earlier TIAGo++ 5 N check was direction-limited: its result did not establish
general wrist accuracy. A common probe now tests both robots with identical
simulated-time windows, one wrist at a time, signed world-frame force vectors,
baseline subtraction and release checks. It saves raw samples as well as means
and standard deviations. This is a simulation integration check, not hardware
calibration or validation of contact-rich force control.

## Reproduced original behavior

Both original configurations failed directional checks. Representative changes
in measured force magnitude, for a 5 N applied load:

| Robot / direction | Left | Right |
| --- | ---: | ---: |
| TIAGo++ world X | 2.753 N | 2.735 N |
| TIAGo++ world Y | 4.975 N | 4.980 N |
| TIAGo++ world Z | 4.550 N | 4.564 N |
| PRO world X | 4.843 N | 4.942 N |
| PRO world Y | 4.821 N | 4.893 N |
| PRO world Z | 3.973 N | 4.054 N |

Signed Z reversal, 10 N loads and repeated 5 N loads were also recorded.
The original TIAGo++ positive-Z check could pass a 0.5 N tolerance while its X
response remained wrong. PRO's Z response also differed between launches;
its historical approximately 2.4 N result was not a universal correction gain.

The historical PRO approximately 12 N result had a separate test error:
Gazebo's world-frame force service adds `reference_point × force` to the
requested couple. Supplying the sensor's world position introduced an unwanted
torque. The corrected probe uses zero reference point and zero requested torque
for pure force at the targeted link's center of mass.

## Controlled comparisons and native configuration fix

The stock physics configuration is ODE `quick`, 1 ms step, 50 iterations,
SOR 1.3, CFM 0 and ERP 0.2. Raising only the iteration count to 1000 did not
repair TIAGo++ X response: it remained 2.742 N and 2.733 N. That experiment was
discarded; neither workspace changes solver settings.

For TIAGo++, the native `gazebo_ros2_control` implementation classifies joints with command
interfaces as actuated. With its default `hold_joints=true`, it calls
`SetVelocity(0)` on inactive actuated joints every physics update. Our minimal
launches declared gripper command interfaces but launched no gripper controller.
Those kinematic finger holds alter the load reactions reported at the wrist.

Changing only the stock `hold_joints` setting to false restored TIAGo++ X
response to 5.004 N on each wrist. The final change is narrower: the generated
model retains gripper state interfaces but removes their unused command
interfaces. The fingers are passive; normal holding behavior remains for the
head, torso and other joints. Vendor sources are unchanged. Future gripper
actuation requires a deliberately configured physical controller; it must not
silently reintroduce these inactive kinematic holds.

PRO needed an additional correction. Its upstream planar-base plugin calls
`Model::SetLinearVel` and `Model::SetAngularVel` at 100 Hz. Gazebo implements
these by writing **every link's** velocity, including the articulated arms and
wrists. Removing unused gripper command interfaces alone still gave about
3.43 N for a 5 N Z load. Suppressing the planar velocity updates restored the Z
component to 4.967/4.999 N but allowed base movement, so that diagnostic still
failed the strict vector/recovery criteria.

A small patch to the existing native planar plugin commands only the physical
`base_footprint` link. At the original 100 Hz this restored force magnitude but
left base yaw drift and transverse-force errors. Matching base velocity updates
to the existing 1000 Hz physics rate removed that regression in the subsequent
strict checks. The original odometry publish rate was already 1000 Hz and stays
unchanged. Physics solver and wrist sensor rates are unchanged.

The patch has its own local library name; the installed plugin is untouched.
PRO's source lock pins Gazebo ROS 3.9.0 at
`ae6e5793a31f5927df89cb58007d2a5eb567baa8`.
`tiago_pro/patches/gazebo_ros_planar_move_base_link.patch` is applied to a build
copy, leaving the vendor checkout clean. `pixi run build-planar-plugin` builds
only that library, and `setup.sh` includes this step. `run_sim.py` selects its
absolute path in the generated model and validates the physical base link on
plugin load. No new force controller or estimator is introduced.

Relevant primary implementation:

- [GazeboSystem 0.4.10](https://github.com/ros-controls/gazebo_ros2_control/blob/0.4.10/gazebo_ros2_control/src/gazebo_system.cpp): `registerJoints` actuated flag, `write` inactive-joint hold and direct native F/T reads.
- [Gazebo ROS force system 3.9.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.9.0/gazebo_ros/src/gazebo_ros_force_system.cpp): COM force application and reference-point couple.
- [Gazebo Classic force/torque sensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/sensors/ForceTorqueSensor.cc): native joint feedback and measurement direction/frame.
- [Gazebo ROS planar move 3.9.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.9.0/gazebo_plugins/src/gazebo_ros_planar_move.cpp): model-wide velocity commands.
- [Gazebo Classic model velocity API](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/Model.cc): velocity setters iterate through all links.

## Reproduction

Final results on 18 October 2026:

| Configuration | Force cases | Maximum force-vector error | Pure-torque cases | Maximum torque-vector error |
| --- | ---: | ---: | ---: | ---: |
| TIAGo++, passive unactuated fingers | 12/12 passed | 0.153 N | 6/6 passed | 0.0238 Nm |
| PRO, passive fingers + base-link patch at 1000 Hz | 12/12 passed | 0.228 N | 6/6 passed | 0.0054 Nm |

Both original motion/sensor regressions passed after the changes. A further
5 N Z load after commanded base/arm motion measured 4.941/5.017 N on TIAGo++
and 5.011/4.983 N on PRO. All release checks passed. PRO's regression measured
0.499 m base displacement and approximately +0.1501 rad on arm joint 6 of each
arm; RGB/depth/point-cloud, scans, IMU, joint, odometry and wrist streams were
received. The PRO full force matrix recorded maximum base baseline-to-release
drift of about 1.0 mm and 0.00468 rad yaw. This is bounded motion in the tested
scene, not an assertion of perfect static equilibrium or real robot dynamics.

All test simulators and ROS launches were stopped after these checks. No GUI
was opened for the investigation, and no other simulator ran concurrently.

Start exactly one robot using its guarded headless task, then run these in
another terminal in that same workspace:

```bash
# TIAGo++
pixi run ft-check
pixi run ft-torque-check

# PRO
pixi run ft-check
pixi run ft-torque-check
```

The force matrix applies +5 N on X/Y/Z, +10 N Z, -5 N Z, and a repeated +5 N Z,
to both wrists. The torque matrix applies pure 0.5 Nm couples on X/Y/Z.
Force-vector error and release residual must each remain below 0.5 N. Pure
torque tests additionally require torque-vector error below 0.05 Nm. No gains,
output rescaling or relaxed acceptance thresholds are used.

Reports live under `tiago_dual/evidence/` and `tiago_pro/artifacts/`:
`ft_matrix_original.json`, `ft_matrix_passive_grippers.json`,
`ft_torque_passive_grippers.json` (TIAGo++), `ft_matrix_corrected.json` and
`ft_torque_corrected.json` (PRO), and `ft_matrix_after_motion.json`, with
corresponding `.raw.json` files retained locally and ignored by Git. Small
TIAGo++ summary reports are committed; raw traces are not required at runtime.
Original failures and intermediate PRO
diagnostics remain available, including base-pose/joint-state data in the
later reports. Force tests apply at the targeted link's COM; their sensor
moments include the COM-to-sensor lever arm. Only pure-couple tests compare
sensor moment directly to requested moment. Original failures remain available.
Some unused wheel velocity state interfaces contain NaN in the upstream ROS
joint-state stream; raw state captures retain these instead of treating them
as measured velocities. Wrist wrench values used in the checks are finite.
