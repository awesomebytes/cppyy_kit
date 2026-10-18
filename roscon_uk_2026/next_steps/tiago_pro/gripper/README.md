# Physical mimic followers for the PRO gripper

Start the [unified playground](../README.md) with `pixi run playground` from
the PRO directory. In RViz select `gripper_left` or `gripper_right`, choose
`open` or `closed`, then Plan and Execute. Each primary screw is actively
controlled by the native effort-based GripperActionController. Its physical
followers use the force-based mimic PID implementation described below.
The robot spawns with coherent closed joints; no opening demonstration runs
at startup. The history below records earlier integration findings, including
the superseded passive-primary configuration.

For a deliberate full-stroke demonstration while other robot commands are
idle, run `pixi run gripper-demo`. This is an optional motion client, not setup.

The primary screw was stable, but the fourteen uncoupled follower joints
oscillated: sampled outer-finger ranges reached 0.376 rad. The right screw
stayed at 0.07 m instead of the URDF-required `0.22 × 0.07 = 0.0154 m`.
ROS joint states exposed only primary screws; native Gazebo joint observations
were needed to see the actual mechanism error.

This adapter builds the existing ROS 2
[roboticsgroup mimic plugin](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins/tree/8bf90f5702ec1cf3012c997df2ddfb2f891b9ec9)
at the exact revision in `sources.lock`. The explicit patch corrects a literal
PID parameter-prefix bug, rejects missing PID gains, removes the kinematic
`SetPosition` branch and fixes the success log severity. Its existing native
PID implementation applies capped `SetForce` to the followers; no custom
controller algorithm is added. Vendor source remains unchanged.

From the PRO workspace:

```bash
pixi run --locked bash gripper/build.sh
```

Before serializing the expanded URDF, the launch adapter calls
`gripper.model.add_physical_mimics(robot)`. It generates fourteen unique native
plugins from the existing URDF mimic ratios. The primary screws are controlled
by the native effort controllers described below. The followers remain passive
in MoveIt's joint-planning model, while native physical PID forces couple them
to their leaders in Gazebo. This does not establish verified object grasping.

Rotary followers use 1 Nm/rad proportional gain; the right screw uses 100 N/m,
matching the upstream MuJoCo finger proportional gain. Derivative gains follow
a nominal damping ratio 0.8 using child mass/inertia about the joint, with a
conservative bound for the 1 ms integration step. Integral action is zero.
Effort caps remain the URDF limits: 0.1 Nm for rotary joints and 10 N for the
screw. These are explicit simulation settings, not calibrated hardware gains.
The helper returns a ledger containing every gain, ratio and cap.

To observe actual Gazebo joints in the already running simulator:

```bash
pixi run --locked python gripper/observe.py --output artifacts/gripper_linkage.json
```

After any change, inspect the native linkage errors and motion ranges, repeat
after arm motion, and rerun the strict wrist load checks. Visual stability by
itself does not establish correct force sensing.

## Verified initial result

Eight native joint snapshots after restoring the force-based followers showed
rotary motion ranges no larger than **0.000099 rad**, compared with up to
**0.376 rad** in six baseline snapshots. Maximum angular linkage error was
**0.0071 rad** (0.41 degrees). The right screws settled around 0.0166 m versus
the ratio target 0.0154 m; their approximately 1.2 mm difference is compliant
PID deflection under load, not an exact mechanical gear constraint.

After successful left/right Cartesian and compact-posture MoveIt executions,
gravity closed the passive primary screws near their lower stops. Eight further
native snapshots showed rotary ranges up to **0.0228 rad** (1.30 degrees) and
linkage error up to **0.0190 rad** (1.09 degrees). Screw ranges and linkage
errors stayed below 8.4 micrometers. Thus restoring physical coupling fixes the
grossly disconnected mechanism, but the passive primary screws do not provide
a controlled opening or universally quiet posture. These later results are in
`artifacts/gripper_linkage_after_moveit.json`; do not generalize the open-pose
quietness above to every arm pose.

The restored model passed all **12 directional/signed force cases** and **six
pure-torque cases**. Maximum vector errors were **0.199 N** and **0.0058 Nm**,
with the original 0.5 N / 0.05 Nm criteria unchanged. Evidence is saved under
the PRO workspace's `artifacts/` as `gripper_native_jitter_baseline.json`,
`gripper_linkage_physical.json`, `ft_matrix_physical_gripper.json` and
`ft_torque_physical_gripper.json`, with raw wrist samples alongside the reports.

The helper uses private `~/mimic/pid_state` diagnostic topics for each unique
plugin node, so separate followers do not mix measurements on one topic. The
private-topic adjustment does not change PID parameters or controller behavior.

## Native control of the primary screws

`configure_primary_effort_control(robot)` prepares the installed stock
`effort_controllers/GripperActionController` for each primary screw. It adds
only `effort` command interfaces and appends `controllers.yaml` to the native
controller manager's parameter files. The returned controller names must be
included in the launcher's normal controller-spawner sequence. It also removes
the ros2_control-only follower mimic parameters: otherwise GazeboSystem would
propagate the primary effort into followers already owned by the physical PID
plugins. URDF mimic relationships remain intact, and actual follower position
and velocity state interfaces become available for observation.

The main controllers run at the existing manager's 100 Hz. P = 100 N/m matches
the upstream MuJoCo finger proportional gain. D = 2 Ns/m is approximately
critical damping for the 0.02 kg screw at that stiffness; it is not a large
artificial joint damping value. I = 50 N/(m s) gives a slow two-second gravity
correction timescale, with integral effort capped at 0.5 N. That cap exceeds
the screw's 0.196 N nominal weight but remains well below the unchanged URDF
10 N total effort limit. Goal tolerance is 1 mm; stalling is not accepted as
successful positioning. These are conservative simulation settings requiring
the runtime checks below, not claimed manufacturer tuning.

After integration and activation, the actual action test is:

```bash
ROS_DOMAIN_ID=77 pixi run --locked python gripper/command.py
```

It sends native `control_msgs/action/GripperCommand` goals to 0.04 m then
0.065 m on both hands, checks successful non-stalled results and actual joint
positions after a further holding interval. It records any action feedback
provided by the stock controller; measured `/joint_states` provides independent
position/velocity feedback even when an action does not stream feedback.
Computed action-result effort is a controller command, not a calibrated force
measurement. Repeat native linkage observations and strict wrist loads after
enabling this physical actuation.

### Observed opening, closing and holding

In the unified playground's raised, mesh-checked pose (lowest gripper geometry
above 1.42 m), both native effort controllers successfully opened to 0.065 m
and then closed to 0.04 m. Each action returned `SUCCEEDED`, `reached_goal=true`
and `stalled=false`. After another 2.5 simulated seconds of holding, opening
errors were 67/35 µm and closing errors 66/95 µm (left/right), against the
unchanged 1 mm criterion. No gain or tolerance changes were needed.

Reports are `artifacts/gripper_open_raised.json` and
`artifacts/gripper_close_raised.json`; both include measured joint traces.
The subsequent twelve native Gazebo snapshots in
`artifacts/gripper_linkage_effort_hold.json` show at most 49.3 µm primary-screw
motion and 0.046° rotary-follower motion. Physical PID coupling remains
compliant: static linkage offsets reach 1.19° and 1.50 mm at the secondary
screw. These observations demonstrate free-space opening, closing and stable
holding; they do not establish accurate object grasping or contact behavior.

With these effort controllers active, the unchanged strict 5 N world-Z wrist
check passed on both sides (`artifacts/ft_quick_primary_effort.json`, including
a separate raw-samples file). Measured force-change norms were 4.873/4.711 N;
world-vector errors were 0.186/0.363 N. Release residuals were 0.340/0.487 N,
the latter close to the existing 0.5 N limit. This is a two-case regression,
not a new full directional matrix for the effort-actuated configuration.
All applied loads were cleared afterward.

An earlier first close request was accepted and physically moved the left
screw, but did not finish within the client's 25-second timeout. Its preserved
failure and cancellation must not be interpreted as reliable first-command
behavior. Inspection of the stock Humble effort adapter found that
`starting()` resets the PID without initializing `last_update_time_`; its
first update therefore measures from a default steady-clock time point.
The PID also defaults to output-only integral clamping (`antiwindup=false`).
This can produce startup integral windup, but the initial error and internal
integral were not captured, so it remains a possible explanation rather than
a demonstrated cause of that timeout. No unverified controller patch was
applied. The command client now cancels its own active goal on failure and
preserves its request, joint trace and error report.

For a visible demonstration with the robot stationary in the raised pose:

```bash
pixi run gripper-demo
```

This derives the full closed/open screw coordinates from the expanded model's
actual primary-joint limits: **0.000 m fully closed and 0.070 m fully open**.
It verifies that every follower's limit permits the full lead-joint interval.
At full opening, rotary mimic targets are ±0.5796 rad and secondary-screw
targets are 0.0154 m, all inside their own limits. The upstream PAL wrapper
[`gripper.yaml`](../vendor/pal_pro_gripper/pal_pro_gripper_wrapper/config/gripper.yaml)
explicitly defines `close_value: [0.0]` and `open_value: [0.07]`; its named
left-hand motions call 0.03969 m **half closed**. The earlier 0.040/0.065 m
tests above therefore demonstrate interior positioning, not full travel.

The demo sends both hands together through three full open/close cycles,
holds each endpoint for 4.5 wall seconds, then leaves both hands fully open.
These targets are screw-joint positions, not measured jaw aperture.
It commands no arms or base. The owned client can also be run directly with
`ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp pixi run --locked python
gripper/demo.py --cycles 3 --hold 4.5`; its default report is
`artifacts/gripper_full_visual_demo.json`. It records all sixteen actual
native joint coordinates, ideal mimic values and action results at every
endpoint. A reached-goal or expected stalled-at-stop action still requires
measured lead positions within the unchanged 1 mm tolerance of the actual
joint limit; an unsuccessful action is rejected. Failures cancel pending goals.
