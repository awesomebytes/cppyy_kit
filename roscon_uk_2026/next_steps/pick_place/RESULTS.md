# Measured results

Measurements below were collected on this machine on 19 October 2026. They are
specific to the locked environment, simulator scene and stated workload.

## Physical manipulation

The custom controller's [fresh composed application run](runs/composed-contact.json) selected the red
object from live RGB/cloud association and completed a physical contact pick and
place. Object lift was **93.3 mm** and placement error was **30.4 mm**, below the
60 mm success threshold. Attachment assistance was enabled as a contingency but
was not used. The application exited with status 0. Its 35 measured waypoint
reaches totalled 134.9 seconds of wall time, excluding startup and gripper waits.
This is one successful run, not a measured success rate.

The [fresh distributed application run](runs/distributed-contact.json) also
completed with contact, using the same measured RGB/cloud selection. It lifted
the object 94.0 mm and placed it within 35.3 mm of the requested centre. Total
elapsed time was 142.53 seconds; waypoint motion accounted for 134.00 seconds.
All three native perception processes joined their workers with zero exceptions,
and the application exited with status 0. No attachment assistance was used.
These are individual successful runs, not a measured success rate.

The simulator was headless with MoveIt and Nav2 disabled for this validation:

```bash
# Requires this checkout, from roscon_uk_2026/next_steps/tiago_pro_jazzy
pixi run playground-headless moveit:=false navigation:=false
```

The controller uses existing Gazebo joint controllers and planar base control.
Its preparation is slower than perfect-following offline integration because the
simulator position interface tracks short position increments gradually. No
simulator gain changes were made for the successful run.

The [contact integration run](runs/contact-integration.json) completed a physical
pick and place without attachment assistance. Measured object lift was 94.7 mm;
placement error was 3.4 mm. That integration probe used the known red-object
location and continued from a partly prepared pose. It validates the gripper and
control sequence, rather than the complete perception-to-action application.

Arm-to-arm handover remains experimental. The current offline two-arm sequence
stalls during presentation at 37.3 mm position error and 0.141 rad orientation
error. It has not demonstrated a physical transfer. The optional flag is retained
for further work and is not part of the validated conference sequence.

## Perception and deployment

The [recorded replay](replay_evidence.json) detects three tabletop clusters and
matches them to red, blue and yellow image regions. NumPy and PCL compute 9,181
voxel centroids from the same finite ROI points with an 8 mm grid. Their maximum
centroid difference is 2.38e-7 m. Median voxel processing was 15.34 ms for NumPy
and 1.31 ms for PCL, including its NumPy input/output copies. These numbers do not
compare full applications or establish a universal PCL speedup.

The [original perception measurement](perception_evidence.json) recorded 3.26 ms
median cloud processing and 4.78 ms median image processing over 30 warmed calls.
The cloud measurement includes plane segmentation and clustering of a cropped
recorded frame. It excludes ROS transport and first compilation.

[Live composed observation](observation.json) selected a red cluster at
`[-2.849978, 2.2290874, 1.0533767]` m in `odom`. Its projected centre lies inside
the matching image box. [Distributed observation](evidence/composition/distributed_observation.json)
also selected the red object from live inputs. Shape names describe 2D
silhouettes; they are not reliable physical cube/cylinder classifications.

The [live deployment comparison](evidence/composition/live_summary.json) used the
same native algorithms in short sequential runs:

| Measurement | Composed | Separate processes |
|---|---:|---:|
| Application CPU, cores | 0.277 | 0.352 |
| Median cloud observer latency, ms | 25.7 | 20.1 |
| p95 cloud observer latency, ms | 58.8 | 52.3 |
| Median cloud age, simulation ms | 41 | 43 |
| p95 cloud age, simulation ms | 65 | 113 |

All processes joined their workers and reported zero native exceptions. Live
inputs differed between runs. Observer latency starts when an external observer
receives the source frame and includes its scheduling. CPU excludes Gazebo and
the observer. These results do not establish a latency speedup from composition.

## Custom controller

The [controller evidence](controller_evidence.json) includes:

- Native forward kinematics matched live TF within 0.8 micrometres at the checked
  pose. This checks conventions, not all URDF configurations.
- A measured-feedback 15 mm live hand lift ended within 1.61 mm of the target.
- A real ros2_control manager with mock hardware ran the whole-body task adapter
  for 101 callbacks and reduced a 36 mm position error to about 0.027 mm.
- Separate sphere, capsule and box crossing probes preserved a configured 40 mm
  sampled arm clearance. They do not establish full-robot collision safety.
- The original obstacle-free task step took about 0.53 ms. Primitive obstacle
  probes took roughly 8–16 ms per step, depending on the shape.

The evidence records parameters used for each run. Subsequent gain/damping tuning
is listed separately. The controller uses weighted preferences and local
linearization, joint bounds, and bounded velocities. It has no global convergence
or real-time guarantee. Base/torso collision and robot self collision are not
implemented by the sampled arm obstacle check.

After the readability refactor, the existing eight-stage offline route probe
completed with a maximum final stage position error of 9.65 mm and orientation
error of 0.0216 rad. This uses perfect position following, not object dynamics.
The controller and URDF wrapper now use named solve/integration operations and
shared robot configuration. Project flake8 and Ruff formatting checks pass for
the controller, URDF wrapper, configuration, demo entry point and ROS adapter.

## Placo controller

The [final composed Placo run](runs/placo-contact.json) completed live
perception-driven pick, lift, placement, release and retreat from a fresh
simulator state. Measured object lift was **84.8 mm** and placement error was
**16.8 mm**. It used physical contact without attachment assistance and exited
with status 0. The 36 waypoint reaches took 116.1 seconds in total, excluding
startup and gripper waits. Full robot self-collision constraints remained enabled.
This is one successful run, not a measured success rate; Placo handover and a
distributed Placo manipulation run were not validated.

The separate `pick-place-placo` environment runs Placo 0.10.0, Pinocchio 4.1.0,
ROS 2 Python interfaces and cppyy in one process. Native PCL/OpenCV replay found
the same three recorded objects, and a `cppyy_kit` compiled call returned the
expected result. Existing environments retained their locked package entries.
The new environment pins GCC 14: activated GCC 15 produced a Cling startup
segmentation fault, while GCC 14.4 passed import and compilation on this machine.

[Placo controller measurements](evidence/placo_controller.json) cover all 61
original URDF collision shapes and 1,114 self-collision pairs after SRDF
exclusions. At the ready pose, a conservative bounding-box filter selected 59
nearby pairs for exact native distance queries. Full and filtered queries agreed
on the minimum clearance, 4.595 mm. The original unfiltered controller step took
about 1.04 seconds; the final filtered 40-step reach probe averaged 25.3 ms, with a
36.3 ms maximum. These measurements are pose-dependent, not a worst-case bound.

In that offline probe, a 30 mm position goal converged to 5.3 micrometres under
perfect command following. A moving goal blocked by a sphere stopped at a
minimum measured 2.0002 mm clearance with a configured 2 mm margin. A deliberately
penetrating environment obstacle caused an infeasible solve and an unchanged
joint command. A self-penetrating state instead admitted a bounded recovery step
that reduced penetration; the controller does not imply that such a starting
state is collision-free.

The [live 15 mm hand lift](evidence/placo_live_reach.json) completed in 4.38 seconds
with 1.97 mm final error, using measured feedback and the existing native Gazebo
trajectory controllers. Full robot self-collision checking was enabled; base
motion was disabled for this reach.

The original preparation route stalled at a collision boundary. The
[outward preparation probe](evidence/placo_route_outward.json) first moves the hand
180 mm forward, 150 mm outward and 65 mm up while preserving its orientation,
then turns it downward. The offline route through approach, descent and lift
completed with position error below 10 mm and orientation error below 0.038 rad,
with base Y bounded at 1.60 m. It uses the same table-clearance waypoint approach
as the custom demo, without adding table collision geometry automatically.

The first complete contact attempt lifted the object but stopped during retreat
on an infeasible QP. The exact transient state was not captured. A
[controlled feedback probe](evidence/placo_passive_feedback.json) reproduced that
failure mechanism by reporting a masked gripper joint 0.1 mm beyond its upper
limit. The backend now applies native position bounds only to commanded joints;
passive increments remain masked and their measured geometry still participates
in collision checking. Measured passive feedback also takes precedence over
mimic estimates. Neither fix removes collision pairs or loosens commanded-joint
limits.

## Native ros2_control controller

The [native Gazebo run](evidence/native_control/live_summary.json) loaded
`control_kit_native/PositionServo` into Gazebo's existing controller manager.
The controller claimed seven left-arm position command interfaces while the
stock arm controller was inactive. A 0.04 rad target increment on the last wrist
joint produced 0.037321 rad of measured motion, with 0.002679 rad maximum final
target error. The example returned toward the initial configuration, restored
the stock controller, unloaded the custom controller and exited with status 0.
The report includes controller states and claims before, during and after the run.

An [interrupted run](evidence/native_control/interrupt_summary.json) also restored
the stock controller and unloaded the custom controller after SIGINT. The owned
simulator then [shut down cleanly](evidence/native_control/gazebo_controller.log).

The native controller's lifecycle and update methods are compiled C++. Its native
ROS callback accepts joint targets. Python builds the library and performs ROS
setup, target publication and observation. This demonstration does not put the
WBC inside the native loop or replace the default pick-and-place actuation path.
It checks joint-position control, not torque or Cartesian control.

The in-process mock-hardware demo also passed: two joints reached `[0.6, -0.35]`
rad with maximum error below 0.000001 rad after three seconds. Native shared
ownership survived removal of the Python controller handles. New Python files
pass the repository's flake8 configuration. No test files were added.

The controller law is ordinary bounded proportional position control. The
cppyy_kit contribution is cached compilation, direct Python configuration without
custom bindings, and generated plugin export metadata. It does not automatically
translate Python into C++ or outperform an equivalent conventional C++ plugin.

## Integration findings

A direct combined reach/rotation from the initial low hand pose can cross the
table edge. Raising first and rotating at a fixed high pose can instead enter an
unreachable local branch. The preparation sequence therefore rotates before the
table at a lower pose, lifts the downward-facing hand, then approaches the object.
Offline kinematic convergence is distinct from physical grasp success.

The application changes also exposed reusable fixes:

- `rclcpp_kit` native stages now accept external C++ include/link arguments and
  best-effort sensor input QoS.
- `control_kit` supplies native URDF FK, a weighted solve, and a position-task
  adapter for the real ros2_control controller interface. Its native header is
  included in package data.
- Native startup selects the matching Conda C++ compiler when `CXX` is absent.
  Removing that variable reproduced a Cling startup crash. Activated `pixi run`
  remains the supported command used for these measurements.
- Sensor-frame transforms use the sensor timestamp. A native worker waits at
  most 100 ms for matching TF, then suppresses that frame if unavailable.

No test files were added. Validation used executable probes, recorded-frame
replay, live sensor observations, lifecycle checks and measured robot commands.

## Python ros2_control plugin

[Live evidence](python_control_result.json) records 929 Python update calls in
Gazebo PID 2058262, distinct from observer PID 2058386. The plugin was mapped in
that process. All seven left-arm position interfaces were claimed. The wrist
moved 0.037050 rad toward a 0.040000 rad offset, with 0.002950 rad target error.
After return, wrist error was 0.002486 rad. The stock controller was restored,
the custom controller unloaded, and the owned simulator stopped.

The law is Python; the generic bridge is C++. This verifies process placement,
interface ownership and motion, not hard real-time timing or WBC execution in
ros2_control. The existing native example remains available for comparison.


## Python admittance with native kinematics

The [interactive guide](ADMITTANCE.md) documents the virtual-force example.
The [automated run](evidence/admittance/live.json) applied 1 N along torso-frame X
for eight wall seconds, then released it for eight seconds. Measured peak hand
displacement was 10.86 mm and return error was 1.45 mm. The Python controller
executed 1,157 updates in Gazebo and called the compiled FK/Jacobian/DLS kernel.
The stock controller was restored and the custom controller unloaded.

The [browser input probe](evidence/admittance/ui-force-expiry.json) recorded hand
motion under held force, followed by zero force after refreshes stopped. The
[root Pixi task run](evidence/admittance/root-task.json) also restored the stock
controller. The owned simulator was stopped after validation.

The native call averaged 31.18 microseconds and reached a maximum of 3.13 ms in
the automated run. This measures the call boundary and native operation together.
It establishes neither a speedup over NumPy nor a hard real-time bound. The force
is synthetic; wrist sensor feedback and contact-based admittance were not tested.
