# Harmonic native integrations

Build from `tiago_pro_jazzy` with `pixi run bash build_native.sh` before generating the model. `native_model.configure(robot)` attaches the plugin and returns ROS/Gazebo bridge entries.

The base uses Harmonic `VelocityControl` and `OdometryPublisher`. Commands accept body-frame holonomic Twist on ROS `/mobile_base_controller/cmd_vel_unstamped`. Odometry on `/mobile_base_controller/odom` uses the simulated world pose with frame `odom` and child `base_footprint`, preserving the world origin at a nonzero spawn position. Prescribed base velocity does not model wheel traction.

The physical follower plugin applies bounded PD forces to fourteen gripper follower joints using the URDF multiplier and offset. It never sets joint position or velocity. The two lead joints retain stock effort GripperActionControllers. Followers have state interfaces only, and `mimic="false"` disables the additional gz_ros2_control velocity servo. Gains start from the Humble nominal child inertia calculation; contact behavior needs the shared DART simulation smoke.

Stock gz_ros2_control mimic sends `JointVelocityCmd` to followers rather than bounded physical efforts. Native wrist force/torque sensing remains owned by gz_ros2_control and the world ForceTorque system.

Primary implementation references:

- [Harmonic VelocityControl](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/velocity_control/VelocityControl.cc)
- [Harmonic physics command handling](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/physics/Physics.cc)
- [Harmonic world-pose odometry](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc)
- [Jazzy gz_ros2_control mimic implementation](https://github.com/ros-controls/gz_ros2_control/blob/jazzy/gz_ros2_control/src/gz_system.cpp)

Build verified against installed gz-sim 8.10.0. Runtime base, gripper, and force/torque validation belongs to the migration lead's single shared simulator session.

The simulator URDF must strip gripper `<mimic>` tags after this plugin captures their parameters. sdformat14 converts those tags into native SDF axis constraints, creating another follower actuator. Keep the tags in the robot-state-publisher description, and remove them only from the simulator copy.
