# In the simulator environment, after Gazebo starts with the overlay.
# CONFIG is the file returned by prepare_admittance().
ros2 run controller_manager spawner cppyy_admittance_arm_left \
  -p "$CONFIG" --inactive --controller-manager-timeout 30

ros2 control switch_controllers --strict --activate-asap \
  --deactivate arm_left_controller \
  --activate cppyy_admittance_arm_left --switch-timeout 10

# The RViz force UI publishes [Fx, Fy, Fz] to this topic.
ros2 topic pub --rate 20 /cppyy_admittance_arm_left/commands \
  std_msgs/msg/Float64MultiArray '{data: [2.0, 0.0, 0.0]}'

# Stop the publisher. Let the force timeout expire and the hand return.
ros2 control switch_controllers --strict --activate-asap \
  --deactivate cppyy_admittance_arm_left \
  --activate arm_left_controller --switch-timeout 10
ros2 control unload_controller cppyy_admittance_arm_left
