# Code for the robot demo slides

From the repository root, run the recorded demos with:

```sh
./roscon_uk_2026/run_robot_demo.sh baseline --gazebo-gui
./roscon_uk_2026/run_robot_demo.sh forward --gazebo-gui
./roscon_uk_2026/run_robot_demo.sh admittance --gazebo-gui
```

Run one at a time. Stop each with Ctrl-C and wait for cleanup.

**The two setup files are presentation extracts. The demo does not import or run them. They define ordinary Python functions, not ROS launch descriptions.**

The actual execution is:

- Navigation: `run_robot_demo.sh` → `robot_demos/run.py` → `playground.launch.py` → `navigation/nav.launch.py` → Nav2 loads the kit adapter → the adapter calls `ForwardCameraPolicy` inside `controller_server`.
- Arm: `run_robot_demo.sh` → `robot_demos/run.py` prepares the adapter and starts Gazebo → `admittance_demo.py` writes parameters, loads and switches the controller → the kit adapter calls `CartesianAdmittance` inside Gazebo. The demo process also hosts the RViz force UI and restores the stock controller on exit.

`nav2_setup.py` expresses the forward-profile configuration from the simulator launch as a complete function. Pass the existing controller-server parameter dictionary, repository path and artifacts directory before starting Nav2. The stock robot-specific Nav2 configuration is in `next_steps/tiago_pro_jazzy/navigation/nav.launch.py`.

`controller_setup.py` expresses the plugin export and arm configuration as a complete function. Pass the repository path, generated robot URDF and overlay directory. Run it in the launcher process before Gazebo starts so Gazebo inherits the overlay. It returns the configuration filename used as `CONFIG` in `controller_lifecycle.sh`. The normal demo launcher handles these operations and the force UI automatically.

The slides contain two code slides per kit: the registration/parameter additions to existing robot bringup, followed by a Python class. `forward_camera_policy.py` here is the fixed-default teaching version of the recorded Nav2 policy. Its scoring behavior was compared with the repository policy on 324 candidate cases. The repository policy adds configurable options and validation.

`hold_position.py` is the complete minimal controller shown in the slides. Save its parameters as `hold.yaml`, select the robot's position joints, and point `python_path` at this directory. Start the controller manager with the kit overlay available, then load it with `ros2 run controller_manager spawner hold -p hold.yaml`. Interfaces must be free; if a stock controller owns them, load inactive and switch controllers using the manager. It holds the positions captured on activation and rejects target changes.

The admittance video demonstrates a richer application of the same adapter. Its full Python law, Eigen computation and force UI remain in the example sources below. The file-overview slides and speaker notes explain the setup order and where Python runs.

Reusable integration code:

- `nav2_kit/nav2_kit/python_critic.py`, `.hpp`, `.cpp`: build/export and DWB adapter.
- `control_kit/control_kit/python_control.py`, `python_controller.hpp`, `.cpp`: build/export and ControllerInterface adapter.

Example code:

- `nav2_kit/demos/forward_camera_policy.py`: forward-facing trajectory policy.
- `control_kit/demos/python_admittance_controller.py`: Python controller lifecycle and admittance law.
- `control_kit/demos/arm_kinematics.py`: URDF chain loading and cppyy_kit calls.
- `control_kit/demos/arm_kinematics.hpp`: Eigen FK, Jacobian and damped IK.
- `control_kit/demos/admittance_rviz.py`: virtual-force interactive marker UI.

All paths above are relative to the repository root. The setup files define functions for use inside an existing robot launcher; the top-level demo commands supply that launcher.
