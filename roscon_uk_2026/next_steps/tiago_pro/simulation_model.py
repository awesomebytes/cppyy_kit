"""Prepare the pinned PRO model with the documented Classic/control corrections."""
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from initial_pose import configure as configure_initial_pose
from gripper.model import add_physical_mimics, configure_primary_effort_control
ROOT = Path(__file__).resolve().parent

def prepare():
    import prepare_model  # Build the local public-description resource index.
    xml = (ROOT / "generated/tiago_pro.urdf").read_text().replace(
        str(Path(get_package_share_directory("tiago_pro_description")) / "ros2_control/gazebo_controller_manager_cfg.yaml"),
        str(ROOT / "controllers.yaml"))
    robot = ET.fromstring(xml)
    for plugin in robot.findall("gazebo/plugin"):
        if plugin.get("filename") == "libgazebo_ros_planar_move.so":
            library = ROOT / "build/planar_plugin/libcppyy_kit_gazebo_ros_planar_move.so"
            if not library.is_file():
                raise RuntimeError("Build the native base-link patch with: pixi run build-planar-plugin")
            # Stock Model::SetLinearVel resets every link, corrupting articulated
            # wrist reactions. This small upstream patch commands only the base.
            plugin.set("filename", str(library))
            ET.SubElement(plugin, "body_name").text = "base_footprint"
            plugin.find("update_rate").text = "1000"
    for gazebo in robot.findall("gazebo"):
        for plugin in list(gazebo.findall("plugin")):
            if plugin.get("filename") == "libgazebo_ros_mimic_joint.so":
                gazebo.remove(plugin)
    # Public ATI description puts tool and sensor on parallel branches. Route the
    # unchanged tool pose through the sensing joint so distal loads cross it.
    for side in ["left", "right"]:
        sensing = robot.find(f"joint[@name='wrist_{side}_ft_sensor_joint']")
        tool = robot.find(f"joint[@name='wrist_{side}_ft_tool_joint']")
        sensing_xyz = [float(value) for value in sensing.find("origin").get("xyz").split()]
        tool_xyz = [float(value) for value in tool.find("origin").get("xyz").split()]
        tool.find("parent").set("link", f"wrist_{side}_ft_sensor_link")
        tool.find("origin").set("xyz", " ".join(str(a - b) for a, b in zip(tool_xyz, sensing_xyz)))
    for sensor in robot.findall(".//sensor"):
        # Gazebo Classic uses CPU ray sensors; gpu_lidar is Gazebo Sim syntax.
        if sensor.get("type") == "gpu_lidar":
            sensor.set("type", "ray")
        if sensor.get("type") == "force_torque":
            ET.SubElement(sensor, "always_on").text = "true"
    # Remove the original gripper position commands before installing the physical
    # effort controllers below. Inactive command interfaces otherwise introduce
    # artificial SetVelocity(0) holds that corrupt wrist reaction measurements.
    for joint in robot.findall("ros2_control/joint"):
        if joint.get("name", "").startswith("gripper_"):
            for interface in list(joint.findall("command_interface")):
                joint.remove(interface)
    # The Classic depth plugin emits optical XYZ at the depth sensor origin.
    for sensor in robot.findall(".//sensor"):
        if sensor.get("name") == "head_front_camera" and sensor.get("type") == "depth":
            sensor.find("plugin/frame_name").text = "head_front_camera_depth_optical_frame"
    gripper_controllers = configure_primary_effort_control(robot)
    configuration = add_physical_mimics(robot)
    initial_positions = configure_initial_pose(robot)
    (ROOT / "artifacts").mkdir(exist_ok=True)
    (ROOT / "artifacts/spawn_initial_positions.json").write_text(json.dumps(initial_positions, indent=2) + "\n")
    (ROOT / "artifacts/gripper_physical_config.json").write_text(json.dumps(configuration, indent=2) + "\n")
    # ROS consumers need valid resource URIs. Resolve package URIs only in the
    # separate Gazebo spawn copy, keeping the published ROS description portable.
    ros_description = ET.tostring(robot, encoding="unicode")
    (ROOT / "generated/robot_description.urdf").write_text(ros_description)
    xml = re.sub(
        r"package://([^/]+)/",
        lambda match: get_package_share_directory(match[1]) + "/",
        ET.tostring(robot, encoding="unicode"))
    (ROOT / "generated/sim.urdf").write_text(xml)
    return ros_description, gripper_controllers
