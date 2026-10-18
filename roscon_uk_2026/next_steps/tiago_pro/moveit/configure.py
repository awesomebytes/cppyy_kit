"""Generate stock MoveIt parameters for the actual ATI-equipped simulator."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUTPUT = HERE / "generated"


def configure():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    prefix = OUTPUT / "prefix"
    for package, source in [("tiago_pro_moveit_config", HERE / "upstream"),
                            ("pal_sea_arm_moveit_config", HERE / "pal_sea_arm_moveit_config")]:
        if not (source / "package.xml").is_file():
            raise RuntimeError("Fetch pinned MoveIt config sources with bash moveit/fetch.sh")
        marker = prefix / "share/ament_index/resource_index/packages" / package
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.touch()
        link = prefix / "share" / package
        if not link.exists():
            link.symlink_to(source, target_is_directory=True)
    os.environ["AMENT_PREFIX_PATH"] = ":".join([
        str(prefix), str(ROOT / "generated/prefix"), os.environ.get("AMENT_PREFIX_PATH", "")])

    # sim.urdf includes the wrist sensing-chain and passive-gripper fixes.
    # Restore only resource URIs; do not regenerate a different kinematic model.
    canonical = ROOT / "generated/robot_description.urdf"
    robot_xml = (canonical if canonical.exists() else ROOT / "generated/sim.urdf").read_text()
    robot = ET.fromstring(robot_xml)
    share = str(ROOT / "generated/prefix/share") + "/"
    for mesh in robot.findall(".//mesh"):
        path = mesh.get("filename", "")
        if path.startswith(share):
            mesh.set("filename", "package://" + path[len(share):])
    robot_xml = ET.tostring(robot, encoding="unicode")
    for side in ("left", "right"):
        if robot.find(f"link[@name='wrist_{side}_ft_sensor_link']") is None:
            raise RuntimeError("Expected current ATI wrist configuration")
        tool = robot.find(f"joint[@name='wrist_{side}_ft_tool_joint']")
        if tool.find("parent").get("link") != f"wrist_{side}_ft_sensor_link":
            raise RuntimeError("Run the corrected PRO simulator before configuring MoveIt")

    srdf_xml = xacro.process_file(str(HERE / "upstream/config/srdf/tiago_pro.srdf.xacro"), mappings={
        "ft_sensor_left": "ati", "ft_sensor_right": "ati",
        "wrist_model_left": "spherical-wrist", "wrist_model_right": "spherical-wrist",
        "has_teleop_arms": "False"}).toxml()
    srdf = ET.fromstring(srdf_xml)
    # MoveIt 2.5.9 accumulates its Octomap in the model's planning frame.
    # Track the mobile base through TF so that frame remains fixed in odom.
    for virtual in list(srdf.findall("virtual_joint")):
        srdf.remove(virtual)
    ET.SubElement(srdf, "virtual_joint", {"name": "odom_base_joint", "type": "floating",
                                        "parent_frame": "odom", "child_link": "base_footprint"})
    # Upstream always defines optional teleop groups, even with no teleop arms.
    for group in list(srdf.findall("group")):
        if group.get("name", "").startswith("arm_teleop_"):
            srdf.remove(group)
    group = ET.SubElement(srdf, "group", {"name": "both_arms"})
    for side in ("left", "right"):
        ET.SubElement(group, "group", {"name": "arm_" + side})
    postures = yaml.safe_load((HERE / "postures.yaml").read_text())
    for name, arms in postures.items():
        planning_group = "both_arms_torso" if "torso_lift_joint" in arms else "both_arms"
        state = ET.SubElement(srdf, "group_state", {"name": name, "group": planning_group})
        for arm, values in arms.items():
            if arm == "torso_lift_joint":
                ET.SubElement(state, "joint", {"name": arm, "value": str(values)})
                continue
            for index, value in enumerate(values, 1):
                ET.SubElement(state, "joint", {"name": f"{arm}_{index}_joint", "value": str(value)})
    leaders = {f"gripper_{side}_finger_joint" for side in ("left", "right")}
    for passive_joint in list(srdf.findall("passive_joint")):
        if passive_joint.get("name") in leaders:
            srdf.remove(passive_joint)
    passive = {joint.get("name") for joint in srdf.findall("passive_joint")}
    for joint in robot.findall("joint"):
        name = joint.get("name", "")
        if (name.startswith("gripper_") and name not in leaders
                and joint.get("type") != "fixed" and name not in passive):
            ET.SubElement(srdf, "passive_joint", {"name": name})
    head = ET.SubElement(srdf, "group", {"name": "head"})
    for name in ("head_1_joint", "head_2_joint"):
        ET.SubElement(head, "joint", {"name": name})
    device_states = yaml.safe_load((HERE / "device_postures.yaml").read_text())
    for group_name, states in device_states.items():
        for state_name, joints in states.items():
            state = ET.SubElement(srdf, "group_state", {"name": state_name, "group": group_name})
            for joint_name, value in joints.items():
                ET.SubElement(state, "joint", {"name": joint_name, "value": str(value)})
    # Public collision matrices include optional hand cameras absent in this model.
    links = {link.get("name") for link in robot.findall("link")}
    for pair in list(srdf.findall("disable_collisions")):
        if pair.get("link1") not in links or pair.get("link2") not in links:
            srdf.remove(pair)
    # Root is the mobile robot base. Navigation owns odom -> base TF.
    # No virtual joint or base trajectory controller is fabricated here.
    srdf_xml = ET.tostring(srdf, encoding="unicode")
    (OUTPUT / "tiago_pro.srdf").write_text(srdf_xml)
    (OUTPUT / "robot.urdf").write_text(robot_xml)

    def load(name):
        return yaml.safe_load((HERE / "upstream/config" / name).read_text())

    kinematics = {key: value for key, value in load("kinematics_kdl.yaml").items()
                  if not key.startswith("arm_teleop_")}
    for value in kinematics.values():
        value["kinematics_solver_timeout"] = 0.1
    limits = load("joint_limits.yaml")
    valid_joints = {joint.get("name") for joint in robot.findall("joint")}
    limits["joint_limits"] = {key: value for key, value in limits["joint_limits"].items()
                              if key in valid_joints}
    # Conservative finite acceleration gives time parameterization a defined bound.
    for value in limits["joint_limits"].values():
        value["has_acceleration_limits"] = True
        value["max_acceleration"] = 0.5
    for joint in leaders:
        limits["joint_limits"][joint] = {"has_velocity_limits": True, "max_velocity": 0.05,
            "has_acceleration_limits": True, "max_acceleration": 0.1}
    for joint in ("head_1_joint", "head_2_joint"):
        limits["joint_limits"][joint] = {"has_velocity_limits": True, "max_velocity": 0.6,
            "has_acceleration_limits": True, "max_acceleration": 0.5}
    ompl = load("ompl_planning.yaml")
    ompl["both_arms"] = {"planner_configs": ["RRTConnectkConfigDefault"],
                          "longest_valid_segment_fraction": 0.005}
    for group_name in device_states:
        ompl[group_name] = {"planner_configs": ["RRTConnectkConfigDefault"],
                            "longest_valid_segment_fraction": 0.005}
    # Use the public settings without an unsupported optional TrajOpt allocator.
    ompl["planner_configs"].pop("TrajOptDefault", None)
    for value in ompl.values():
        if isinstance(value, dict) and isinstance(value.get("planner_configs"), list):
            value["planner_configs"] = [v for v in value["planner_configs"] if v != "TrajOptDefault"]
    controllers = {"controller_names": ["arm_left_controller", "arm_right_controller", "torso_controller",
                                       "head_controller", "gripper_left_controller", "gripper_right_controller"]}
    for side in ("left", "right"):
        controllers[f"arm_{side}_controller"] = {
            "type": "FollowJointTrajectory", "action_ns": "follow_joint_trajectory",
            "default": True, "joints": [f"arm_{side}_{i}_joint" for i in range(1, 8)]}
    controllers["torso_controller"] = {"type": "FollowJointTrajectory",
        "action_ns": "follow_joint_trajectory", "default": True, "joints": ["torso_lift_joint"]}
    controllers["head_controller"] = {"type": "FollowJointTrajectory",
        "action_ns": "follow_joint_trajectory", "default": True, "joints": ["head_1_joint", "head_2_joint"]}
    for side in ("left", "right"):
        leader = f"gripper_{side}_finger_joint"
        controllers[f"gripper_{side}_controller"] = {"type": "GripperCommand", "action_ns": "gripper_cmd",
            "default": True, "joints": [leader], "command_joint": leader, "max_effort": 10.0}
    parameters = {
        "use_sim_time": True, "robot_description": robot_xml,
        "robot_description_semantic": srdf_xml, "robot_description_kinematics": kinematics,
        "robot_description_planning": limits, "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl", "ompl": ompl,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": False, "moveit_simple_controller_manager": controllers,
        "trajectory_execution": {"allowed_execution_duration_scaling": 2.0,
            "allowed_goal_duration_margin": 2.0, "allowed_start_tolerance": 0.02},
        "publish_robot_description": False, "publish_robot_description_semantic": True,
        "publish_planning_scene": True, "publish_geometry_updates": True,
        "publish_state_updates": True, "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "octomap_frame": "odom", "octomap_resolution": 0.025,
        "sensors": ["head_depth"],
        "head_depth": {
            "sensor_plugin": "occupancy_map_monitor/PointCloudOctomapUpdater",
            "point_cloud_topic": "/head_front_camera/depth/color/points",
            "max_range": 3.0, "point_subsample": 2,
            "padding_offset": 0.02, "padding_scale": 1.0,
            "max_update_rate": 2.0,
            "filtered_cloud_topic": "/moveit/filtered_head_depth"},
    }
    path = OUTPUT / "parameters.yaml"
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}, sort_keys=False))
    return path


if __name__ == "__main__":
    print(configure())
