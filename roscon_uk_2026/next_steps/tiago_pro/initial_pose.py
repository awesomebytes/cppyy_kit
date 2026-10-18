"""Configure native GazeboSystem initial joint state without startup motion."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parent


def configure(robot):
    ready = yaml.safe_load((ROOT / "moveit/postures.yaml").read_text())["ready"]
    positions = {"torso_lift_joint": float(ready["torso_lift_joint"])}
    for side in ("left", "right"):
        positions.update({f"arm_{side}_{index}_joint": float(value)
                          for index, value in enumerate(ready[f"arm_{side}"], 1)})
    devices = yaml.safe_load((ROOT / "moveit/device_postures.yaml").read_text())
    positions.update(devices["head"]["center"])
    # Native 0.4.10 physically initializes joints with command interfaces only.
    # Zero leaders and zero state-only followers are a coherent closed mechanism.
    for joint in robot.findall("joint"):
        if joint.get("name", "").startswith("gripper_") and joint.get("type") != "fixed":
            positions[joint.get("name")] = 0.0
    hardware = {joint.get("name"): joint for joint in robot.findall("ros2_control/joint")}
    for name, value in positions.items():
        joint = hardware.get(name)
        if joint is None:
            raise RuntimeError(f"Missing native joint state interface: {name}")
        interface = joint.find("state_interface[@name='position']")
        if interface is None:
            raise RuntimeError(f"Missing position state interface: {name}")
        initial = interface.find("param[@name='initial_value']")
        if initial is None:
            initial = ET.SubElement(interface, "param", {"name": "initial_value"})
        initial.text = str(value)
    return positions
