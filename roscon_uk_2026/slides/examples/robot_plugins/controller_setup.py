"""Build the adapter before Gazebo starts; write its arm configuration."""
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from control_kit.python_control import install_python_controller_plugin


def prepare_admittance(repo, urdf_path, overlay):
    repo = Path(repo).resolve()
    prefix = Path(install_python_controller_plugin(overlay))
    os.environ["AMENT_PREFIX_PATH"] = (
        str(prefix) + ":" + os.environ.get("AMENT_PREFIX_PATH", "")
    )
    joints = [f"arm_left_{i}_joint" for i in range(1, 8)]
    model = ET.parse(urdf_path).getroot()
    limits = {j.get("name"): j.find("limit") for j in model.findall("joint")}
    options = {
        "urdf_path": str(Path(urdf_path).resolve()),
        "base_link": "torso_lift_link",
        "tip_link": "gripper_left_grasping_link",
        "lower_limits": [float(limits[j].get("lower")) for j in joints],
        "upper_limits": [float(limits[j].get("upper")) for j in joints],
    }
    parameters = {
        "type": "control_kit_python/PythonController",
        "joints": joints,
        "python_module": "python_admittance_controller",
        "python_class": "CartesianAdmittance",
        "python_path": str(repo / "control_kit/demos"),
        "python_options": json.dumps(options),
    }
    output = prefix / "admittance.yaml"
    output.write_text(json.dumps({
        "cppyy_admittance_arm_left": {"ros__parameters": parameters}
    }, indent=2) + "\n")
    return output
