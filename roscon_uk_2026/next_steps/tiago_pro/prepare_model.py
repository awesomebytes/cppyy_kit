"""Generate an explicit package index for pinned source description packages."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro

ROOT = Path(__file__).resolve().parent
prefix = ROOT / "generated/prefix"
for manifest in ([] if os.environ.get("TIAGO_PRO_INSTALLED_PREFIX") else
                 (ROOT / "vendor").glob("**/package.xml")):
    # This checkout supplies one separately built plugin, not description data.
    # Indexing it here would shadow installed native gazebo_ros executables.
    if "gazebo_ros_pkgs" in manifest.relative_to(ROOT / "vendor").parts:
        continue
    name = ET.parse(manifest).getroot().findtext("name")
    marker = prefix / "share/ament_index/resource_index/packages" / name
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.touch()
    destination = prefix / "share" / name
    if not destination.exists():
        destination.symlink_to(manifest.parent, target_is_directory=True)
os.environ["AMENT_PREFIX_PATH"] = str(prefix) + ":" + os.environ.get("AMENT_PREFIX_PATH", "")
path = (Path(os.environ["TIAGO_PRO_INSTALLED_PREFIX"]) / "share/tiago_pro_description/robots/tiago_pro.urdf.xacro"
        if os.environ.get("TIAGO_PRO_INSTALLED_PREFIX") else
        ROOT / "vendor/tiago_pro_robot/tiago_pro_description/robots/tiago_pro.urdf.xacro")
document = xacro.process_file(str(path), mappings={
    "is_public_sim": "true", "use_sim_time": "true",
    "camera_model": "realsense-d435",
    "ft_sensor_left": "ati", "ft_sensor_right": "ati"})
(ROOT / "generated/tiago_pro.urdf").write_text(document.toprettyxml())
print("Generated TIAGo PRO public URDF", flush=True)
