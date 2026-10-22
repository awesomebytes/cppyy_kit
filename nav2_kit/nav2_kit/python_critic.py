"""Export a DWB critic that invokes a Python trajectory policy.

Python runs inside the external Nav2 controller_server process. The bridge takes the
GIL and allocates Python objects for each candidate. It has no hard real-time guarantee.
"""

import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path
import sys
import sysconfig

import cppyy_kit


def _build_options():
    from rclcpp_kit.bringup_rclcpp import ros2_include_paths

    directory = Path(__file__).parent
    source = directory / "python_critic.cpp"
    header = directory / "python_critic.hpp"
    prefix = Path(os.environ["CONDA_PREFIX"])
    python_library = Path(sysconfig.get_config_var("LIBDIR")) / (
        "libpython" + sysconfig.get_config_var("LDVERSION") + ".so"
    )
    if not python_library.is_file():
        raise cppyy_kit.CppyyKitError(f"Python shared library is missing: {python_library}")
    digest = hashlib.sha256(header.read_bytes() + source.read_bytes()).hexdigest()
    return {
        "code": (
            f"// {digest}\n"
            f"#define NAV2_KIT_PYTHON_HOME {json.dumps(sys.prefix)}\n"
            f"#define NAV2_KIT_PYTHON_LIBRARY {json.dumps(str(python_library))}\n"
            f'#include "{source}"'
        ),
        "name": "nav2_kit_python_critic",
        "decls": f'#include "{header}"',
        "include_paths": ros2_include_paths() + [
            str(directory), str(prefix / "include"), sysconfig.get_path("include")
        ],
        "library_paths": [str(prefix / "lib"), str(python_library.parent)],
        "libraries": [
            "dwb_core", "rclcpp", "rclcpp_lifecycle", "rcutils",
            "class_loader",
            "python" + sysconfig.get_config_var("LDVERSION"), "dl",
        ],
        "link_args": [
            "-Wl,-rpath," + str(prefix / "lib"),
            "-Wl,-rpath," + str(python_library.parent), "-pthread",
        ],
    }


def install_python_critic_plugin(prefix):
    """Build a DWB TrajectoryCritic overlay for this Python installation.

    Add the returned prefix to AMENT_PREFIX_PATH before starting Nav2. Use class
    nav2_kit_python::PythonCritic and parameters python_module, python_class,
    absolute python_path, python_options (JSON), and lookahead_distance (metres).
    Python implements configure(options), prepare(yaw, path_heading, goal_distance),
    and score(vx, vy, wz, final_heading). score returns None to reject a candidate
    or a finite nonnegative float. Angles are radians, velocities are m/s and rad/s.
    One prepare call runs per DWB cycle; one score call runs per candidate reaching
    this critic. The bridge acquires the GIL and has no real-time guarantee.
    """
    library = cppyy_kit.prebuild(**_build_options())
    if not library:
        raise cppyy_kit.CppyyKitError("plugin export requires the compiled cache")
    prefix = Path(prefix).resolve()
    destination = prefix / "lib/libnav2_kit_python_critic.so"
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as staged:
        staged_path = Path(staged.name)
    try:
        shutil.copy2(library, staged_path)
        os.replace(staged_path, destination)
    finally:
        staged_path.unlink(missing_ok=True)
    package = "nav2_kit_python"
    share = prefix / "share" / package
    share.mkdir(parents=True, exist_ok=True)
    (share / "package.xml").write_text(
        '<package format="3"><name>nav2_kit_python</name><version>0.1.0</version>'
        '<description>Python cppyy_kit DWB critic bridge</description>'
        '<maintainer email="maintainers@example.com">cppyy_kit maintainers</maintainer>'
        '<license>BSD-3-Clause</license>'
        '<export><dwb_core plugin="${prefix}/critics.xml"/></export>'
        '</package>\n'
    )
    (share / "critics.xml").write_text(
        '<library path="nav2_kit_python_critic">\n'
        '  <class name="nav2_kit_python::PythonCritic" '
        'type="nav2_kit_python::PythonCritic" '
        'base_class_type="dwb_core::TrajectoryCritic"/>\n'
        '</library>\n'
    )
    index = prefix / "share/ament_index/resource_index"
    for kind, content in (
        ("packages", ""),
        ("dwb_core__pluginlib__plugin", "share/" + package + "/critics.xml\n"),
    ):
        resource = index / kind / package
        resource.parent.mkdir(parents=True, exist_ok=True)
        resource.write_text(content)
    return str(prefix)
