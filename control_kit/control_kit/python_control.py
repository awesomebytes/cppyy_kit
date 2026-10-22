"""Export a ros2_control plugin that invokes a Python position controller.

Python runs inside the external controller_manager process. The bridge takes the
GIL and allocates Python objects each update. It has no hard real-time guarantee.
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
    source = directory / "python_controller.cpp"
    header = directory / "python_controller.hpp"
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
            f"#define CONTROL_KIT_PYTHON_HOME {json.dumps(sys.prefix)}\n"
            f"#define CONTROL_KIT_PYTHON_LIBRARY {json.dumps(str(python_library))}\n"
            f'#include "{source}"'
        ),
        "name": "control_kit_python_controller",
        "decls": f'#include "{header}"',
        "include_paths": ros2_include_paths() + [
            str(directory), str(prefix / "include"), sysconfig.get_path("include")
        ],
        "library_paths": [str(prefix / "lib"), str(python_library.parent)],
        "libraries": [
            "controller_interface", "rclcpp", "rclcpp_lifecycle", "rcutils",
            "class_loader", "std_msgs__rosidl_typesupport_cpp",
            "python" + sysconfig.get_config_var("LDVERSION"), "dl",
        ],
        "link_args": [
            "-Wl,-rpath," + str(prefix / "lib"),
            "-Wl,-rpath," + str(python_library.parent), "-pthread",
        ],
    }


def install_python_controller_plugin(prefix):
    """Compile and export an overlay without loading the plugin into Cling.

    Set AMENT_PREFIX_PATH to include the returned prefix before starting Gazebo.
    Plugin type: control_kit_python/PythonController. Parameters: ordered joints,
    python_module, python_class, absolute python_path, and python_options (JSON).

    The selected class has configure(joints, options), set_target(values), and
    update(measured_positions, dt_seconds). update returns ordered positions.
    Optional activate(measured_positions) and deactivate() implement lifecycle.
    Operations may raise an exception or return False to reject a request.
    update errors or invalid results return ERROR before writing commands.

    The compiled plugin links this Python installation. Keep it available. An
    existing host interpreter with matching major/minor is reused without changing
    its prefix. Python extension dependencies must match the host environment.
    The library persists in the overlay after build-cache cleanup.
    """
    library = cppyy_kit.prebuild(**_build_options())
    if not library:
        raise cppyy_kit.CppyyKitError("plugin export requires the compiled cache")
    prefix = Path(prefix).resolve()
    destination = prefix / "lib/libcontrol_kit_python_controller.so"
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as staged:
        staged_path = Path(staged.name)
    try:
        shutil.copy2(library, staged_path)
        os.replace(staged_path, destination)
    finally:
        staged_path.unlink(missing_ok=True)
    package = "control_kit_python"
    share = prefix / "share" / package
    share.mkdir(parents=True, exist_ok=True)
    (share / "package.xml").write_text(
        '<package format="3"><name>control_kit_python</name><version>0.1.0</version>'
        '<description>Python cppyy_kit controller bridge</description>'
        '<maintainer email="maintainers@example.com">cppyy_kit maintainers</maintainer>'
        '<license>BSD-3-Clause</license>'
        '<export><controller_interface plugin="${prefix}/controllers.xml"/></export>'
        '</package>\n'
    )
    (share / "controllers.xml").write_text(
        '<library path="control_kit_python_controller">\n'
        '  <class name="control_kit_python/PythonController" '
        'type="control_kit_python::PythonController" '
        'base_class_type="controller_interface::ControllerInterface"/>\n'
        '</library>\n'
    )
    index = prefix / "share/ament_index/resource_index"
    for kind, content in (
        ("packages", ""),
        ("controller_interface__pluginlib__plugin", "share/" + package + "/controllers.xml\n"),
    ):
        resource = index / kind / package
        resource.parent.mkdir(parents=True, exist_ok=True)
        resource.write_text(content)
    return str(prefix)
