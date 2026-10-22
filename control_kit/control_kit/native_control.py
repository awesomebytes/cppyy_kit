"""Compile and load a native ros2_control position servo through cppyy_kit."""

import hashlib
import os
import shutil
import tempfile
from pathlib import Path

import cppyy_kit
from cppyy_kit import cppyy

_BUILD = None


def _build_options(plugin=False):
    from rclcpp_kit.bringup_rclcpp import ros2_include_paths

    directory = Path(__file__).parent
    header = directory / "native_controller.hpp"
    source = directory / "native_controller.cpp"
    digest = hashlib.sha256(header.read_bytes() + source.read_bytes()).hexdigest()
    prefix = Path(os.environ["CONDA_PREFIX"])
    return {
        "code": (
            f"// {digest}\n"
            + ("#define CONTROL_KIT_PLUGINLIB\n" if plugin else "")
            + f'#include "{source}"'
        ),
        "decls": f'#include "{header}"',
        "name": "control_kit_native_controller",
        "include_paths": ros2_include_paths()
        + [str(directory), str(prefix / "include")],
        "library_paths": [str(prefix / "lib")],
        "libraries": [
            "controller_interface",
            "rclcpp",
            "rclcpp_lifecycle",
            "rcutils",
            "class_loader",
            "std_msgs__rosidl_typesupport_cpp",
        ],
        "link_args": ["-Wl,-rpath," + str(prefix / "lib"), "-pthread"],
    }


def compile_native_controller():
    """Compile and load the cached library, and return its build metadata."""
    global _BUILD
    if _BUILD is None:
        from control_kit import bringup_control, load_message_support

        bringup_control()
        load_message_support()
        _BUILD = cppyy_kit.cppdef_cached(**_build_options())
    return dict(_BUILD)


def make_native_position_controller():
    """Create an owning C++ shared_ptr. Add it to a rig, then set parameters.

    Parameters: joints (ordered names), gain (s^-1), speed_limits (joint units/s),
    lower_limits and upper_limits (joint units). Empty limits use no position bounds.
    Parameters are read on configure. Activation holds measured positions and
    clears earlier goals. Updates require 0 < period <= 0.1 s. Invalid state or
    period returns ERROR before writing a command. Set a target after activation using
    controller.set_target([...]) or a Float64MultiArray on ~/commands.
    """
    compile_native_controller()
    return cppyy.gbl.control_kit_native.make_position_servo()


def install_native_controller_plugin(prefix):
    """Write a plugin overlay and return its prefix for AMENT_PREFIX_PATH.

    Call before starting an external controller_manager or Gazebo process.
    The overlay contains a copy of the cached library and survives cache cleanup.
    """
    library = cppyy_kit.prebuild(**_build_options(plugin=True))
    if not library:
        raise cppyy_kit.CppyyKitError("plugin export requires the compiled cache")
    prefix = Path(prefix).resolve()
    destination = prefix / "lib/libcontrol_kit_native_controller.so"
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Preserve the inode mapped by an already running controller manager.
    with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as staged:
        staged_path = Path(staged.name)
    try:
        shutil.copy2(library, staged_path)
        os.replace(staged_path, destination)
    finally:
        staged_path.unlink(missing_ok=True)
    package = "control_kit_native"
    share = prefix / "share" / package
    share.mkdir(parents=True, exist_ok=True)
    (share / "package.xml").write_text(
        '<package format="3"><name>control_kit_native</name><version>0.1.0</version>'
        "<description>Native cppyy_kit position servo</description>"
        '<maintainer email="maintainers@example.com">cppyy_kit maintainers</maintainer>'
        "<license>BSD-3-Clause</license>"
        '<export><controller_interface plugin="${prefix}/controllers.xml"/></export>'
        "</package>\n"
    )
    xml = share / "controllers.xml"
    xml.write_text(
        '<library path="control_kit_native_controller">\n'
        '  <class name="control_kit_native/PositionServo" '
        'type="control_kit_native::PositionServo" '
        'base_class_type="controller_interface::ControllerInterface"/>\n'
        "</library>\n"
    )
    index = prefix / "share/ament_index/resource_index"
    for kind, content in (
        ("packages", ""),
        (
            "controller_interface__pluginlib__plugin",
            "share/" + package + "/controllers.xml\n",
        ),
    ):
        resource = index / kind / package
        resource.parent.mkdir(parents=True, exist_ok=True)
        resource.write_text(content)
    return str(prefix)
