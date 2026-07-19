"""rclcpp_kit -- the kit for rclcpp (ROS 2 core), driven from Python via cppyy.

This is the rclcpp core **capability layer**, carved out of the
[rclcppyy](https://github.com/awesomebytes/rclcppyy) product in M1b. It holds the
pieces every ROS-touching kit (and the rclcppyy product itself) needs:

  * **bringup** -- ``bringup_rclcpp()`` JITs ``rclcpp/rclcpp.hpp`` and loads the
    core libraries; ``add_ros2_include_paths()`` puts every ament package's headers
    on cppyy's include path; plus the rclpy-style ``rclcpp.Node`` adapters
    (create_publisher / create_subscription / create_timer / destroy_node);
  * **message machinery** -- C++ message type resolution and the shared recursive
    ``convert_python_msg_to_cpp`` (rclpy message -> the equivalent C++ message);
  * **serialization** -- CDR serialize/deserialize of C++ messages, byte-compatible
    with ``rclpy.serialization`` (``rclcpp_kit.serialization``);
  * **rosbag2** -- the C++ ``rosbag2_cpp`` reader/writer (``rclcpp_kit.rosbag2_cpp``)
    and a ``rosbag2_py``-compatible shim (``rclcpp_kit.rosbag2_py_compat``);
  * **tf** -- the tf2 C++ transform stack (``rclcpp_kit.tf``): a
    ``tf2_ros::TransformListener`` ingesting ``/tf`` wholly in C++ on its own thread.
  * **managed native entities** -- C++ callbacks, services, and typed clients whose
    futures and teardown remain owned by a ``NativeSession``.

It builds on the ROS-free ``cppyy_kit`` base (load_libraries / keep_alive /
register_teardown / pretty_cpp_error). The surface mirrors the names the rclcppyy
product exposed, so rclcppyy can slim to thin re-export shims over this package (M3)
and stay a drop-in rclpy accelerator.

Usage::

    import rclcpp_kit
    rclcpp = rclcpp_kit.bringup_rclcpp()          # rclcpp up under cppyy
    from rclcpp_kit import tf, serialization, rosbag2_cpp
"""
from rclcpp_kit.bringup_rclcpp import (
    bringup_rclcpp,
    shutdown_rclcpp,
    add_ros2_include_paths,
    add_ros2_include_path,
    get_ros2_include_path,
    get_ros2_lib_path,
    ensure_ros_libraries_loaded,
    load_ros_library,
    convert_python_msg_to_cpp,
)
from rclcpp_kit import serialization
from rclcpp_kit import rosbag2_cpp
from rclcpp_kit import rosbag2_py_compat
from rclcpp_kit import tf
from rclcpp_kit import subscription_cache
from rclcpp_kit import borrowed_publish
from rclcpp_kit import borrowed_subscription
from rclcpp_kit import message_facade
from rclcpp_kit import direct_entities
from rclcpp_kit.native import (
    NativeCapabilities,
    NativeExecutorThread,
    NativeSession,
    native,
    publisher_capabilities,
)
from rclcpp_kit.native_pipeline import (
    FusedPipeline,
    NativeCallback,
    NativeStats,
    create_fused_pipeline,
    create_native_callback,
)
from rclcpp_kit.type_adapter import (
    AdapterCapabilities,
    TypeAdapter,
    get_type_adapter,
    register_type_adapter,
    type_adapter_capabilities,
)
from rclcpp_kit.native_service import (
    NativeService,
    NativeServiceStats,
    create_native_service,
)
from rclcpp_kit.python_service import (
    PythonService,
    PythonServiceStats,
    create_python_service,
)
from rclcpp_kit.native_client import (
    NativeClient,
    NativeClientStats,
    create_native_client,
)
from rclcpp_kit.native_lifecycle import (
    NativeLifecycleNode,
    create_native_lifecycle_node,
)
from rclcpp_kit.native_action import (
    NativeActionClient,
    NativeActionClientStats,
    NativeActionResult,
    create_native_action_client,
)
from rclcpp_kit.native_component import (
    NativeComponentManager,
    create_native_component_manager,
)

__all__ = [
    "bringup_rclcpp",
    "shutdown_rclcpp",
    "add_ros2_include_paths",
    "add_ros2_include_path",
    "get_ros2_include_path",
    "get_ros2_lib_path",
    "ensure_ros_libraries_loaded",
    "load_ros_library",
    "convert_python_msg_to_cpp",
    "serialization",
    "rosbag2_cpp",
    "rosbag2_py_compat",
    "tf",
    "subscription_cache",
    "borrowed_publish",
    "borrowed_subscription",
    "message_facade",
    "direct_entities",
    "NativeCapabilities",
    "NativeExecutorThread",
    "NativeSession",
    "native",
    "publisher_capabilities",
    "FusedPipeline",
    "NativeCallback",
    "NativeStats",
    "create_fused_pipeline",
    "create_native_callback",
    "AdapterCapabilities",
    "TypeAdapter",
    "get_type_adapter",
    "register_type_adapter",
    "type_adapter_capabilities",
    "NativeService",
    "NativeServiceStats",
    "create_native_service",
    "PythonService",
    "PythonServiceStats",
    "create_python_service",
    "NativeClient",
    "NativeClientStats",
    "create_native_client",
    "NativeLifecycleNode",
    "create_native_lifecycle_node",
    "NativeActionClient",
    "NativeActionClientStats",
    "NativeActionResult",
    "create_native_action_client",
    "NativeComponentManager",
    "create_native_component_manager",
]
