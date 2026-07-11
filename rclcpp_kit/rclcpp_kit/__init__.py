"""rclcpp_kit — the kit for rclcpp (ROS 2 core). Placeholder until M1b.

Per ARCHITECTURE_V2 §4.2, the rclcpp core capability layer (rclcpp bringup,
C++ message resolution/conversion, serialization, rosbag2_cpp, **tf**,
executor/node helpers, the rclcpp PCH recipe) is carved out of the rclcppyy
product into this package in **M1b**.

Until then this package is intentionally empty. The tf REPORT, demos and tests
staged under ``rclcpp_kit/`` import the rclcppyy product directly
(``rclcppyy.bringup_rclcpp`` / ``rclcppyy.tf``) — the M1b-temporary bridge. See
``rclcpp_kit/README.md`` and the repo README for details.
"""

__all__ = []
