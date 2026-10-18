"""Check ROS sensor payloads, simulation time, and commanded base/arm motion."""
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, JointState, LaserScan, PointCloud2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rclpy.init()
node = rclpy.create_node("tiago_pro_smoke")
counts, latest = {}, {}
subscriptions = []
for topic, message_type in [
    ("/clock", Clock),
    ("/wrist_left_ft_broadcaster/wrench", WrenchStamped),
    ("/wrist_right_ft_broadcaster/wrench", WrenchStamped),
    ("/joint_states", JointState),
    ("/mobile_base_controller/odom", Odometry),
    ("/base_imu", Imu),
    ("/head_front_camera/color/image_raw", Image),
    ("/head_front_camera/depth/image_rect_raw", Image),
    ("/head_front_camera/depth/color/points", PointCloud2),
    ("/scan_front_raw", LaserScan),
    ("/scan_rear_raw", LaserScan),
]:
    def receive(message, key=topic):
        counts[key] = counts.get(key, 0) + 1
        latest[key] = message

    subscriptions.append(node.create_subscription(
        message_type, topic, receive, qos_profile_sensor_data))

arms = {side: node.create_publisher(
    JointTrajectory, f"/arm_{side}_controller/joint_trajectory", 10)
    for side in ["left", "right"]}
velocity = node.create_publisher(
    Twist, "/mobile_base_controller/cmd_vel_unstamped", 10)


def spin(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


def positions():
    message = latest["/joint_states"]
    return dict(zip(message.name, message.position))


try:
    spin(8)
    assert len(counts) == len(subscriptions), f"Missing inputs: {counts}"
    start = positions()
    initial_x = latest["/mobile_base_controller/odom"].pose.pose.position.x
    initial_clock = latest["/clock"].clock
    initial_time = initial_clock.sec + initial_clock.nanosec * 1e-9
    for side, publisher in arms.items():
        message = JointTrajectory()
        message.joint_names = [f"arm_{side}_{index}_joint" for index in range(1, 8)]
        point = JointTrajectoryPoint()
        point.positions = [start[joint] for joint in message.joint_names]
        point.positions[5] += 0.15
        point.time_from_start.sec = 3
        message.points = [point]
        publisher.publish(message)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        message = Twist()
        message.linear.x = 0.1
        velocity.publish(message)
        rclpy.spin_once(node, timeout_sec=0.05)
    velocity.publish(Twist())
    spin(2)
    end = positions()
    clock = latest["/clock"].clock
    result = {
        "messages": counts,
        "base_dx_m": latest["/mobile_base_controller/odom"].pose.pose.position.x - initial_x,
        "arm_delta_rad": {
            side: end[f"arm_{side}_6_joint"] - start[f"arm_{side}_6_joint"]
            for side in arms},
        "sim_elapsed_s": clock.sec + clock.nanosec * 1e-9 - initial_time,
        "images": {},
    }
    for topic in ["/head_front_camera/color/image_raw", "/head_front_camera/depth/image_rect_raw"]:
        message = latest[topic]
        assert message.width > 0 and message.height > 0 and len(message.data) > 0
        result["images"][topic] = {
            "width": message.width, "height": message.height,
            "encoding": message.encoding, "bytes": len(message.data)}
    cloud = latest["/head_front_camera/depth/color/points"]
    assert cloud.width * cloud.height > 0 and len(cloud.data) > 0
    result["point_cloud_points"] = cloud.width * cloud.height
    for topic in ["/scan_front_raw", "/scan_rear_raw"]:
        assert len(latest[topic].ranges) > 0
    imu = latest["/base_imu"]
    assert abs(imu.linear_acceleration.z) > 1
    assert result["sim_elapsed_s"] > 1
    assert result["base_dx_m"] > 0.05
    assert all(abs(delta - 0.15) < 0.05 for delta in result["arm_delta_rad"].values())
    result["wrist_force_torque_presence"] = {}
    for side in ["left", "right"]:
        message = latest[f"/wrist_{side}_ft_broadcaster/wrench"]
        axes = [getattr(part, axis) for part in [message.wrench.force, message.wrench.torque]
                for axis in ["x", "y", "z"]]
        assert all(math.isfinite(value) for value in axes)
        assert message.header.frame_id == f"wrist_{side}_ft_sensor_link"
        result["wrist_force_torque_presence"][side] = {
            "finite_six_axes": True, "frame_id": message.header.frame_id,
            "force_N_torque_Nm": axes}
    result["passed"] = True
    output = Path(__file__).parent / "artifacts/smoke.json"
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
finally:
    velocity.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()
