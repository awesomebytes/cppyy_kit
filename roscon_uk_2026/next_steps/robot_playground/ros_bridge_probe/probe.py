"""Observe and command the narrowly patched upstream Stretch 4 ROS simulator driver."""
from collections import Counter
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("ROS_DOMAIN_ID", "89")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
SOURCE_PATHS = [ROOT / "source/stretch_simulation", ROOT / "source/hello_helpers/src",
                ROOT / "ros2_numpy_source"]
sys.path[:0] = [str(p) for p in SOURCE_PATHS]
os.environ["PYTHONPATH"] = os.pathsep.join([*(str(p) for p in SOURCE_PATHS),
                                            os.environ.get("PYTHONPATH", "")])


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, JointState, LaserScan, Image, PointCloud2
    from rclpy.qos import qos_profile_sensor_data

    cameras = "--cameras" in sys.argv
    position_only = "--position-only" in sys.argv
    params = {
        "use_cameras": cameras, "use_mujoco_viewer": False, "use_robocasa": False,
        "controller_calibration_file": "", "scene_xml": "", "scene_name": "",
        "mode": "position" if position_only else "navigation", "broadcast_odom_tf": True,
        "fail_out_of_range_goal": False,
        "fail_if_motor_initial_point_is_not_trajectory_first_point": False,
        "action_server_rate": 30.0, "joint_state_rate": 15.0,
        "timeout": 0.5, "default_goal_timeout_s": 10.0,
    }
    command = [sys.executable, "-m", "stretch_mujoco_driver.stretch_mujoco_driver", "--ros-args"]
    for name, value in params.items():
        value = str(value).lower() if isinstance(value, bool) else value
        command += ["-p", f"{name}:={value}" if value != "" else f'{name}:=""']
    artifacts = ROOT / "artifacts"
    artifacts.mkdir(exist_ok=True)
    suffix = "position" if position_only else ("cameras" if cameras else "base")
    log_path = artifacts / f"driver_{suffix}.log"
    rclpy.init()
    observer = rclpy.create_node("stretch4_probe_observer")
    counts = Counter()
    messages = {}
    first = {}
    subscriptions = []
    topics = {"/odom": Odometry, "/joint_states": JointState, "/imu_mobile_base": Imu,
              "/clock": Clock, "/scan_filtered": LaserScan}
    if cameras:
        topics.update({"/camera_head/center/image_raw": Image,
                       "/gripper_camera/depth/color/points": PointCloud2})
    def receive(topic, message):
        counts[topic] += 1
        messages[topic] = message
        first.setdefault(topic, message)
    for topic, message_type in topics.items():
        subscriptions.append(observer.create_subscription(
            message_type, topic, lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data))
    publisher = observer.create_publisher(Twist, "/cmd_vel", 1)
    process = None
    report = None
    shutdown = {"sigint_sent": False, "forced_sigterm": False}
    # Preserve the shared simulator guard's lock in the driver process too.
    # Direct Python invocation may have no inherited descriptor.
    try:
        os.fstat(9)
        inherited_fds = (9,)
    except OSError:
        inherited_fds = ()
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, pass_fds=inherited_fds)
            deadline = time.monotonic() + 100
            while "/odom" not in messages and time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f"Upstream driver exited ({process.returncode}); see {log_path}")
                rclpy.spin_once(observer, timeout_sec=0.2)
            assert "/odom" in messages, f"No odometry; see {log_path}"
            start = messages["/odom"].pose.pose.position
            initial_xy = (start.x, start.y)
            deadline = time.monotonic() + (15 if cameras else 5)
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f"Upstream driver exited ({process.returncode}); see {log_path}")
                command_message = Twist()
                command_message.linear.x = 0.06
                publisher.publish(command_message)
                rclpy.spin_once(observer, timeout_sec=0.05)
            publisher.publish(Twist())
            for _ in range(10):
                rclpy.spin_once(observer, timeout_sec=0.05)
            assert process.poll() is None, f"Driver exited during observation; see {log_path}"
            final = messages["/odom"].pose.pose.position
            distance = math.hypot(final.x - initial_xy[0], final.y - initial_xy[1])
            if not position_only:
                assert distance > 0.01, f"No base motion from ROS commands: {distance}"
            for topic in topics:
                assert counts[topic] > 0, f"No messages: {topic}; see {log_path}"
            assert len(messages["/joint_states"].name) > 5
            clock = messages["/clock"].clock
            first_clock = first["/clock"].clock
            clock_advance = (clock.sec - first_clock.sec) + (clock.nanosec - first_clock.nanosec) * 1e-9
            assert clock_advance > 0, "ROS simulation clock did not advance"
            report = {"upstream_driver": "hello-robot/stretch4_ros2@" + subprocess.check_output(
                ["git", "-C", str(ROOT / "source"), "rev-parse", "HEAD"], text=True).strip(),
                "driver_is_patched": True, "patch": "upstream_navigation.patch",
                "patch_sha256": hashlib.sha256((ROOT / "upstream_navigation.patch").read_bytes()).hexdigest(),
                "cameras": cameras, "message_counts": dict(counts),
                "joint_count": len(messages["/joint_states"].name),
                "base_distance_m": distance, "command_topic": "/cmd_vel",
                "clock_advance_s": clock_advance,
                "ros_domain_id": os.environ["ROS_DOMAIN_ID"], "rmw": os.environ["RMW_IMPLEMENTATION"]}
            if cameras:
                image = messages["/camera_head/center/image_raw"]
                cloud = messages["/gripper_camera/depth/color/points"]
                assert image.width > 0 and image.height > 0 and len(image.data) > 0
                assert cloud.width * cloud.height > 0 and len(cloud.data) > 0
                report["head_image"] = {"width": image.width, "height": image.height,
                                        "encoding": image.encoding, "bytes": len(image.data)}
                report["gripper_pointcloud"] = {"points": cloud.width * cloud.height,
                                                "bytes": len(cloud.data)}
    finally:
        if process is not None and process.poll() is None:
            shutdown["sigint_sent"] = True
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                shutdown["forced_sigterm"] = True
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        shutdown["driver_returncode"] = process.returncode if process else None
        observer.destroy_node()
        rclpy.shutdown()
    if report is not None:
        report["shutdown"] = shutdown
        (artifacts / f"result_{suffix}.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
