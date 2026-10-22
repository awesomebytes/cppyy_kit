"""Capture ROS sensors and show revised perception output in RViz."""

import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from roscon_uk_2026.next_steps.agent_session.perception import (
    load_snapshot,
    overlay,
    process,
    serializable,
    snapshot_from_messages,
)


class Live(Node):
    """Own sensor subscriptions, TF and latched RViz output publishers."""

    def __init__(self):
        super().__init__("agent_perception_session")
        self.messages = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subscriptions_owned = [
            self.create_subscription(
                message_type,
                topic,
                lambda message, key=key: self.messages.update({key: message}),
                qos_profile_sensor_data,
            )
            for key, message_type, topic in (
                ("cloud", PointCloud2, "/head_front_camera/depth/color/points"),
                ("image", Image, "/head_front_camera/color/image_raw"),
                ("camera", CameraInfo, "/head_front_camera/color/camera_info"),
            )
        ]
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_publisher = self.create_publisher(
            PointCloud2, "/agent_session/filtered_points", qos
        )
        self.image_publisher = self.create_publisher(Image, "/agent_session/overlay", qos)
        self.marker_publisher = self.create_publisher(MarkerArray, "/agent_session/objects", qos)

    def capture(self, timeout=30, min_stamp=None):
        deadline = time.monotonic() + timeout
        reason = "no camera messages"
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.messages) != 3:
                continue
            stamp = self.messages["cloud"].header.stamp
            if min_stamp is not None and stamp.sec + stamp.nanosec / 1e9 <= min_stamp:
                continue
            try:
                result = snapshot_from_messages(**self.messages, buffer=self.buffer)
                if abs(result.stamp - result.image_stamp) <= 0.3:
                    return result
                reason = "RGB/cloud stamps differ by more than 0.3 s"
            except Exception as error:
                reason = str(error)
        raise TimeoutError(reason)

    def publish(self, snapshot, result):
        """Publish filtered points, selected visible bounds and RGB overlay."""
        header = Header(
            frame_id=snapshot.frame, stamp=Time(nanoseconds=round(snapshot.stamp * 1e9)).to_msg()
        )
        cloud = point_cloud2.create_cloud_xyz32(header, np.asarray(result["filtered"]))
        self.cloud_publisher.publish(cloud)
        image = Image()
        rgb = overlay(snapshot, result)
        image.header = Header(
            frame_id=snapshot.image_frame,
            stamp=Time(nanoseconds=round(snapshot.image_stamp * 1e9)).to_msg(),
        )
        image.height, image.width = rgb.shape[:2]
        image.encoding, image.step = "rgb8", image.width * 3
        image.data = rgb.tobytes()
        self.image_publisher.publish(image)
        markers = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        markers.markers.append(clear)
        for index, obj in enumerate(result["observations"]):
            marker = Marker(
                header=header, ns="visible_bounds", id=index, type=Marker.CUBE, action=Marker.ADD
            )
            marker.pose.orientation.w = 1.0
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = obj["center"]
            marker.scale.x, marker.scale.y, marker.scale.z = [max(0.005, n) for n in obj["size"]]
            marker.color.a = 0.65 if obj is result["selected"] else 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0 if obj is result["selected"] else 1.0
            marker.color.b = 0.0
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)
        return {
            "filtered_points": len(result["filtered"]),
            "markers": len(markers.markers) - 1,
            "selected_marker": result["selected"] is not None,
            "topics": [
                "/agent_session/filtered_points",
                "/agent_session/objects",
                "/agent_session/overlay",
            ],
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--duration", type=float, default=30)
    parser.add_argument("--roi", type=float, nargs=6, default=(0.5, -0.5, 0.85, 1.3, 0.5, 1.3))
    parser.add_argument("--color", default="red")
    parser.add_argument("--leaf", type=float, default=0.008)
    parser.add_argument("--min-cluster", type=int, default=12)
    parser.add_argument("--rate", type=float, default=2.0, help="live processing rate in Hz")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error("--rate must be positive")
    rclpy.init()
    live = Live()
    try:
        snapshot = load_snapshot(args.bag) if args.bag else live.capture()
        frames = []
        deadline = None
        while True:
            cycle = time.monotonic()
            result = process(
                snapshot,
                roi=args.roi,
                color=args.color,
                leaf=args.leaf,
                min_cluster=args.min_cluster,
            )
            publication = live.publish(snapshot, result)
            frames.append(
                {
                    "stamp": snapshot.stamp,
                    "timings": result["timings"],
                    "objects": len(result["observations"]),
                    "selected": result["selected"] is not None,
                }
            )
            print(json.dumps(frames[-1]), flush=True)
            if deadline is None:
                deadline = time.monotonic() + args.duration
            if args.bag:
                while time.monotonic() < deadline:
                    rclpy.spin_once(live, timeout_sec=0.1)
                break
            next_frame = min(cycle + 1 / args.rate, deadline)
            while time.monotonic() < next_frame:
                rclpy.spin_once(live, timeout_sec=min(0.1, next_frame - time.monotonic()))
            if time.monotonic() >= deadline:
                break
            snapshot = live.capture(min_stamp=snapshot.stamp)
        report = {
            "result": serializable(result),
            "publication": publication,
            "frames": frames,
            "requested_rate_hz": args.rate,
            "mode": "retained MCAP snapshot" if args.bag else "fresh live frames",
        }
        if args.report:
            args.report.write_text(json.dumps(report, indent=2) + "\n")
    finally:
        live.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
