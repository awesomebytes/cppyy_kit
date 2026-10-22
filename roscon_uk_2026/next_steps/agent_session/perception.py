"""Retain one measured RGB/cloud frame and revise native perception parameters."""

from collections import deque
from dataclasses import dataclass
import importlib.util
from pathlib import Path
import time

import cv2
import numpy as np
import pcl_kit
import rosbag2_py
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer

_SOURCE = Path(__file__).resolve().parents[1] / "pick_place" / "perception.py"
_SPEC = importlib.util.spec_from_file_location("pick_place_perception", _SOURCE)
_NATIVE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_NATIVE)


@dataclass
class Snapshot:
    rgb: np.ndarray
    points: np.ndarray
    camera_k: np.ndarray
    rotation: np.ndarray
    translation: np.ndarray
    frame: str = "base_link"
    source: str = ""
    stamp: float = 0.0
    image_stamp: float = 0.0
    image_frame: str = ""


def rotation(quaternion):
    """Return a rotation matrix from ROS XYZW quaternion components."""
    vector, scalar = np.asarray(quaternion[:3]), quaternion[3]
    basis = np.eye(3)
    return (basis + 2 * np.cross(vector, np.cross(vector, basis) + scalar * basis)).T


def snapshot_from_messages(cloud, image, camera, buffer, source="live"):
    """Decode sensor messages and use timestamped TF for geometry association."""
    stamp = Time.from_msg(cloud.header.stamp)
    transform = buffer.lookup_transform("base_link", cloud.header.frame_id, stamp).transform
    optical = buffer.lookup_transform(image.header.frame_id, "base_link", stamp).transform
    q, t = transform.rotation, transform.translation
    body_rotation = rotation((q.x, q.y, q.z, q.w))
    body_translation = np.array((t.x, t.y, t.z))
    xyz = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"))
    xyz = xyz.reshape(-1, 3)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    points = np.ascontiguousarray(xyz @ body_rotation.T + body_translation, dtype=np.float32)
    channels = 3 if image.encoding in ("rgb8", "bgr8") else 4
    if image.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
        raise ValueError("unsupported image encoding: " + image.encoding)
    raw = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step)
    rgb = raw[:, : image.width * channels].reshape(image.height, image.width, channels)[..., :3]
    if image.encoding.startswith("bgr"):
        rgb = rgb[..., ::-1]
    q, t = optical.rotation, optical.translation
    return Snapshot(
        np.ascontiguousarray(rgb),
        points,
        np.array(camera.k).reshape(3, 3),
        rotation((q.x, q.y, q.z, q.w)),
        np.array((t.x, t.y, t.z)),
        source=source,
        stamp=stamp.nanoseconds / 1e9,
        image_stamp=Time.from_msg(image.header.stamp).nanoseconds / 1e9,
        image_frame=image.header.frame_id,
    )


def load_snapshot(path):
    """Load one RGB/cloud pair within 0.3 s from an MCAP rosbag directory."""
    path = Path(path)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()}
    buffer = Buffer()
    cloud = image = camera = None
    pairs = deque(maxlen=10)
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic not in (
            "/tf",
            "/tf_static",
            "/head_front_camera/depth/color/points",
            "/head_front_camera/color/image_raw",
            "/head_front_camera/color/camera_info",
        ):
            continue
        message = deserialize_message(payload, types[topic])
        if topic in ("/tf", "/tf_static"):
            for transform in message.transforms:
                if topic == "/tf_static":
                    buffer.set_transform_static(transform, "recording")
                else:
                    buffer.set_transform(transform, "recording")
        elif topic.endswith("/points"):
            cloud = message
        elif topic.endswith("/image_raw"):
            image = message
        else:
            camera = message
        if cloud is not None and image is not None and camera is not None:
            delta = (
                abs(
                    Time.from_msg(cloud.header.stamp).nanoseconds
                    - Time.from_msg(image.header.stamp).nanoseconds
                )
                / 1e9
            )
            if delta <= 0.3:
                pairs.append((cloud, image, camera))
    if not pairs:
        raise RuntimeError("no RGB/cloud pair within 0.3 s and camera calibration in recording")
    failures = []
    for pair in reversed(pairs):
        try:
            return snapshot_from_messages(*pair, buffer, source=str(path))
        except Exception as error:
            failures.append(str(error))
    raise RuntimeError("no retained sensor pair has timestamped TF: " + failures[-1])


def process(
    snapshot,
    roi=(0.5, -0.5, 0.85, 1.3, 0.5, 1.3),
    color=None,
    min_cluster=12,
    leaf=0.008,
    min_area=40,
    cluster_distance=0.035,
):
    """Reuse PCL table removal/clustering and OpenCV HSV silhouettes.

    Geometry uses base_link metres. Selected bounds describe visible points.
    Loading time includes first native calls. Repeated calls retain loaded C++.
    Timings exclude imports, sensor decoding, TF and snapshot construction.
    """
    started = time.perf_counter()
    native = _NATIVE.load_cloud()
    config = native.CloudConfig()
    config.lower, config.upper = list(roi[:3]), list(roi[3:])
    config.min_cluster, config.leaf = int(min_cluster), float(leaf)
    config.cluster_distance = float(cluster_distance)
    pipeline = _NATIVE.Perception(config)
    cloud = pipeline.cloud(snapshot.points)
    cloud_done = time.perf_counter()
    image = pipeline.image(snapshot.rgb, min_area=min_area)
    image_done = time.perf_counter()
    observations = []
    for candidate in cloud.objects:
        center = np.array(list(candidate.center))
        optical = snapshot.rotation @ center + snapshot.translation
        if optical[2] <= 0:
            continue
        pixel = snapshot.camera_k @ optical
        pixel = pixel[:2] / pixel[2]
        labels = [
            obj
            for obj in image.objects
            if obj.x - 5 <= pixel[0] <= obj.x + obj.width + 5
            and obj.y - 5 <= pixel[1] <= obj.y + obj.height + 5
        ]
        label = (
            min(
                labels,
                key=lambda obj: (
                    (pixel[0] - obj.x - obj.width / 2) ** 2
                    + (pixel[1] - obj.y - obj.height / 2) ** 2
                ),
            )
            if labels
            else None
        )
        observations.append(
            {
                "center": list(candidate.center),
                "size": list(candidate.size),
                "lower": list(candidate.lower),
                "upper": list(candidate.upper),
                "points": int(candidate.points),
                "pixel": pixel.tolist(),
                "color": str(label.color) if label else None,
                "shape": str(label.shape) if label else None,
                "bbox": [label.x, label.y, label.width, label.height] if label else None,
            }
        )
    matches = [obj for obj in observations if obj["color"] == color] if color else []
    selected = max(matches, key=lambda obj: obj["points"]) if matches else None
    filtered = pcl_kit.cloud_to_numpy(cloud.objects_cloud)
    return {
        "source": snapshot.source,
        "frame": snapshot.frame,
        "stamp": snapshot.stamp,
        "image_frame": snapshot.image_frame,
        "image_stamp": snapshot.image_stamp,
        "image_delta_s": abs(snapshot.stamp - snapshot.image_stamp),
        "roi": list(roi),
        "parameters": {
            "color": color,
            "min_cluster": min_cluster,
            "leaf": leaf,
            "min_area": min_area,
            "cluster_distance": cluster_distance,
        },
        "table_found": bool(cloud.table_found),
        "table_height": float(cloud.table_height),
        "input_points": int(cloud.input_points),
        "voxel_points": int(cloud.voxel_points),
        "observations": observations,
        "selected": selected,
        "filtered": filtered,
        "timings": {
            "cloud_ms": (cloud_done - started) * 1000,
            "image_ms": (image_done - cloud_done) * 1000,
            "total_ms": (time.perf_counter() - started) * 1000,
        },
    }


def overlay(snapshot, result):
    """Draw measured image boxes, projected cluster centers and selection."""
    rgb = snapshot.rgb.copy()
    for obj in result["observations"]:
        color = (30, 255, 30) if obj is result["selected"] else (255, 255, 255)
        if obj["bbox"]:
            x, y, width, height = obj["bbox"]
            cv2.rectangle(rgb, (x, y), (x + width, y + height), color, 2)
        u, v = map(round, obj["pixel"])
        cv2.circle(rgb, (u, v), 4, color, -1)
        label = f"{obj['color'] or 'unlabelled'}: {obj['points']} pts"
        cv2.putText(rgb, label, (u - 35, v - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    return rgb


def save_overlay(snapshot, result, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), overlay(snapshot, result)[..., ::-1]):
        raise OSError(f"could not write {path}")
    return str(path)


def serializable(result):
    """Return report fields without the potentially large filtered point array."""
    return {key: value for key, value in result.items() if key != "filtered"}
