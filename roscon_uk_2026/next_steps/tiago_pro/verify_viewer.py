"""Verify actual ROS description resources and depth ground registration."""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

root = Path(__file__).parent
import os
os.environ['AMENT_PREFIX_PATH'] = str(root.resolve() / 'generated/prefix') + ':' + os.environ['AMENT_PREFIX_PATH']
rclpy.init()
node = rclpy.create_node('pro_visual_registration_check')
buffer = Buffer()
listener = TransformListener(buffer, node)
received = {}
subscriptions = []
for topic, kind, qos in [
    ('/robot_description', String, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ('/head_front_camera/depth/color/points', PointCloud2, qos_profile_sensor_data),
    ('/head_front_camera/color/image_raw', Image, qos_profile_sensor_data),
    ('/head_front_camera/depth/image_rect_raw', Image, qos_profile_sensor_data),
]:
    subscriptions.append(node.create_subscription(kind, topic, lambda message, key=topic: received.update({key: message}), qos))
end = time.monotonic() + 5
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
assert len(received) == 4, received.keys()
robot = ET.fromstring(received['/robot_description'].data)
meshes = []
for mesh in robot.findall('.//mesh'):
    uri = mesh.get('filename')
    assert uri.startswith('package://'), uri
    package, relative = uri[len('package://'):].split('/', 1)
    assert (Path(get_package_share_directory(package)) / relative.lstrip("/")).is_file(), uri
    meshes.append(uri)
cloud = received['/head_front_camera/depth/color/points']
assert cloud.header.frame_id == 'head_front_camera_depth_optical_frame'
for topic in ['/head_front_camera/color/image_raw', '/head_front_camera/depth/image_rect_raw']:
    assert received[topic].header.frame_id == cloud.header.frame_id
transform = buffer.lookup_transform('base_footprint', cloud.header.frame_id, rclpy.time.Time())
q = transform.transform.rotation
x, y, z, w = q.x, q.y, q.z, q.w
rotation = np.array([
    [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
    [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
    [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
t = transform.transform.translation
points = np.ndarray((cloud.height, cloud.width, cloud.point_step // 4), dtype='<f4', buffer=cloud.data, strides=(cloud.row_step, cloud.point_step, 4))[..., :3].reshape(-1, 3)
points = points[np.isfinite(points).all(axis=1)]
world = points @ rotation.T + np.array([t.x, t.y, t.z])
ground = world[(world[:, 0] > 0.8) & (np.abs(world[:, 2]) < 0.04)]
assert len(ground) > 1000, f'Ground registration failed: {len(ground)}'
result = {
    'mesh_resources_checked': len(meshes),
    'camera_frame': cloud.header.frame_id,
    'optical_forward_axis_in_base': rotation[:, 2].tolist(),
    'ground_points_within_4cm': len(ground),
    'ground_median_z_m': float(np.median(ground[:, 2])),
    'passed': True,
}
(root / 'artifacts/viewer_validation.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
node.destroy_node()
rclpy.shutdown()
