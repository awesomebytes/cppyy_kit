"""Compare transformed depth returns with the global costmap.

With --depth-only, disable global static and laser marking and clear the map.
Run this option while navigation is idle. Restart Nav2 to restore its sources.
"""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import JointState, PointCloud2
from tf2_ros import Buffer, TransformListener


def depth_only(node):
    client = AsyncParameterClient(node, '/global_costmap/global_costmap')
    assert client.wait_for_services(timeout_sec=5)
    future = client.set_parameters([
        Parameter('obstacle_layer.front.marking', value=False),
        Parameter('obstacle_layer.rear.marking', value=False),
        Parameter('static_layer.enabled', value=False),
    ])
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    assert future.done() and all(r.successful for r in future.result().results)
    clear = node.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
    assert clear.wait_for_service(timeout_sec=5)
    future = clear.call_async(ClearEntireCostmap.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    assert future.done()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--depth-only', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('depth_costmap_proof')
    buffer = Buffer()
    node.depth_tf_listener = TransformListener(buffer, node)
    latest = {}
    specs = [('cloud', PointCloud2, '/head_front_camera/depth/color/points'),
             ('joints', JointState, '/joint_states'),
             ('map', OccupancyGrid, '/global_costmap/costmap')]
    node.depth_subscriptions = [node.create_subscription(kind, topic, lambda msg, key=key: latest.update({key: msg}),
                                    qos_profile_sensor_data if key != 'map' else
                                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            for key, kind, topic in specs]
    try:
        if args.depth_only:
            depth_only(node)
        end = time.monotonic() + 10
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        cloud, costmap, joints = (latest[k] for k in ('cloud', 'map', 'joints'))
        transform = buffer.lookup_transform(costmap.header.frame_id, cloud.header.frame_id, Time())
        base = buffer.lookup_transform('base_footprint', cloud.header.frame_id, Time())
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])
        # Native Harmonic cloud has float32 XYZ at offsets 0, 4, and 8.
        assert [(f.name, f.offset) for f in cloud.fields[:3]] == [('x', 0), ('y', 4), ('z', 8)]
        points = np.ndarray((cloud.height, cloud.width, cloud.point_step//4),
                            dtype='<f4', buffer=cloud.data)[:, :, :3].reshape(-1, 3)[::20]
        points = points[np.isfinite(points).all(axis=1)]
        t = transform.transform.translation
        world = points @ rotation.T + np.array([t.x, t.y, t.z])
        tall = world[(world[:, 2] > .5) & (world[:, 2] < 1.5)]
        ix = ((tall[:, 0]-costmap.info.origin.position.x)/costmap.info.resolution).astype(int)
        iy = ((tall[:, 1]-costmap.info.origin.position.y)/costmap.info.resolution).astype(int)
        inside = (ix >= 0) & (ix < costmap.info.width) & (iy >= 0) & (iy < costmap.info.height)
        costs = np.asarray(costmap.data).reshape(costmap.info.height, costmap.info.width)
        values = costs[iy[inside], ix[inside]]
        b = base.transform
        report = {
            'cloud_frame': cloud.header.frame_id, 'cloud_size': [cloud.width, cloud.height],
            'head_joints': {k: float(p) for k, p in zip(joints.name, joints.position) if k.startswith('head_')},
            'camera_base_transform': {
                'translation': [b.translation.x, b.translation.y, b.translation.z],
                'rotation_xyzw': [b.rotation.x, b.rotation.y, b.rotation.z, b.rotation.w]},
            'world_height_range_m': [float(world[:, 2].min()), float(world[:, 2].max())],
            'sampled_tall_points': len(tall), 'tall_points_in_global_costmap': int(len(values)),
            'tall_points_at_lethal_cost': int((values == 100).sum()),
            'depth_only_marking': args.depth_only,
            'note': ('Global static and front/rear laser marking disabled; global costmap cleared before sampling.'
                     if args.depth_only else 'Overlapping lethal cells may also come from laser or static marking.'),
        }
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
