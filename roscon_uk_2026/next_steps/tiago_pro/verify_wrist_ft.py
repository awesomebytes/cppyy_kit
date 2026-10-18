"""Historical positive-Z recheck protocol for reproducing earlier reports.

Prefer ``pixi run ft-check``: its shared directional probe additionally checks
world-vector error, sign reversal and repeatability on both robot variants.
The old failed accuracy conclusions are superseded by WRIST_FT_INVESTIGATION.md.
"""
import json
import math
from pathlib import Path
import time

from gazebo_msgs.srv import ApplyLinkWrench, LinkRequest
from geometry_msgs.msg import WrenchStamped
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node('pro_wrist_ft_validation')
buffer = Buffer()
listener = TransformListener(buffer, node)
latest = {}
samples = {side: [] for side in ['left', 'right']}
subscriptions = []


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def rotate(vector, rotation):
    q = np.array([rotation.x, rotation.y, rotation.z])
    value = np.array(vector)
    return value + 2 * np.cross(q, np.cross(q, value) + rotation.w * value)


for side in samples:
    def receive(message, side=side):
        latest[side] = message
        transform = buffer.lookup_transform('odom', message.header.frame_id, rclpy.time.Time()) if buffer.can_transform('odom', message.header.frame_id, rclpy.time.Time()) else None
        if transform is not None:
            values = [getattr(part, axis) for part in [message.wrench.force, message.wrench.torque]
                      for axis in ['x', 'y', 'z']]
            assert all(math.isfinite(value) for value in values)
            world_force = rotate(values[:3], transform.transform.rotation)
            samples[side].append((stamp_seconds(message.header.stamp), values, world_force.tolist()))
    subscriptions.append(node.create_subscription(
        WrenchStamped, f'/wrist_{side}_ft_broadcaster/wrench', receive,
        qos_profile_sensor_data))
subscriptions.append(node.create_subscription(
    Clock, '/clock', lambda message: latest.update(clock=message), qos_profile_sensor_data))


def until(target):
    deadline = time.monotonic() + 30
    while 'clock' not in latest or stamp_seconds(latest['clock'].clock) < target:
        assert time.monotonic() < deadline, 'Simulation clock stopped'
        rclpy.spin_once(node, timeout_sec=0.05)


def now():
    return stamp_seconds(latest['clock'].clock)


def summarize(side, start, end):
    rows = [row for row in samples[side] if start <= row[0] <= end]
    assert len(rows) >= 10, f'Insufficient timestamp-filtered samples: {len(rows)}'
    return {
        'sim_window_s': [start, end], 'samples': len(rows),
        'mean_sensor_force_N_torque_Nm': np.mean([row[1] for row in rows], axis=0).tolist(),
        'mean_world_force_N': np.mean([row[2] for row in rows], axis=0).tolist(),
    }


apply = node.create_client(ApplyLinkWrench, '/apply_link_wrench')
clear = node.create_client(LinkRequest, '/clear_link_wrenches')


def call(client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    assert future.done() and future.result() is not None
    return future.result()


report = {'method': 'one wrist at a time; pure world force at link COM; timestamp-filtered SIM windows', 'tests': []}
output = Path(__file__).parent / 'artifacts/wrist_ft_recheck.json'
try:
    assert apply.wait_for_service(timeout_sec=5) and clear.wait_for_service(timeout_sec=5)
    until(5)
    until(now() + 3)
    for force in [5.0, 10.0]:
        for side in ['left', 'right']:
            for name in ['left', 'right']:
                call(clear, LinkRequest.Request(link_name=f'tiago_pro::wrist_{name}_ft_sensor_link'))
            until(now() + 2)
            baseline_start = now() + 0.2
            baseline_end = baseline_start + 1
            until(baseline_end + 0.2)
            baseline = summarize(side, baseline_start, baseline_end)
            start = now() + 0.5
            request = ApplyLinkWrench.Request()
            request.link_name = f'tiago_pro::wrist_{side}_ft_sensor_link'
            request.reference_frame = 'world'
            # Stock world branch directly adds reference_point x force as a
            # couple. Zero point + zero torque is pure SetForce at link COM.
            request.wrench.force.z = force
            request.start_time.sec = int(start)
            request.start_time.nanosec = int((start - int(start)) * 1e9)
            request.duration.sec = 5
            response = call(apply, request)
            assert response.success, response.status_message
            until(start + 3.2)
            loaded = summarize(side, start + 1, start + 3)
            until(start + 7.2)
            released = summarize(side, start + 6, start + 7)
            delta = np.array(loaded['mean_world_force_N']) - baseline['mean_world_force_N']
            recovery = np.linalg.norm(np.array(released['mean_world_force_N']) - baseline['mean_world_force_N'])
            row = {
                'side': side, 'requested_force_world_N': [0, 0, force],
                'requested_torque_world_Nm': [0, 0, 0], 'reference_point': [0, 0, 0],
                'link': request.link_name, 'start_sim_s': start, 'duration_sim_s': 5,
                'baseline': baseline, 'loaded': loaded, 'released': released,
                'world_force_change_N': delta.tolist(), 'force_change_norm_N': float(np.linalg.norm(delta)),
                'release_residual_N': float(recovery),
                'passed': bool(abs(np.linalg.norm(delta) - force) < 0.5 and recovery < 0.5),
            }
            report['tests'].append(row)
            output.write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(row), flush=True)
        if not all(row['passed'] for row in report['tests']):
            break
    report['passed'] = all(row['passed'] for row in report['tests'])
    output.write_text(json.dumps(report, indent=2) + '\n')
    assert report['passed'], 'Strict known-force check failed; see artifacts/wrist_ft_recheck.json'
finally:
    for side in ['left', 'right']:
        if clear.service_is_ready():
            call(clear, LinkRequest.Request(link_name=f'tiago_pro::wrist_{side}_ft_sensor_link'))
    node.destroy_node()
    rclpy.shutdown()
