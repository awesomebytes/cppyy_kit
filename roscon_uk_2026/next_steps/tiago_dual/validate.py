"""Observe sensors and verify commanded base/arm motion in the live simulator."""
import json
import math
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, WrenchStamped
from gazebo_msgs.srv import ApplyLinkWrench
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, JointState, LaserScan, PointCloud2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data, QoSProfile

rclpy.init()
node = rclpy.create_node('tiago_dual_validation')
latest = {}
counts = defaultdict(int)
subscriptions = []
wrench_samples = {side: [] for side in ('left', 'right')}
for key, topic, kind in [
    ('clock', '/clock', Clock), ('imu', '/base_imu', Imu),
    ('rgb', '/head_front_camera/rgb/image_raw', Image),
    ('depth', '/head_front_camera/depth/image_raw', Image),
    ('cloud', '/head_front_camera/depth/rgb/points', PointCloud2),
    ('scan', '/scan_raw', LaserScan), ('joints', '/joint_states', JointState),
    ('odom', '/mobile_base_controller/odom', Odometry),
    ('ft_left', '/ft_sensor_left_controller/wrench', WrenchStamped),
    ('ft_right', '/ft_sensor_right_controller/wrench', WrenchStamped),
]:
    def callback(msg, key=key):
        latest[key] = msg
        counts[key] += 1
        if key.startswith('ft_'):
            wrench_samples[key[3:]].append((time.monotonic(), msg))
    subscriptions.append(node.create_subscription(kind, topic, callback, QoSProfile(depth=5) if key in ('rgb', 'depth', 'cloud') else qos_profile_sensor_data))

def spin(seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.05)

spin(12)
report = {'message_counts': dict(counts)}
required = {'clock', 'imu', 'rgb', 'depth', 'cloud', 'scan', 'joints', 'odom', 'ft_left', 'ft_right'}
report['missing_streams'] = sorted(required - set(latest))
Path('validation.json').write_text(json.dumps(report, indent=2) + '\n')
assert not report['missing_streams'], report
start_clock = latest['clock'].clock
start_odom = latest['odom'].pose.pose.position
start_joints = dict(zip(latest['joints'].name, latest['joints'].position))
base = node.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)
arms = {side: node.create_publisher(JointTrajectory, f'/arm_{side}_controller/joint_trajectory', 10)
        for side in ('left', 'right')}
spin(1)
for side, publisher in arms.items():
    names = [f'arm_{side}_{i}_joint' for i in range(1, 8)]
    positions = [start_joints[name] for name in names]
    positions[0] += 0.15
    point = JointTrajectoryPoint(positions=positions)
    point.time_from_start.sec = 2
    publisher.publish(JointTrajectory(joint_names=names, points=[point]))
command = Twist()
command.linear.x = 0.1
for _ in range(35):
    base.publish(command)
    spin(0.1)
base.publish(Twist())
spin(3)
end_joints = dict(zip(latest['joints'].name, latest['joints'].position))
end_odom = latest['odom'].pose.pose.position
end_clock = latest['clock'].clock
report.update({
    'message_counts': dict(counts),
    'simulated_seconds_advanced': end_clock.sec - start_clock.sec + (end_clock.nanosec-start_clock.nanosec)/1e9,
    'base_odometry_displacement_m': math.hypot(end_odom.x-start_odom.x, end_odom.y-start_odom.y),
    'arm_joint_1_delta_rad': {side: end_joints[f'arm_{side}_1_joint'] - start_joints[f'arm_{side}_1_joint']
                              for side in ('left', 'right')},
    'rgb': {'width': latest['rgb'].width, 'height': latest['rgb'].height, 'encoding': latest['rgb'].encoding},
    'depth': {'width': latest['depth'].width, 'height': latest['depth'].height, 'encoding': latest['depth'].encoding},
    'pointcloud_points': latest['cloud'].width * latest['cloud'].height,
    'scan_ranges': len(latest['scan'].ranges),
})
motion_passed = (report['simulated_seconds_advanced'] > 1
                    and report['base_odometry_displacement_m'] > 0.03
                    and all(abs(delta - 0.15) < 0.05 for delta in report['arm_joint_1_delta_rad'].values())
                    and report['pointcloud_points'] > 0 and report['scan_ranges'] > 0)

def wrench_axes(message):
    return [getattr(part, axis) for part in (message.wrench.force, message.wrench.torque)
            for axis in ('x', 'y', 'z')]

def mean_axes(samples):
    rows = [wrench_axes(message) for _, message in samples]
    return [sum(row[i] for row in rows) / len(rows) for i in range(6)]

def clock_seconds():
    clock = latest['clock'].clock
    return clock.sec + clock.nanosec / 1e9

def advance_sim(seconds):
    target = clock_seconds() + seconds
    timeout = time.monotonic() + 15
    while clock_seconds() < target:
        if time.monotonic() > timeout:
            raise RuntimeError('Simulation clock stalled during F/T validation')
        rclpy.spin_once(node, timeout_sec=0.05)

# Apply a bounded external force through Gazebo's existing service. Compare a
# settled baseline, the loaded interval, and the baseline after force removal.
wrench_samples = {side: [] for side in ('left', 'right')}
advance_sim(0.5)
baseline = {side: mean_axes(samples) for side, samples in wrench_samples.items()}
client = node.create_client(ApplyLinkWrench, '/apply_link_wrench')
assert client.wait_for_service(timeout_sec=5), 'Gazebo apply_link_wrench service unavailable'
applications = {}
for side in ('left', 'right'):
    request = ApplyLinkWrench.Request()
    request.link_name = f'tiago_dual::wrist_{side}_ft_link'
    request.reference_frame = 'world'
    request.wrench.force.z = 5.0
    # The stock service uses absolute simulation time; zero would already have
    # expired by this stage even though its interface comment suggests otherwise.
    request.start_time = latest['clock'].clock
    request.duration.sec = 3
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    assert future.done() and future.result() is not None, 'Force application timed out'
    response = future.result()
    applications[side] = {'success': response.success, 'message': response.status_message}
    assert response.success, applications
advance_sim(0.2)
wrench_samples = {side: [] for side in ('left', 'right')}
advance_sim(0.5)
loaded = {side: mean_axes(samples) for side, samples in wrench_samples.items()}
advance_sim(3.2)
wrench_samples = {side: [] for side in ('left', 'right')}
advance_sim(0.5)
released = {side: mean_axes(samples) for side, samples in wrench_samples.items()}
ft_report = {}
for side in ('left', 'right'):
    delta = [value - initial for value, initial in zip(loaded[side], baseline[side])]
    response_norm = math.sqrt(sum(value ** 2 for value in delta[:3]))
    recovery = math.sqrt(sum((released[side][i] - baseline[side][i]) ** 2 for i in range(3)))
    finite = all(math.isfinite(value) for row in (baseline[side], loaded[side], released[side]) for value in row)
    ft_report[side] = {
        'frame_id': latest[f'ft_{side}'].header.frame_id,
        'baseline_force_N_torque_Nm': baseline[side],
        'loaded_force_N_torque_Nm': loaded[side],
        'released_force_N_torque_Nm': released[side],
        'applied_force_world_z_N': 5.0,
        'observed_force_change_norm_N': response_norm,
        'release_force_residual_N': recovery,
        'finite_six_axes': finite,
        'service_result': applications[side],
        'passed': (finite and latest[f'ft_{side}'].header.frame_id == f'wrist_{side}_ft_link'
                   and counts[f'ft_{side}'] > 1 and abs(response_norm - 5.0) < 0.5
                   and recovery < 0.5),
    }
report['force_torque'] = ft_report
report['message_counts'] = dict(counts)
report['passed'] = motion_passed and all(result['passed'] for result in ft_report.values())
Path('validation.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
node.destroy_node()
rclpy.shutdown()
assert report['passed'], report
