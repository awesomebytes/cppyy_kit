"""Verify base world motion and both seven-joint trajectory controllers."""
import json
import argparse
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

parser = argparse.ArgumentParser()
parser.add_argument("--base-only", action="store_true")
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('pro_direct_control_check')
latest = {}
subscriptions = []
for topic, key, kind in [('/clock', 'clock', Clock), ('/joint_states', 'joints', JointState), ('/mobile_base_controller/odom', 'odom', Odometry)]:
    subscriptions.append(node.create_subscription(kind, topic, lambda message, key=key: latest.update({key: message}), qos_profile_sensor_data))
velocity = node.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)


def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def sim_time():
    stamp = latest['clock'].clock
    return stamp.sec + stamp.nanosec * 1e-9


def advance(seconds):
    target = sim_time() + seconds
    deadline = time.monotonic() + 30
    while sim_time() < target:
        assert time.monotonic() < deadline
        rclpy.spin_once(node, timeout_sec=0.02)


def world_pose():
    output = subprocess.check_output(['gz', 'model', '-m', 'tiago_pro', '-p'], text=True, timeout=10)
    values = [float(value) for value in output.split()]
    assert len(values) == 6, output
    return values


def odom_pose():
    pose = latest['odom'].pose.pose
    q = pose.orientation
    return [pose.position.x, pose.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


report = {'base': [], 'arms': {}}
try:
    spin(3)
    assert {'clock', 'joints', 'odom'} <= latest.keys()
    for name, vx, vy, wz in [('forward', 0.15, 0.0, 0.0), ('lateral', 0.0, 0.15, 0.0), ('yaw', 0.0, 0.0, 0.3)]:
        before = world_pose()
        # Gazebo CLI blocks the client. Drain clock callbacks before timing.
        spin(0.5)
        odom_before = odom_pose()
        start = sim_time()
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = vx, vy, wz
        while sim_time() - start < 2:
            velocity.publish(message)
            spin(0.02)
        velocity.publish(Twist())
        advance(0.3)
        after = world_pose()
        odom_after = odom_pose()
        expected = [2*(math.cos(before[5])*vx - math.sin(before[5])*vy),
                    2*(math.sin(before[5])*vx + math.cos(before[5])*vy), 2*wz]
        measured = [after[0]-before[0], after[1]-before[1], wrap(after[5]-before[5])]
        odometry = [odom_after[0]-odom_before[0], odom_after[1]-odom_before[1], wrap(odom_after[2]-odom_before[2])]
        row = {'command': name, 'expected_dx_dy_yaw': expected, 'gazebo_dx_dy_yaw': measured,
               'odom_dx_dy_yaw': odometry, 'passed': all(abs(a-b) < 0.05 for a,b in zip(expected, measured)) and all(abs(a-b) < 0.03 for a,b in zip(measured, odometry))}
        report['base'].append(row)
        assert row['passed'], row
    robot = ET.parse('generated/robot_description.urdf').getroot()
    limits = {joint.get('name'): joint.find('limit') for joint in robot.findall('joint') if joint.find('limit') is not None}
    for side in ([] if args.base_only else ['left', 'right']):
        names = [f'arm_{side}_{i}_joint' for i in range(1,8)]
        positions = dict(zip(latest['joints'].name, latest['joints'].position))
        targets = []
        for index, name in enumerate(names):
            lower = float(limits[name].get('lower'))
            upper = float(limits[name].get('upper'))
            delta = 0.04 if index % 2 == 0 else -0.04
            if not lower+0.01 < positions[name]+delta < upper-0.01:
                delta = -delta
            targets.append(min(upper-0.01,max(lower+0.01,positions[name]+delta)))
        client = ActionClient(node, FollowJointTrajectory, f'/arm_{side}_controller/follow_joint_trajectory')
        assert client.wait_for_server(timeout_sec=10)
        request = FollowJointTrajectory.Goal()
        request.trajectory.joint_names = names
        point = JointTrajectoryPoint(positions=targets)
        point.time_from_start.sec = 3
        request.trajectory.points = [point]
        future = client.send_goal_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        handle = future.result()
        assert handle and handle.accepted
        completion = handle.get_result_async()
        rclpy.spin_until_future_complete(node, completion, timeout_sec=20)
        assert completion.done() and completion.result().result.error_code == 0
        advance(0.5)
        measured = dict(zip(latest['joints'].name,latest['joints'].position))
        errors = {name: measured[name]-target for name,target in zip(names,targets)}
        report['arms'][side] = {'joints': names, 'targets_rad': targets,
                                'joint_feedback_rad': [measured[name] for name in names],
                                'max_target_error_rad': max(abs(v) for v in errors.values())}
        assert report['arms'][side]['max_target_error_rad'] < 0.015, report['arms'][side]
    report['passed'] = True
    Path('artifacts/direct_control.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
finally:
    velocity.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()
