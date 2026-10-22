"""Measure real Nav2 goal translation against the robot's forward camera axis."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dx', type=float, default=0.8)
    parser.add_argument('--dy', type=float, default=0.0)
    parser.add_argument('--goal-yaw', type=float, default=0.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('verify_forward_navigation',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    latest, commands, poses = {}, [], []
    def odom(msg):
        latest['odom'] = msg
        p = msg.pose.pose
        poses.append((msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                      p.position.x, p.position.y, yaw(p.orientation)))
    def command(msg):
        if math.hypot(msg.linear.x, msg.linear.y) > 0.03:
            commands.append([msg.linear.x, msg.linear.y, msg.angular.z])
    node.proof_subscriptions = [
        node.create_subscription(Odometry, '/mobile_base_controller/odom', odom, qos_profile_sensor_data),
        node.create_subscription(Twist, '/mobile_base_controller/cmd_vel_unstamped', command, 10)]
    def wait(predicate, timeout):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('Nav2 verification timeout')
            rclpy.spin_once(node, timeout_sec=.1)
    try:
        wait(lambda: 'odom' in latest, 20)
        start = latest['odom'].pose.pose
        gx, gy = start.position.x+args.dx, start.position.y+args.dy
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.z = math.sin(args.goal_yaw/2)
        goal.pose.pose.orientation.w = math.cos(args.goal_yaw/2)
        client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        assert client.wait_for_server(timeout_sec=30)
        sent = client.send_goal_async(goal)
        wait(sent.done, 10)
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        try:
            wait(result.done, 180)
        except TimeoutError:
            handle.cancel_goal_async()
            raise
        final = latest['odom'].pose.pose
        errors = [abs(math.atan2(vy, vx)) for vx, vy, _ in commands]
        outside = sum(e > math.pi/6 for e in errors)
        traveled = 0.0
        outside_distance = 0.0
        for a, b in zip(poses, poses[1:]):
            dx, dy = b[1]-a[1], b[2]-a[2]
            d = math.hypot(dx, dy)
            if d < .0001:
                continue
            error = abs(math.atan2(math.sin(math.atan2(dy, dx)-a[3]), math.cos(math.atan2(dy, dx)-a[3])))
            traveled += d
            if error > math.pi/6:
                outside_distance += d
        report = {
            'action_status': result.result().status,
            'initial': [start.position.x, start.position.y, yaw(start.orientation)],
            'goal': [gx, gy, args.goal_yaw],
            'final': [final.position.x, final.position.y, yaw(final.orientation)],
            'position_error_m': math.hypot(final.position.x-gx, final.position.y-gy),
            'yaw_error_rad': abs(math.atan2(math.sin(yaw(final.orientation)-args.goal_yaw),
                                           math.cos(yaw(final.orientation)-args.goal_yaw))),
            'moving_commands': len(commands), 'commands_outside_30deg': outside,
            'max_command_heading_error_rad': max(errors, default=0),
            'distance_m': traveled, 'distance_outside_30deg_m': outside_distance,
        }
        report['passed'] = (report['action_status'] == 4 and report['position_error_m'] < .15
                            and report['yaw_error_rad'] < .18)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        assert report['passed']
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
