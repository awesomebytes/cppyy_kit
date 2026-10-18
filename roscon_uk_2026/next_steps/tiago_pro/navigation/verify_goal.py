"""Send a real Nav2 goal and independently check simulator odometry and yaw."""
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def main():
    rclpy.init()
    node = rclpy.create_node('verify_pro_navigation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    latest = {}
    counts = {'cmd_vel': 0, 'path': 0}
    def command(message):
        if abs(message.linear.x) + abs(message.linear.y) + abs(message.angular.z) > 0.001:
            counts['cmd_vel'] += 1
    def path(message):
        if message.poses:
            counts['path'] += 1
    subscriptions = [
        node.create_subscription(Odometry, '/mobile_base_controller/odom', lambda msg: latest.update(odom=msg), qos_profile_sensor_data),
        node.create_subscription(Twist, '/mobile_base_controller/cmd_vel_unstamped', command, 10),
        node.create_subscription(NavPath, '/plan', path, 10),
    ]
    def wait(predicate, seconds):
        deadline = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError('Navigation verification timed out')
            rclpy.spin_once(node, timeout_sec=0.1)
    try:
        wait(lambda: 'odom' in latest and node.get_clock().now().nanoseconds > 0, 20)
        initial = latest['odom'].pose.pose
        initial_yaw = yaw(initial.orientation)
        goal_x, goal_y, goal_yaw = initial.position.x + 0.8, initial.position.y + 0.4, initial_yaw + 0.4
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = goal_x, goal_y
        goal.pose.pose.orientation.z = math.sin(goal_yaw / 2)
        goal.pose.pose.orientation.w = math.cos(goal_yaw / 2)
        client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        assert client.wait_for_server(timeout_sec=120), 'Nav2 action server unavailable'
        request = client.send_goal_async(goal)
        wait(request.done, 15)
        handle = request.result()
        assert handle.accepted, 'Nav2 rejected goal'
        result = handle.get_result_async()
        try:
            wait(result.done, 180)
        except TimeoutError:
            handle.cancel_goal_async()
            raise
        # Process fresh odometry after the action result.
        previous_stamp = latest['odom'].header.stamp
        wait(lambda: latest['odom'].header.stamp != previous_stamp, 5)
        final = latest['odom'].pose.pose
        distance = math.hypot(final.position.x - goal_x, final.position.y - goal_y)
        heading = angle_error(yaw(final.orientation), goal_yaw)
        report = {
            'localization': 'simulation planar-plugin odometry; map equals odom',
            'initial': {'x': initial.position.x, 'y': initial.position.y, 'yaw': initial_yaw},
            'goal': {'x': goal_x, 'y': goal_y, 'yaw': goal_yaw},
            'final': {'x': final.position.x, 'y': final.position.y, 'yaw': yaw(final.orientation)},
            'action_status': result.result().status,
            'position_error_m': distance, 'yaw_error_rad': heading, **counts,
            'passed': result.result().status == GoalStatus.STATUS_SUCCEEDED and distance < 0.15 and heading < 0.18 and counts['cmd_vel'] > 0 and counts['path'] > 0,
        }
        output = Path(__file__).parent / 'artifacts/goal_validation.json'
        output.parent.mkdir(exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        assert report['passed'], report
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
