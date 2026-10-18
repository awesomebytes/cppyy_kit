"""Cancel ROS navigation/arm goals and command a stopped chassis."""
import json
import time

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
import rclpy

rclpy.init()
node = rclpy.create_node('pro_stop_motion')
stop = node.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)
clients, futures = {}, {}
for action in ['navigate_to_pose', 'move_action', 'execute_trajectory',
               'arm_left_controller/follow_joint_trajectory',
               'arm_right_controller/follow_joint_trajectory',
               'gripper_left_controller/gripper_cmd',
               'gripper_right_controller/gripper_cmd']:
    client = node.create_client(CancelGoal, '/' + action + '/_action/cancel_goal')
    clients[action] = client
deadline = time.monotonic() + 3
while time.monotonic() < deadline:
    stop.publish(Twist())
    for action, client in clients.items():
        if action not in futures and client.service_is_ready():
            futures[action] = client.call_async(CancelGoal.Request())
    rclpy.spin_once(node, timeout_sec=0.05)
report = {}
for action in clients:
    future = futures.get(action)
    result = future.result() if future is not None and future.done() else None
    report[action] = dict(available=future is not None,
        completed=future.done() if future is not None else False,
        return_code=result.return_code if result is not None else None)
print(json.dumps(report, indent=2))
node.destroy_node()
rclpy.shutdown()
