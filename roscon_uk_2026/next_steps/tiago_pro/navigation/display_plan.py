"""Ask the native Nav2 planner for a visible path, without executing it."""
import argparse
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dx', type=float, default=0.8, help='Map-frame offset in metres')
    parser.add_argument('--dy', type=float, default=0.4, help='Map-frame offset in metres')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('pro_navigation_plan_preview', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    observed = []
    subscription = node.create_subscription(NavPath, '/plan', observed.append, 1)
    def wait(predicate, seconds=20):
        deadline = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError('Native planning preview timed out')
            rclpy.spin_once(node, timeout_sec=0.1)
    try:
        wait(lambda: buffer.can_transform('map', 'base_footprint', rclpy.time.Time()))
        pose = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        goal = ComputePathToPose.Goal()
        goal.use_start = False  # Planner obtains the actual robot pose from its costmap TF.
        goal.planner_id = 'GridBased'
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x = pose.translation.x + args.dx
        goal.goal.pose.position.y = pose.translation.y + args.dy
        goal.goal.pose.orientation = pose.rotation
        client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        assert client.wait_for_server(timeout_sec=20), 'Native planner action unavailable'
        request = client.send_goal_async(goal)
        wait(request.done)
        handle = request.result()
        assert handle.accepted, 'Native planner rejected preview request'
        result = handle.get_result_async()
        wait(result.done)
        response = result.result()
        assert response.status == GoalStatus.STATUS_SUCCEEDED, response.status
        path = response.result.path
        assert path.poses, 'Planner returned an empty path'
        wait(lambda: bool(observed))
        report = {
            'action': '/compute_path_to_pose', 'action_status': response.status,
            'execution_requested': False,
            'path_frame': path.header.frame_id, 'path_pose_count': len(path.poses),
            'observed_native_plan_topic': '/plan', 'observed_plan_pose_count': len(observed[-1].poses),
            'goal_xy_m': [goal.goal.pose.position.x, goal.goal.pose.position.y],
            'endpoint_error_m': math.hypot(path.poses[-1].pose.position.x - goal.goal.pose.position.x,
                                           path.poses[-1].pose.position.y - goal.goal.pose.position.y),
        }
        output = Path(__file__).parent / 'artifacts/plan_preview.json'
        output.parent.mkdir(exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
