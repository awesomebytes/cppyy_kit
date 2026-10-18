"""Exercise native effort GripperCommand actions and record actual feedback."""
import argparse
import json
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

p = argparse.ArgumentParser()
p.add_argument('--positions', type=float, nargs='+', default=[.04,.065])
p.add_argument('--output', default='artifacts/gripper_action_validation.json')
a = p.parse_args()
assert all(0 <= value <= .07 for value in a.positions)
rclpy.init()
node = rclpy.create_node('physical_gripper_action_check')
latest = {}
rows = []
joint_trace = []


def receive_joints(message):
    latest['joints'] = message
    selected = {}
    for index,name in enumerate(message.name):
        if name in ['gripper_left_finger_joint','gripper_right_finger_joint']:
            selected[name] = dict(position=message.position[index],velocity=message.velocity[index],
                effort=message.effort[index] if index<len(message.effort) else None)
    joint_trace.append(dict(sim_s=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,joints=selected))


subscriptions = [
    node.create_subscription(Clock,'/clock',lambda m:latest.update(clock=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data),
    node.create_subscription(JointState,'/joint_states',receive_joints,qos_profile_sensor_data)]


def spin_until(predicate, timeout=25):
    deadline = time.monotonic()+timeout
    while not predicate():
        assert time.monotonic()<deadline, 'Native gripper action/feedback timeout'
        rclpy.spin_once(node,timeout_sec=.03)


def capture_position(side):
    state = latest['joints']
    index = list(state.name).index(f'gripper_{side}_finger_joint')
    return state.position[index],state.velocity[index]


output = Path(a.output)
active_handle = None
requests = []
error = None
try:
    spin_until(lambda: 'clock' in latest and 'joints' in latest)
    clients = {side:ActionClient(node,GripperCommand,f'/gripper_{side}_controller/gripper_cmd') for side in ['left','right']}
    for client in clients.values():
        assert client.wait_for_server(timeout_sec=10), 'Native effort GripperActionController unavailable'
    for target in a.positions:
        for side, client in clients.items():
            initial,_ = capture_position(side)
            feedback = []
            goal = GripperCommand.Goal()
            goal.command.position = target
            goal.command.max_effort = 10.0
            future = client.send_goal_async(goal,feedback_callback=lambda m:feedback.append(dict(position=m.feedback.position,effort=m.feedback.effort,reached_goal=m.feedback.reached_goal,stalled=m.feedback.stalled)))
            spin_until(future.done)
            handle = future.result()
            assert handle.accepted
            active_handle = handle
            requests.append(dict(side=side,target_m=target,accepted=True,start_sim_s=latest['clock']))
            result_future = handle.get_result_async()
            spin_until(result_future.done)
            result = result_future.result()
            active_handle = None
            # The stock controller continues physically holding after success.
            start = latest['clock']
            spin_until(lambda: latest['clock'] >= start+2.5)
            final,velocity = capture_position(side)
            row = dict(side=side,target_m=target,initial_m=initial,final_m=final,velocity_m_s=velocity,
                       status=result.status,reached_goal=result.result.reached_goal,stalled=result.result.stalled,
                       action_result_position_m=result.result.position,action_result_effort_N=result.result.effort,
                       feedback=feedback,passed=bool(result.status==GoalStatus.STATUS_SUCCEEDED and result.result.reached_goal and not result.result.stalled and abs(final-target)<.001))
            rows.append(row)
            output.write_text(json.dumps({'tests':rows,'passed':all(r['passed'] for r in rows)},indent=2)+'\n')
            print(json.dumps({k:v for k,v in row.items() if k!='feedback'}),flush=True)
    assert all(r['passed'] for r in rows), 'Native effort gripper action check failed'
except Exception as exc:
    error = str(exc)
    raise
finally:
    if active_handle is not None:
        cancellation = active_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node,cancellation,timeout_sec=3)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(dict(tests=rows,requests=requests,error=error,
        joint_trace=joint_trace,passed=bool(rows and all(r['passed'] for r in rows) and error is None)),indent=2)+'\n')
    node.destroy_node()
    rclpy.shutdown()
