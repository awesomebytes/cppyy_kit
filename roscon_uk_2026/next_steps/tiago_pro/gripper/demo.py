"""Visible, bounded native gripper actions; arms and base are never commanded."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState

p = argparse.ArgumentParser()
p.add_argument('--cycles', type=int, default=3)
p.add_argument('--hold', type=float, default=4.5, help='Wall seconds at each endpoint')
p.add_argument('--output', default='artifacts/gripper_full_visual_demo.json')
a = p.parse_args()
assert 1 <= a.cycles <= 10 and 0 <= a.hold <= 30
# Read the same expanded model used by the running playground. PAL's wrapper
# and named motions define the lower/upper lead limits as closed/open.
model_path = Path(__file__).resolve().parents[1]/'generated/sim.urdf'
robot = ET.parse(model_path).getroot()
joints = {j.get('name'):j for j in robot.findall('joint')
          if j.get('name','').startswith('gripper_') and j.get('type') != 'fixed'}
endpoint_limits = {}
for side in ['left','right']:
    lead = f'gripper_{side}_finger_joint'
    limit = joints[lead].find('limit')
    low,high = float(limit.get('lower')),float(limit.get('upper'))
    allowed_low,allowed_high = low,high
    for name,joint in joints.items():
        mimic = joint.find('mimic')
        if mimic is None or mimic.get('joint') != lead:
            continue
        ratio,offset = float(mimic.get('multiplier','1')),float(mimic.get('offset','0'))
        bounds = joint.find('limit')
        follower_low,follower_high = float(bounds.get('lower')),float(bounds.get('upper'))
        if ratio == 0:
            assert follower_low <= offset <= follower_high
            continue
        a_low,a_high = sorted([(follower_low-offset)/ratio,(follower_high-offset)/ratio])
        allowed_low,allowed_high = max(allowed_low,a_low),min(allowed_high,a_high)
    assert abs(allowed_low-low)<1e-12 and abs(allowed_high-high)<1e-12, 'Follower limits prohibit full lead travel'
    endpoint_limits[side] = dict(closed=low,open=high)
assert endpoint_limits['left'] == endpoint_limits['right']
closed,opened = endpoint_limits['left']['closed'],endpoint_limits['left']['open']
rclpy.init()
n = rclpy.create_node('physical_gripper_visual_demo')
latest = {}
sub = n.create_subscription(JointState,'/joint_states',lambda m:latest.update(state=m),qos_profile_sensor_data)
clients = {side:ActionClient(n,GripperCommand,f'/gripper_{side}_controller/gripper_cmd') for side in ['left','right']}
active = []
rows = []
error = None

def wait(predicate, timeout=25):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError('Native gripper demo timeout')
        rclpy.spin_once(n,timeout_sec=.03)

def measured():
    state = latest['state']
    return {side:state.position[list(state.name).index(f'gripper_{side}_finger_joint')] for side in clients}

try:
    wait(lambda:'state' in latest)
    for client in clients.values():
        assert client.wait_for_server(timeout_sec=10), 'Gripper action server unavailable'
    print(f'FULL VISUAL DEMO START: arms/base fixed; fully open {opened:.3f} / fully closed {closed:.3f} m (URDF lead limits).',flush=True)
    for step,target in enumerate([opened,closed]*a.cycles+[opened],1):
        initial = measured()
        futures = {}
        for side,client in clients.items():
            goal = GripperCommand.Goal()
            goal.command.position = target
            goal.command.max_effort = 10.0
            futures[side] = client.send_goal_async(goal)
        results = {}
        for side,future in futures.items():
            wait(future.done)
            handle = future.result()
            assert handle.accepted, f'{side} rejected goal'
            active.append(handle)
            results[side] = handle.get_result_async()
        wait(lambda:all(f.done() for f in results.values()))
        outcomes = {side:f.result() for side,f in results.items()}
        assert all(r.status==4 and (r.result.reached_goal or r.result.stalled) for r in outcomes.values()), 'Native gripper action failed'
        active.clear()
        print(f'Step {step}: both hands reached {target:.3f} m; holding {a.hold:g} seconds',flush=True)
        end = time.monotonic()+a.hold
        wait(lambda:time.monotonic()>=end,timeout=a.hold+5)
        final = measured()
        state = latest['state']
        actual = {name:state.position[list(state.name).index(name)] for name in joints}
        expected = {}
        for name,joint in joints.items():
            mimic = joint.find('mimic')
            expected[name] = target if mimic is None else target*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0'))
        row = dict(step=step,target_m=target,initial_m=initial,final_m=final,hold_wall_s=a.hold,endpoint="fully_open" if target==opened else "fully_closed",
                   actual_native_joints=actual,ideal_mimic_joints=expected,
                   actions={side:dict(status=r.status,reached_goal=r.result.reached_goal,stalled=r.result.stalled,position_m=r.result.position) for side,r in outcomes.items()},
                   passed=all(abs(v-target)<.001 for v in final.values()))
        rows.append(row)
        assert row['passed'], 'Measured gripper endpoint exceeds 1 mm tolerance'
except BaseException as exc:
    error = str(exc)
    raise
finally:
    for handle in active:
        future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(n,future,timeout_sec=3)
    output = Path(a.output)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(dict(model_path=str(model_path),limits=endpoint_limits,tests=rows,error=error,passed=bool(rows and error is None)),indent=2)+'\n')
    n.destroy_node()
    rclpy.shutdown()
