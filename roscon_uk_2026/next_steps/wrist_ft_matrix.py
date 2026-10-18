"""Same timestamped native wrist force protocol for both TIAGo playgrounds.

Run in robot's Pixi environment against its running headless simulator.
No simulated force rescaling or guessed calibration factors are used.
"""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.srv import ApplyLinkWrench, LinkRequest
from geometry_msgs.msg import WrenchStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

p = argparse.ArgumentParser()
p.add_argument('--robot', choices=['pro', 'dual'], required=True)
p.add_argument('--output', required=True)
p.add_argument('--quick', action='store_true')
p.add_argument('--axis', choices=['x','y','z'])
p.add_argument('--torque', action='store_true', help='Apply pure 0.5 Nm couples on all three world axes')
a = p.parse_args()
rclpy.init()
n = rclpy.create_node('wrist_ft_matrix')
buffer = Buffer()
listener = TransformListener(buffer, n)
rows = {s: [] for s in ['left', 'right']}
latest = {}
subscriptions = []

def sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

def rotate(v, r):
    q = np.array([r.x,r.y,r.z]); v = np.array(v)
    return v + 2*np.cross(q, np.cross(q,v)+r.w*v)

for side in rows:
    def receive(m, side=side):
        target = 'odom' if a.robot=='pro' else 'base_footprint'
        if not buffer.can_transform(target, m.header.frame_id, rclpy.time.Time()): return
        t = buffer.lookup_transform(target, m.header.frame_id, rclpy.time.Time()).transform
        f = [getattr(m.wrench.force, ax) for ax in 'xyz']
        tau = [getattr(m.wrench.torque, ax) for ax in 'xyz']
        world_f=rotate(f,t.rotation); world_t=rotate(tau,t.rotation)
        if a.robot=='dual':
            if 'odom_orientation' not in latest: return
            world_f=rotate(world_f,latest['odom_orientation']);world_t=rotate(world_t,latest['odom_orientation'])
        if 'joints' not in latest or 'odom' not in latest: return
        joint_state=latest['joints']; odom=latest['odom']
        rows[side].append({'sim_s': sec(m.header.stamp), 'sensor_wrench': f+tau,
            'world_force': world_f.tolist(),
            'world_torque': world_t.tolist(),
            'joint_position': list(joint_state.position), 'joint_velocity': list(joint_state.velocity),
            'base_pose_odom': [getattr(odom.pose.pose.position,ax) for ax in 'xyz']+
                [getattr(odom.pose.pose.orientation,ax) for ax in 'xyzw'],
            'base_twist': [getattr(v,ax) for v in [odom.twist.twist.linear,odom.twist.twist.angular] for ax in 'xyz'],
            'sensor_position_odom': [getattr(t.translation,ax) for ax in 'xyz']})
    topic = f'/wrist_{side}_ft_broadcaster/wrench' if a.robot=='pro' else f'/ft_sensor_{side}_controller/wrench'
    subscriptions.append(n.create_subscription(WrenchStamped, topic, receive, qos_profile_sensor_data))
subscriptions.append(n.create_subscription(Clock,'/clock',lambda m:latest.update(clock=sec(m.clock)),qos_profile_sensor_data))
subscriptions.append(n.create_subscription(Odometry,'/mobile_base_controller/odom',lambda m:latest.update(odom=m,odom_orientation=m.pose.pose.orientation),qos_profile_sensor_data))
subscriptions.append(n.create_subscription(JointState,'/joint_states',lambda m:latest.update(joints=m),qos_profile_sensor_data))

def until(t):
    deadline=time.monotonic()+90
    while latest.get('clock',-1)<t:
        assert time.monotonic()<deadline,'Simulation clock stalled'
        rclpy.spin_once(n,timeout_sec=.03)

def now(): return latest['clock']

def summary(side,start,end):
    selected=[r for r in rows[side] if start<=r['sim_s']<=end]
    assert len(selected)>=10, f'Only {len(selected)} samples on {side}'
    out={'window_sim_s':[start,end], 'samples':len(selected)}
    for key in ['world_force','world_torque','sensor_wrench','sensor_position_odom','joint_position','joint_velocity','base_pose_odom','base_twist']:
        values=np.array([r[key] for r in selected])
        out[key+'_mean']=values.mean(axis=0).tolist()
        out[key+'_std']=values.std(axis=0).tolist()
    return out

apply=n.create_client(ApplyLinkWrench,'/apply_link_wrench')
clear=n.create_client(LinkRequest,'/clear_link_wrenches')
def call(client,req):
    future=client.call_async(req)
    rclpy.spin_until_future_complete(n,future,timeout_sec=10)
    assert future.done() and future.result() is not None
    response=future.result()
    if hasattr(response,'success'): assert response.success,response.status_message
    return response

def link(side):
    return f'tiago_pro::wrist_{side}_ft_sensor_link' if a.robot=='pro' else f'tiago_dual::wrist_{side}_ft_link'

report={'robot':a.robot,'protocol':'pure world force at link COM; single wrist; simulated-time windows; vectors rotated into odom before averaging',
    'tolerances': {'force_vector_error_N': .5, 'force_recovery_N': .5, 'pure_torque_vector_error_Nm': .05}, 'tests':[]}
output=Path(a.output); output.parent.mkdir(parents=True,exist_ok=True)
try:
    assert apply.wait_for_service(timeout_sec=10) and clear.wait_for_service(timeout_sec=10)
    deadline=time.monotonic()+60
    while 'clock' not in latest or any(len(values)<10 for values in rows.values()):
        assert time.monotonic()<deadline,'Wrist streams, TF or odometry unavailable'
        rclpy.spin_once(n,timeout_sec=.05)
    report['joint_names']=list(latest['joints'].name)
    until(now()+3)
    forces= [(0,0,5),(5,0,0),(0,5,0),(0,0,10),(0,0,-5),(0,0,5)]
    if a.quick: forces=forces[:1]
    if a.axis: forces=[tuple(5 if ax==a.axis else 0 for ax in 'xyz')]
    loads=[(force,(0,0,0)) for force in forces]
    if a.torque:
        loads=[((0,0,0),tuple(.5 if i==j else 0 for i in range(3))) for j in range(3)]
    for force, torque in loads:
        for side in ['left','right']:
            for other in rows: call(clear, LinkRequest.Request(link_name=link(other)))
            start_base=now()+.4
            until(start_base+1.2)
            baseline=summary(side,start_base,start_base+1)
            start=now()+.4
            req=ApplyLinkWrench.Request(); req.link_name=link(side); req.reference_frame='world'
            for ax,value in zip('xyz',force):setattr(req.wrench.force,ax,float(value))
            for ax,value in zip('xyz',torque):setattr(req.wrench.torque,ax,float(value))
            req.start_time.sec=int(start); req.start_time.nanosec=int((start-int(start))*1e9);req.duration.sec=3
            call(apply,req)
            until(start+4.8)
            loaded=summary(side,start+1,start+2.5)
            released=summary(side,start+3.7,start+4.7)
            delta=np.array(loaded['world_force_mean'])-baseline['world_force_mean']
            torque_delta=np.array(loaded['world_torque_mean'])-baseline['world_torque_mean']
            recovery=np.linalg.norm(np.array(released['world_force_mean'])-baseline['world_force_mean'])
            force_error=np.linalg.norm(delta-np.array(force))
            torque_error=np.linalg.norm(torque_delta-np.array(torque))
            row={'side':side,'link':link(side),'requested_world_force_N':force,'requested_world_torque_Nm':torque,'reference_point':[0,0,0], 'start_sim_s':start,'duration_sim_s':3,
                'baseline':baseline,'loaded':loaded,'released':released,
                'world_force_delta_N':delta.tolist(),'force_norm_N':float(np.linalg.norm(delta)),
                'world_torque_delta_Nm':torque_delta.tolist(), 'recovery_N':float(recovery),
                'vector_error_N':float(force_error),'torque_vector_error_Nm':float(torque_error),
                'passed':bool(force_error<.5 and recovery<.5 and (not a.torque or torque_error<.05))}
            report['tests'].append(row)
            output.write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps({k:row[k] for k in ['side','requested_world_force_N','requested_world_torque_Nm','world_force_delta_N','world_torque_delta_Nm','force_norm_N','recovery_N','passed']}),flush=True)
    report['passed']=all(r['passed'] for r in report['tests'])
finally:
    for side in rows:
        if clear.service_is_ready(): call(clear,LinkRequest.Request(link_name=link(side)))
    output.write_text(json.dumps(report,indent=2)+'\n')
    output.with_suffix('.raw.json').write_text(json.dumps(rows)+'\n')
    n.destroy_node();rclpy.shutdown()

raise SystemExit(0 if report.get('passed', False) else 1)
