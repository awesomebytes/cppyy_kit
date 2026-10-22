"""Send the RViz feedback protocol to verify force, release and measured arm motion."""
import json
import os
from pathlib import Path
import time
import sys

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.srv import GetInteractiveMarkers
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'control_kit/demos'))
from arm_kinematics import Chain

os.environ.update(ROS_DOMAIN_ID='77', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
rclpy.init()
node = rclpy.create_node('force_marker_recording')
client = node.create_client(GetInteractiveMarkers, '/virtual_force/get_interactive_markers')
if not client.wait_for_service(timeout_sec=60):
    raise TimeoutError('No virtual force interactive marker server')
future = client.call_async(GetInteractiveMarkers.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=10)
marker = future.result().markers[0]
state = {}
node.create_subscription(JointState, '/joint_states', lambda m: state.update(zip(m.name,m.position)), qos_profile_sensor_data)
publisher = node.create_publisher(InteractiveMarkerFeedback, '/virtual_force/feedback', 10)
joints = [f'arm_left_{i}_joint' for i in range(1,8)]
model = Path(__file__).resolve().parent.parent / 'tiago_pro_jazzy/generated/robot_description.urdf'
chain = Chain(str(model), joints, 'torso_lift_link', 'gripper_left_grasping_link')
def xyz():
    return chain.fk(np.array([state[j] for j in joints], dtype=np.float64))[:3,3].copy()
end = time.monotonic()+10
while not all(j in state for j in joints):
    if time.monotonic()>end:
        raise TimeoutError('No arm feedback')
    rclpy.spin_once(node,timeout_sec=.1)
start = xyz()
message = InteractiveMarkerFeedback()
message.header = marker.header
message.client_id = 'force_marker_recording'
message.marker_name = marker.name
message.control_name = 'move_x'
message.pose = marker.pose
message.event_type = InteractiveMarkerFeedback.MOUSE_DOWN
publisher.publish(message)
for _ in range(5):
    rclpy.spin_once(node,timeout_sec=.05)
message.pose.position.x += .1  # 20 N/m * 0.1 m = 2 N.
samples=[]
try:
    end=time.monotonic()+8
    while time.monotonic()<end:
        message.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        publisher.publish(message)
        rclpy.spin_once(node,timeout_sec=.05)
        samples.append(float(np.linalg.norm(xyz()-start)))
finally:
    message.event_type = InteractiveMarkerFeedback.MOUSE_UP
    publisher.publish(message)
end=time.monotonic()+9
while time.monotonic()<end:
    rclpy.spin_once(node,timeout_sec=.05)
returned=float(np.linalg.norm(xyz()-start))
result={'force_N':2.,'peak_displacement_m':max(samples),'return_error_m':returned,
        'passed':max(samples)>.003 and returned<.008}
output=Path(__file__).with_name('evidence')/'rviz_force.json'
output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
node.destroy_node()
rclpy.shutdown()
if not result['passed']:
    raise RuntimeError('Marker-driven force response failed the motion check')
