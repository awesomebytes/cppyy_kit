"""Read-only startup proof: controllers and live sensor samples; never commands motion."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from controller_manager_msgs.srv import ListControllers

rclpy.init()
n = rclpy.create_node('jazzy_startup_check')
result = {}
def receive(topic, message):
    value = {'frame': message.header.frame_id}
    if isinstance(message, LaserScan):
        value.update(samples=len(message.ranges), finite=sum(math.isfinite(x) for x in message.ranges))
    elif isinstance(message, (Image, PointCloud2)):
        value.update(width=message.width,height=message.height,bytes=len(message.data))
    elif isinstance(message, WrenchStamped):
        w=message.wrench
        value['wrench']=[w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z]
    elif isinstance(message, Odometry):
        p=message.pose.pose.position
        value['position']=[p.x,p.y,p.z]
    result[topic]=value
specs=[('/scan_front_raw',LaserScan),('/scan_rear_raw',LaserScan),
       ('/head_front_camera/color/image_raw',Image),('/head_front_camera/depth/color/points',PointCloud2),
       ('/wrist_left_ft_broadcaster/wrench',WrenchStamped),('/wrist_right_ft_broadcaster/wrench',WrenchStamped),
       ('/mobile_base_controller/odom',Odometry)]
subs=[n.create_subscription(t, name, lambda m,name=name: receive(name,m), qos_profile_sensor_data) for name,t in specs]
client=n.create_client(ListControllers,'/controller_manager/list_controllers')
future=client.call_async(ListControllers.Request()) if client.wait_for_service(timeout_sec=5) else None
end=time.monotonic()+25
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.2)
    if len(result)==len(specs) and future and future.done(): break
if future and future.done(): result['controllers']={c.name:c.state for c in future.result().controller}
result['missing_topics']=[name for name,_ in specs if name not in result]
p=Path(__file__).parent/'artifacts';p.mkdir(exist_ok=True)
(p/'startup.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
n.destroy_node();rclpy.shutdown()
