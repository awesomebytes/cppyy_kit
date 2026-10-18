"""Check actual data and commanded motion against a running isolated TIAGo."""
import json
import math
from pathlib import Path
import time
import struct
import zlib
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, JointState, LaserScan, PointCloud2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    rclpy.init()
    node = rclpy.create_node('tiago_single_smoke')
    counts, latest, first = {}, {}, {}
    topics = {
        '/clock': Clock, '/joint_states': JointState,
        '/mobile_base_controller/odom': Odometry,
        '/head_front_camera/rgb/image_raw': Image,
        '/head_front_camera/depth/image_raw': Image,
        '/head_front_camera/depth/rgb/points': PointCloud2,
        '/scan_raw': LaserScan, '/base_imu': Imu,
    }
    def receive(topic, msg):
        counts[topic] = counts.get(topic, 0) + 1
        latest[topic] = msg
        first.setdefault(topic, msg)
    subscriptions = [node.create_subscription(kind, topic, lambda msg,t=topic:receive(t,msg), 10 if kind in (Image, PointCloud2) else qos_profile_sensor_data) for topic,kind in topics.items()]
    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
    deadline=time.monotonic()+40
    while time.monotonic()<deadline and not all(counts.get(t,0)>=2 for t in topics):
        spin(.2)
    missing=[t for t in topics if counts.get(t,0)<2]
    assert not missing, f'Missing repeated messages: {missing}; counts={counts}'
    start = latest['/mobile_base_controller/odom'].pose.pose.position
    joints = dict(zip(latest['/joint_states'].name, latest['/joint_states'].position))
    arm_names=[f'arm_{i}_joint' for i in range(1,8)]
    before=joints['arm_1_joint']
    target=[joints[j] for j in arm_names]
    target[0] += .15
    basepub=node.create_publisher(Twist,'/mobile_base_controller/cmd_vel_unstamped',10)
    armpub=node.create_publisher(JointTrajectory,'/arm_controller/joint_trajectory',10)
    spin(1)
    trajectory=JointTrajectory(joint_names=arm_names)
    point=JointTrajectoryPoint(positions=target)
    point.time_from_start.sec=2
    trajectory.points=[point]
    armpub.publish(trajectory)
    end=time.monotonic()+5
    twist=Twist(); twist.linear.x=.15
    while time.monotonic()<end:
        basepub.publish(twist)
        spin(.05)
    for _ in range(5):
        basepub.publish(Twist()); spin(.05)
    spin(1)
    finish=latest['/mobile_base_controller/odom'].pose.pose.position
    distance=math.hypot(finish.x-start.x,finish.y-start.y)
    after=dict(zip(latest['/joint_states'].name, latest['/joint_states'].position))['arm_1_joint']
    clock=lambda m:m.clock.sec+m.clock.nanosec/1e9
    image=latest['/head_front_camera/rgb/image_raw']
    depth=latest['/head_front_camera/depth/image_raw']
    cloud=latest['/head_front_camera/depth/rgb/points']
    scan=latest['/scan_raw']
    result={'message_counts':counts,'clock_advance_s':clock(latest['/clock'])-clock(first['/clock']), 'base_displacement_m':distance,'arm_1_before_rad':before,'arm_1_after_rad':after,'arm_1_change_rad':after-before,'rgb':{'width':image.width,'height':image.height,'encoding':image.encoding,'bytes':len(image.data)},'depth':{'width':depth.width,'height':depth.height,'encoding':depth.encoding,'bytes':len(depth.data)},'pointcloud':{'width':cloud.width,'height':cloud.height,'bytes':len(cloud.data)},'lidar_samples':len(scan.ranges)}
    assert result['clock_advance_s']>1, result
    assert distance>.03, result
    assert abs((after-before)-.15)<.05, result
    assert len(image.data)>0 and len(depth.data)>0 and len(cloud.data)>0 and len(scan.ranges)>0, result
    values = [v[0] for v in struct.iter_unpack('<f',bytes(depth.data))]
    result['depth']['finite_fraction'] = sum(math.isfinite(v) and v>0 for v in values)/len(values)
    assert result['depth']['finite_fraction'] > .01, result
    Path('evidence').mkdir(exist_ok=True)
    Path('evidence/smoke.json').write_text(json.dumps(result,indent=2)+'\n')
    rgb=bytes(image.data)
    if image.encoding=='bgr8':
        converted=bytearray(len(rgb))
        converted[0::3]=rgb[2::3];converted[1::3]=rgb[1::3];converted[2::3]=rgb[0::3]
        rgb=bytes(converted)
    assert image.encoding in ('rgb8','bgr8'), image.encoding
    def chunk(kind,data):
        return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
    rows=b''.join(b'\x00'+rgb[y*image.step:y*image.step+image.width*3] for y in range(image.height))
    png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',image.width,image.height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(rows))+chunk(b'IEND',b'')
    Path('evidence/camera.png').write_bytes(png)
    print(json.dumps(result,indent=2))
    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
