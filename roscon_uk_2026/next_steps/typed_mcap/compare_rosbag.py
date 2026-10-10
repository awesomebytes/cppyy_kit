"""Checkout-only comparison with existing rclcpp_kit rosbag2 and CDR serialization."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path")
    parser.add_argument("--output",type=Path,default=Path(__file__).parent/"rosbag_comparison.json")
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[3]
    sys.path.insert(0,str(root/"rclcpp_kit"))
    started=time.perf_counter()
    from rclcpp_kit import rosbag2_cpp,serialization
    from geometry_msgs.msg import PoseStamped
    setup_ms=(time.perf_counter()-started)*1000
    times=[]
    for _ in range(4):
        started=time.perf_counter();reader=rosbag2_cpp.open_reader(args.path)
        log,publish,stamp,positions,quaternions=[],[],[],[],[]
        for message in rosbag2_cpp.iter_messages(reader):
            if str(message.topic_name)!="/pose":continue
            raw=message.serialized_data
            view=raw.buffer;view.reshape((raw.buffer_length,))
            serialized=serialization.serialized_message_from_bytes(bytes(view))
            decoded=serialization.deserialize_message(serialized,PoseStamped)
            log.append(message.recv_timestamp);publish.append(message.send_timestamp)
            stamp.append(decoded.header.stamp.sec*10**9+decoded.header.stamp.nanosec)
            positions.append((decoded.pose.position.x,decoded.pose.position.y,decoded.pose.position.z))
            q=decoded.pose.orientation;quaternions.append((q.x,q.y,q.z,q.w))
        times.append((time.perf_counter()-started)*1000)
        del reader
    from extract import extract_poses
    batch=extract_poses(args.path)
    for actual,expected in [(batch.log_time_ns,log),(batch.publish_time_ns,publish),(batch.header_time_ns,stamp),(batch.position_m,positions),(batch.quaternion_xyzw,quaternions)]:
        np.testing.assert_array_equal(actual,expected)
    result={"samples":len(log),"imports_setup_ms":setup_ms,"first_rosbag_decode_ms":times[0],
            "warm_rosbag_decode_median_ms":float(np.median(times[1:])),"parity":"exact recorded times, Cartesian positions and xyzw orientations",
            "scope":"existing rosbag2_cpp C++ container + rclcpp CDR serializer; Python per-message loop, serialized bytes copies and native objects included"}
    args.output.write_text(json.dumps(result,indent=2)+"\n");print(json.dumps(result,indent=2))


if __name__=="__main__":main()
