"""Save separate setup, extraction, conversion and query measurements."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import resource
import statistics
import time

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from extract import extract_poses, count_speed_above, generate_fixture, SETUP_TIMINGS


def python_load(path):
    log, publish, stamp, position, quaternion = [], [], [], [], []
    with open(path, "rb") as source:
        reader = make_reader(source, decoder_factories=[DecoderFactory()])
        for _, _, message, decoded in reader.iter_decoded_messages(topics=["/pose"], log_time_order=False):
            log.append(message.log_time); publish.append(message.publish_time)
            stamp.append(decoded.header.stamp.sec*10**9+decoded.header.stamp.nanosec)
            p, q = decoded.pose.position, decoded.pose.orientation
            position.append((p.x,p.y,p.z)); quaternion.append((q.x,q.y,q.z,q.w))
    return np.array(log,np.uint64),np.array(publish,np.uint64),np.array(stamp,np.int64),np.array(position),np.array(quaternion)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples",type=int,default=600)
    parser.add_argument("--output",type=Path,default=Path(__file__).parent/"measurement.json")
    args=parser.parse_args()
    path=Path(__file__).parent/"build/measurement.mcap"
    generation=time.perf_counter();generate_fixture(path,args.samples);generation_ms=(time.perf_counter()-generation)*1000
    first=time.perf_counter();batch=extract_poses(path);first_ms=(time.perf_counter()-first)*1000
    python=[];native=[];components=[];query=[]
    for _ in range(9):
        started=time.perf_counter();reference=python_load(path);python.append(1000*(time.perf_counter()-started))
        started=time.perf_counter();batch=extract_poses(path);native.append(1000*(time.perf_counter()-started));components.append(batch.timings_ms)
        for actual,expected in zip([batch.log_time_ns,batch.publish_time_ns,batch.header_time_ns,batch.position_m,batch.quaternion_xyzw],reference):
            np.testing.assert_array_equal(actual,expected)
        started=time.perf_counter();matches=count_speed_above(batch,.3);query.append(1000*(time.perf_counter()-started))
    baseline_speed=np.linalg.norm(np.diff(batch.position_m,axis=0),axis=1)/(np.diff(batch.log_time_ns).astype(float)*1e-9)
    assert matches==int(np.count_nonzero(baseline_speed>.3))
    result={"samples":args.samples,"input_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
            "file_bytes":path.stat().st_size,"fixture_generation_ms":generation_ms,"first_native_total_ms":first_ms,
            "setup_ms":SETUP_TIMINGS,"python_decode_and_numpy_median_ms":statistics.median(python),
            "native_extract_and_numpy_median_ms":statistics.median(native),
            "native_components_median_ms":{k:statistics.median(x[k] for x in components) for k in components[0]},
            "speed_query_median_ms":statistics.median(query),"speed_query_matches":matches,
            "process_peak_rss_kib":resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            "versions":{p:importlib.metadata.version(p) for p in ["numpy","cppyy","mcap","mcap-ros2-support"]},
            "timing_conditions":"9 sequential warmed reads, filesystem cache warm; conversion is an owned bulk copy; container timing includes I/O and parsing; compressed chunks rejected"}
    args.output.write_text(json.dumps(result,indent=2)+"\n");print(json.dumps(result,indent=2))


if __name__=="__main__":main()
