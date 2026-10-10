"""Validate opt-in public samples against the real Python ROS 2 decoder."""
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from extract import extract_images, extract_poses


def main():
    directory=Path(__file__).parent/"build/public"
    housing=directory/"housing-slice.mcap"
    started=time.perf_counter()
    batch=extract_images(housing,topic="/camera/depth/colorized")
    native_ms=1000*(time.perf_counter()-started)
    started=time.perf_counter()
    with housing.open("rb") as source:
        decoded=list(make_reader(source,validate_crcs=True,decoder_factories=[DecoderFactory()])
                     .iter_decoded_messages(log_time_order=False))
    python_ms=1000*(time.perf_counter()-started)
    assert len(decoded)==len(batch.log_time_ns)==1
    _,channel,message,image=decoded[0]
    assert batch.log_time_ns[0]==message.log_time and batch.publish_time_ns[0]==message.publish_time
    assert batch.header_time_ns[0]==image.header.stamp.sec*10**9+image.header.stamp.nanosec
    assert (batch.width[0],batch.height[0],batch.step[0],batch.encoding,batch.frame_id)==(image.width,image.height,image.step,image.encoding,image.header.frame_id)
    assert batch.is_bigendian[0]==image.is_bigendian
    np.testing.assert_array_equal(batch.data,np.frombuffer(image.data,np.uint8))
    source_provenance=json.loads(housing.with_suffix('.json').read_text())
    result={"housing":{"provenance":source_provenance,"width":int(batch.width[0]),"height":int(batch.height[0]),
                       "encoding":batch.encoding,"frame_id":batch.frame_id,"header_time_ns":int(batch.header_time_ns[0]),
                       "pixel_sha256":hashlib.sha256(batch.data.tobytes()).hexdigest(),"parity":"exact timestamps, headers, dimensions and every byte",
                       "native_first_total_ms":native_ms,"native_components_ms":batch.timings_ms,"python_decoder_ms":python_ms}}
    cdr=directory/"rosbag2-cdr.mcap"
    with cdr.open("rb") as source:
        decoded=list(make_reader(source,validate_crcs=True,decoder_factories=[DecoderFactory()]).iter_decoded_messages())
    result["rosbag2_cdr"]={"decoded_messages":len(decoded),"schema_names":sorted({s.name for s,_,_,_ in decoded})}
    try:extract_poses(cdr,topic="/test_topic")
    except Exception as error:result["rosbag2_cdr"]["native_rejection"]=str(error).splitlines()[-1]
    bagel=directory/"bagel-tour.mcap"
    try:
        with bagel.open("rb") as source:
            next(make_reader(source,decoder_factories=[DecoderFactory()]).iter_decoded_messages(topics=["/camera/image_raw"]))
        raise AssertionError("expected recorded source schema limitation")
    except NotImplementedError as error:result["bagel"]={"python_rejection":str(error)}
    try:extract_images(bagel,topic="/camera/image_raw")
    except Exception as error:result["bagel"]["native_rejection"]=str(error).splitlines()[-1]
    output=Path(__file__).parent/"public_evidence.json"
    output.write_text(json.dumps(result,indent=2)+"\n");print(json.dumps(result,indent=2))


if __name__=="__main__":main()
