"""Independent decoder parity, known fields and adversarial supported-layout inputs."""
from io import BytesIO
from dataclasses import replace
import struct

import numpy as np
import pytest
from mcap.reader import make_reader
from mcap.writer import Writer, CompressionType
from mcap_ros2.decoder import DecoderFactory
from mcap_ros2._cdr import CdrWriter, EncapsulationKind

from extract import generate_fixture, extract_poses, extract_images, extract_image_headers, count_speed_above
from schemas import POSE


@pytest.fixture(scope="module")
def recording(tmp_path_factory):
    path = tmp_path_factory.mktemp("typed") / "episode.mcap"
    generate_fixture(path)
    return path


def test_python_decoder_and_known_truth(recording):
    for topic in ["/pose", "/truth"]:
        batch = extract_poses(recording, topic)
        with open(recording, "rb") as source:
            messages = list(make_reader(source, validate_crcs=True, decoder_factories=[DecoderFactory()])
                            .iter_decoded_messages(topics=[topic], log_time_order=False))
        np.testing.assert_array_equal(batch.log_time_ns, [m.log_time for _, _, m, _ in messages])
        np.testing.assert_array_equal(batch.publish_time_ns, [m.publish_time for _, _, m, _ in messages])
        np.testing.assert_array_equal(batch.header_time_ns, [d.header.stamp.sec*10**9+d.header.stamp.nanosec for _,_,_,d in messages])
        np.testing.assert_array_equal(batch.position_m, [[d.pose.position.x,d.pose.position.y,d.pose.position.z] for _,_,_,d in messages])
        np.testing.assert_array_equal(batch.quaternion_xyzw, [[d.pose.orientation.x,d.pose.orientation.y,d.pose.orientation.z,d.pose.orientation.w] for _,_,_,d in messages])
        assert batch.frame_id == "map"
    truth = extract_poses(recording, "/truth")
    t = np.arange(600)*.01
    np.testing.assert_array_equal(truth.position_m, np.column_stack([.2*t,.1*np.sin(t),.03*np.cos(2*t)]))
    np.testing.assert_array_equal(truth.log_time_ns-truth.publish_time_ns, np.full(600,2_000_000))
    observed = extract_poses(recording)
    speed = np.linalg.norm(np.diff(observed.position_m,axis=0),axis=1) / (np.diff(observed.log_time_ns).astype(float)*1e-9)
    assert count_speed_above(observed,.3) == int(np.count_nonzero(speed > .3))


def test_image_pixels_headers_and_selection(recording):
    images = extract_images(recording)
    headers = extract_image_headers(recording)
    assert len(images.log_time_ns) == 50
    assert len(headers.data) == 0
    np.testing.assert_array_equal(headers.header_time_ns, images.header_time_ns)
    with open(recording, "rb") as source:
        decoded = list(make_reader(source, decoder_factories=[DecoderFactory()])
                       .iter_decoded_messages(topics=["/camera/image"], log_time_order=False))
    for i, (_,_,message,data) in enumerate(decoded):
        pixels = images.data[int(images.offsets[i]):int(images.offsets[i+1])]
        np.testing.assert_array_equal(pixels,np.frombuffer(data.data,np.uint8))
        assert (images.width[i],images.height[i],images.step[i],images.encoding)==(16,12,48,"rgb8")
        one = extract_images(recording,start_ns=message.log_time,end_ns=message.log_time+1)
        np.testing.assert_array_equal(one.data,pixels)
    assert not len(extract_poses(recording,start_ns=0,end_ns=1).log_time_ns)
    assert len(extract_poses(recording,start_ns=int(images.log_time_ns[0]),end_ns=int(images.log_time_ns[0])+10_000_000).log_time_ns)==1
    with pytest.raises(Exception,match="missing topic"):
        extract_poses(recording,"/absent")


def cdr_pose(big=False):
    stream=BytesIO(); c=CdrWriter(stream, EncapsulationKind.CDR_BE if big else EncapsulationKind.CDR_LE)
    c.write_int32(-1); c.write_uint32(123); c.write_string("map")
    for value in [1.,2.,3.,0.,0.,0.,1.]: c.write_float64(value)
    return stream.getvalue()


def raw_recording(path, payload=None, definition=POSE, name="geometry_msgs/msg/PoseStamped", times=(10,), compression=CompressionType.NONE):
    with path.open("wb") as file:
        writer=Writer(file,compression=compression);writer.start(profile="ros2")
        schema=writer.register_schema(name,"ros2msg",definition.encode())
        channel=writer.register_channel("/pose","cdr",schema)
        for t in times: writer.add_message(channel,t,cdr_pose() if payload is None else payload,publish_time=t)
        writer.finish()
    return path


def test_endian_duplicates_and_order(tmp_path):
    for big in [False,True]:
        batch=extract_poses(raw_recording(tmp_path/"endian.mcap",cdr_pose(big),times=(10,10,11)))
        np.testing.assert_array_equal(batch.position_m,[[1,2,3]]*3)
        np.testing.assert_array_equal(batch.header_time_ns,[-999999877]*3)
        np.testing.assert_array_equal(batch.log_time_ns,[10,10,11])
    with pytest.raises(Exception,match="decreasing"):
        extract_poses(raw_recording(tmp_path/"order.mcap",times=(11,10)))


@pytest.mark.parametrize("payload,reason",[(cdr_pose()[:-1],"truncated"),(bytes([0,3,0,0])+cdr_pose()[4:],"CDR v1"),
                                           (cdr_pose()+b'\0',"trailing"),
                                           (cdr_pose()[:8]+struct.pack('<I',10**9)+cdr_pose()[12:],"nanosec")])
def test_corrupt_cdr(tmp_path,payload,reason):
    with pytest.raises(Exception,match=reason): extract_poses(raw_recording(tmp_path/"bad.mcap",payload))


def test_schema_compression_and_corrupt_container(tmp_path):
    for name,definition,reason in [("std_msgs/msg/String",POSE,"unsupported"),
                                   ("geometry_msgs/msg/PoseStamped",POSE.replace("float64 x","float32 x"),"layout")]:
        with pytest.raises(Exception,match=reason): extract_poses(raw_recording(tmp_path/"schema.mcap",name=name,definition=definition))
    with pytest.raises(Exception,match="compressed chunks"):
        extract_poses(raw_recording(tmp_path/"compressed.mcap",compression=CompressionType.ZSTD))
    corrupt=tmp_path/"short.mcap";corrupt.write_bytes(b"broken")
    with pytest.raises(Exception,match="MCAP"):
        extract_poses(corrupt)
    complete=raw_recording(tmp_path/"complete.mcap")
    corrupt.write_bytes(complete.read_bytes()[:-12])
    with pytest.raises(Exception,match="MCAP"):
        extract_poses(corrupt)


def test_speed_empty_alignment_and_timestamp_shape(recording):
    batch=extract_poses(recording)
    empty=replace(batch,log_time_ns=np.empty(0,np.uint64),position_m=np.empty((0,3)))
    assert count_speed_above(empty)==0
    timestamps=np.ndarray(batch.log_time_ns.shape,dtype=np.uint64,buffer=bytearray(batch.log_time_ns.nbytes+1),offset=1)
    positions=np.ndarray(batch.position_m.shape,dtype=np.float64,buffer=bytearray(batch.position_m.nbytes+1),offset=1)
    timestamps[:]=batch.log_time_ns;positions[:]=batch.position_m
    assert not positions.flags.aligned and not timestamps.flags.aligned
    assert count_speed_above(replace(batch,log_time_ns=timestamps,position_m=positions))==count_speed_above(batch)
    with pytest.raises(ValueError,match="uint64"):
        count_speed_above(replace(batch,log_time_ns=batch.log_time_ns.reshape(-1,1)))
    huge=replace(batch,log_time_ns=np.array([0,10**9],np.uint64),position_m=np.array([[0.,0.,0.],[1e200,0.,0.]]))
    assert count_speed_above(huge,2e200)==0
    huge=replace(batch,log_time_ns=np.array([0,10**19],np.uint64),position_m=np.array([[-1e308,0.,0.],[1e308,0.,0.]]))
    assert count_speed_above(huge,3e298)==0
