"""Opt-in pinned public samples. No external data download during normal checks."""
import argparse
import hashlib
import json
from io import BytesIO
from pathlib import Path
import struct
import time
from urllib.request import Request, urlopen

DATASETS={
    "rosbag2-cdr":{
        "url":"https://raw.githubusercontent.com/ros2/rosbag2/e6803915796bae3a37cf836df1abb31440fb8cd3/rosbag2_tests/resources/mcap/cdr_test/cdr_test_0.mcap",
        "sha256":"deb2e9e548cb853b04c7e005eb34ecbc95034fbf1e4bf442bade6b6e7bec561f",
        "bytes":10626,"license":"Apache-2.0",
        "source":"https://github.com/ros2/rosbag2/blob/e6803915796bae3a37cf836df1abb31440fb8cd3/rosbag2_tests/resources/mcap/cdr_test/metadata.yaml",
        "license_source":"https://github.com/ros2/rosbag2/blob/e6803915796bae3a37cf836df1abb31440fb8cd3/LICENSE"},
    "bagel-tour":{
        "url":"https://raw.githubusercontent.com/Hussain004/BAGEL/ecfe27ef9b596848669e22a3d83907bec96aef7b/public/sample-bags/tour.mcap",
        "sha256":"f24b80e15dda1876accc4b3142a53779651ce50f93da607c014ca79403e15c3b",
        "bytes":3428118,"license":"MIT",
        "source":"https://github.com/Hussain004/BAGEL/blob/ecfe27ef9b596848669e22a3d83907bec96aef7b/scripts/build-sample-bag.mjs",
        "license_source":"https://github.com/Hussain004/BAGEL/blob/ecfe27ef9b596848669e22a3d83907bec96aef7b/LICENSE"},
    "housing-slice":{
        "url":"https://huggingface.co/datasets/cortexdatalabs/MCAP-Housing/resolve/1a6b8967d43febbd54ece889b248ebac5b2ad6de/Lighting%20a%20stove.mcap",
        "sha256":"ea615ff07096543a55680fa90a1108403cad49ba7c0c39924e0a185f45aec287",
        "bytes":4194304,"range":"bytes=0-4194303","source_file_bytes":1541317776,
        "source_lfs_sha256":"2f8846208b2e6817f034350e38aaf7dd2e90a5ba12fe391c579b2b14b26fcc3a",
        "license":"CC-BY-NC-4.0",
        "required_attribution":"This work uses the MCAP-Housing dataset (Cortex Data Labs, 2025).",
        "source":"https://huggingface.co/datasets/cortexdatalabs/MCAP-Housing/tree/1a6b8967d43febbd54ece889b248ebac5b2ad6de",
        "license_source":"https://huggingface.co/datasets/cortexdatalabs/MCAP-Housing/blob/1a6b8967d43febbd54ece889b248ebac5b2ad6de/README.md"}}


def housing_slice(data, target, spec):
    """Rewrap one original CDR Image without changing its fields or clocks."""
    from mcap.records import Chunk, Schema, Channel, Message
    from mcap.stream_reader import StreamReader, breakup_chunk
    from mcap.writer import Writer, CompressionType
    position, records_count = 8, 0
    while position + 9 <= len(data):
        size = struct.unpack_from("<Q", data, position+1)[0]
        if position + 9 + size > len(data):
            break
        position += 9 + size; records_count += 1
    iterator = iter(StreamReader(BytesIO(data[:position]), emit_chunks=True).records)
    schemas, channels = {}, {}
    selected = None
    topic_index = 0
    started = time.perf_counter()
    for _ in range(records_count):
        record = next(iterator)
        # Decompression is an offline source conversion, outside the native extractor.
        records = breakup_chunk(record, validate_crc=True) if isinstance(record, Chunk) else [record]
        for record in records:
            if isinstance(record, Schema): schemas[record.id] = record
            elif isinstance(record, Channel): channels[record.id] = record
            elif isinstance(record, Message):
                channel = channels[record.channel_id]
                if channel.topic == "/camera/depth/colorized":
                    topic_index += 1
                    if topic_index < 2:
                        continue
                    selected = (schemas[channel.schema_id], channel, record)
                    break
        if selected: break
    if selected is None:
        raise ValueError("fewer than two complete colorized Images in bounded prefix")
    schema, channel, message = selected
    with target.open("wb") as output:
        writer = Writer(output, compression=CompressionType.NONE)
        writer.start(profile="ros2", library="typed_mcap public slice")
        schema_id = writer.register_schema(schema.name, schema.encoding, schema.data)
        channel_id = writer.register_channel(channel.topic, channel.message_encoding, schema_id, channel.metadata)
        writer.add_message(channel_id, message.log_time, message.data, message.publish_time, message.sequence)
        writer.finish()
    provenance = dict(spec, slice_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                      slice_bytes=target.stat().st_size, original_cdr_sha256=hashlib.sha256(message.data).hexdigest(),
                      log_time_ns=message.log_time, publish_time_ns=message.publish_time,
                      converted_topic=channel.topic, messages=1, retained_topic_index=1, complete_prefix_bytes=position,
                      conversion_decompression_write_ms=1000*(time.perf_counter()-started),
                      verification="range checksum verified; full source LFS checksum reported by source, full source not downloaded")
    target.with_suffix(".json").write_text(json.dumps(provenance,indent=2)+"\n")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset",choices=DATASETS)
    parser.add_argument("--download",action="store_true")
    args=parser.parse_args();spec=DATASETS[args.dataset]
    if not args.download:
        print(json.dumps(spec,indent=2));return
    target=Path(__file__).parent/"build/public"/(args.dataset+".mcap")
    target.parent.mkdir(parents=True,exist_ok=True)
    request=Request(spec["url"],headers={"Range":spec["range"]} if "range" in spec else {})
    with urlopen(request,timeout=30) as stream:
        if "range" in spec and (stream.status!=206 or stream.headers.get("Content-Range")!=
                                 "bytes 0-4194303/1541317776"):
            raise ValueError("source did not honor the exact bounded HTTP range")
        data=stream.read(spec["bytes"]+1)
    if len(data)!=spec["bytes"] or hashlib.sha256(data).hexdigest()!=spec["sha256"]:
        raise ValueError("public download size or checksum mismatch")
    if args.dataset=="housing-slice":housing_slice(data,target,spec)
    else:target.write_bytes(data)
    print(target)


if __name__=="__main__":main()
