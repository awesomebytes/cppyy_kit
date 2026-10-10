"""Create an analysis-only ROS2 MCAP with recorded times inside String payloads."""
import json
from pathlib import Path
from types import SimpleNamespace
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from mcap_ros2.writer import Writer


def main():
    root = Path(__file__).resolve().parents[1]
    source = root / "data/hiw_pillow_episode_0002.mcap"
    target = root / "data/poses_replay.mcap"
    with source.open("rb") as stream, target.open("wb") as output, Writer(output) as writer:
        schema = writer.register_msgdef("std_msgs/msg/String", "string data")
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        n = 0
        for _, _, message, decoded in reader.iter_decoded_messages(topics=["/wbc_lerobot"]):
            payload = {"sequence": n, "recorded_ns": message.log_time,
                       "ee_state": json.loads(decoded.data)["ee_state"]}
            writer.write_message("/roscon/poses", schema, SimpleNamespace(data=json.dumps(payload)),
                                 log_time=message.log_time, publish_time=message.log_time, sequence=n)
            n += 1
    print({"path": str(target), "messages": n, "bytes": target.stat().st_size})


if __name__ == "__main__":
    main()
