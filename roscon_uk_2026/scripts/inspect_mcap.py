"""List actual MCAP channels and decode only the selected analysis topic."""
import argparse
import json
from pathlib import Path

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def main():
    root = Path(__file__).resolve().parents[1]
    spec = json.loads((root / "DATASET.json").read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", default=str(root / "data" / spec["local_name"]))
    parser.add_argument("--topic", default="/wbc_lerobot")
    args = parser.parse_args()
    with open(args.path, "rb") as source:
        reader = make_reader(source, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()
        channels = [{"topic": c.topic, "message_encoding": c.message_encoding,
                     "schema": summary.schemas[c.schema_id].name,
                     "schema_encoding": summary.schemas[c.schema_id].encoding,
                     "count": summary.statistics.channel_message_counts.get(c.id, 0)}
                    for c in summary.channels.values()]
        print(json.dumps({"channels": channels}, indent=2))
        for schema, channel, message, decoded in reader.iter_decoded_messages(topics=[args.topic]):
            print("first selected message", schema.name, channel.topic, message.log_time)
            print(str(decoded)[:2500])
            break


if __name__ == "__main__":
    main()
