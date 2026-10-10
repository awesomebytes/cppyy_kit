# Recording for the presentation

Use one public Unitree G1 pillow-manipulation episode from
[HIW-500](https://huggingface.co/datasets/BitRobot/HIW-500).
The dataset contains teleoperated humanoid demonstrations in homes and is
released under CC BY 4.0. Attribution: BitRobot, Unitree, and Hugging Face,
*HIW-500: Humanoids In-the-Wild Dataset for Robot Learning*, 2026.
See the [project page](https://bitrobot-foundation.github.io/humanoids-in-the-wild-500-hours/).

The [download manifest](DATASET.json) pins revision, file path, size, and SHA-256.
The selected file is 173,838,627 bytes. Its sidecar reports 54.703 s. The decoded
pose stream spans 54.679 s. These are different measurements of one episode.
The original file and generated derivatives stay in ignored `data/`.

## Verified contents

These counts come from the downloaded file, not the dataset card. The complete
inspection is saved in [mcap_channels.json](evaluation/mcap_channels.json).

| Topic | Schema | Samples | Use |
|---|---|---:|---|
| `/wbc_lerobot` | `std_msgs/msg/String` | 2,735 | JSON Cartesian states and actions |
| `/camera/head/image/compressed` | `sensor_msgs/msg/CompressedImage` | 1,640 | Stereo head images |
| `/camera/left_wrist/image/compressed` | `sensor_msgs/msg/CompressedImage` | 1,640 | Left wrist images |
| `/camera/right_wrist/image/compressed` | `sensor_msgs/msg/CompressedImage` | 1,640 | Right wrist images |
| `/stamped/lowstate` | `homies/msg/LowStateStamped` | 5,470 | Optional joint-state/FK extension |
| `/lf/odommodestate` | `unitree_go/msg/SportModeState` | 1,094 | Optional base-motion analysis |
| `/annotation` | `std_msgs/msg/String` | 5 | Recorded annotations |

`ee_state` has 12 coordinates: left XYZ and rotation vector, then right XYZ and
rotation vector. This layout is described by the
[Rerun dataset example](https://github.com/rerun-io/hiw-500_demo#what-the-episodes-contain).
The query uses positions only. The inspected payload does not identify their
coordinate frame. Do not call the values world poses or infer a calibrated
grasp location. A head-camera frame at about 20 s was decoded and inspected;
it shows the grippers approaching a pillow. The image contains two stereo views.

The directory and episode task label refer to moving a pillow from floor to
sofa. The sidecar subtask strings instead mention a bed and chair. Keep that
disagreement visible. A speed heuristic does not resolve annotation quality or
prove a successful grasp.

## Download and inspect

Run from this folder:

```bash
pixi run fetch-data
pixi run inspect-data
pixi run python scripts/make_replay.py
```

[fetch_data.py](scripts/fetch_data.py) verifies the MCAP's size and SHA-256.
It requests one pinned episode directly. It does not enumerate the dataset.
The sidecar is fetched from the same pinned revision.

[make_replay.py](scripts/make_replay.py) creates a 301,606-byte derivative with
2,735 standard `String` messages on `/roscon/poses`. Each carries the original
MCAP log time, sequence number, and end-effector state. Log and publish times
are retained in the derivative. This is an analysis adapter, not an unchanged
copy of the original topic. Recorded time remains inside the payload at any
playback speed. Keep the original dataset attribution on derived material.

## Other candidate

[The humanoid IKEA assembly challenge](https://huggingface.co/datasets/BitRobot/2026-humanoid-ikea-assembly-challenge)
provides G1 furniture-assembly recordings. The first investigated MCAP was about
4.63 GB. Use it for a later manipulation/FK extension after checking that file's
schemas, robot model, and attribution. The smaller pillow episode is the
downloaded and evaluated presentation input.
