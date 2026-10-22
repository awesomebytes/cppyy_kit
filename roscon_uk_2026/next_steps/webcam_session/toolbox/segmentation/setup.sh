#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../../../.."
assets=build/perception_toolbox/segmentation
mkdir -p "$assets"
for part in encoder decoder; do
  name="edge_sam_3x_${part}.onnx"
  if [[ ! -f "$assets/$name" ]]; then
    curl --fail --location --retry 3 "https://huggingface.co/spaces/chongzhou/EdgeSAM/resolve/main/weights/$name" -o "$assets/$name"
  fi
done
curl --fail --location --retry 3 https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00000.png -o "$assets/tag36_11_00000.png"
sha256sum --check roscon_uk_2026/next_steps/webcam_session/toolbox/segmentation/models.sha256
pixi install -e python312 --manifest-path roscon_uk_2026/next_steps/webcam_session/toolbox/segmentation/pixi.toml
