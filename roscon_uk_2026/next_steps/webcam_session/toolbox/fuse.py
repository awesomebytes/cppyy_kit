"""Estimate an object's selected 3D points from one saved webcam image."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from models import DepthModel, ObjectModel
from roscon_uk_2026.next_steps.webcam_session.geometry import masked_depth_points

ROOT = Path(__file__).resolve().parents[4]
parser = argparse.ArgumentParser()
parser.add_argument('--image', type=Path, default=ROOT / 'build/webcam_session/rotation_0.jpg')
parser.add_argument('--object', default='person')
args = parser.parse_args()
bgr = cv2.imread(str(args.image))
if bgr is None:
    raise ValueError(f'Cannot read {args.image}')
depth_model = DepthModel()
object_model = ObjectModel([args.object])
depth = depth_model(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
# Detect on the processed RGB grid to keep masks, K and depth aligned.
objects = object_model(depth['rgb'])
mask = np.any(objects['masks'], axis=0) if len(objects['masks']) else np.zeros_like(depth['depth'], dtype=bool)
points, pixels = masked_depth_points(depth['depth'], mask, depth['intrinsics'])
output = ROOT / 'build/perception_toolbox/evidence'
output.mkdir(parents=True, exist_ok=True)
np.savez_compressed(output / 'selected_cloud.npz', xyz=points, pixels=pixels,
                    depth=depth['depth'], mask=mask, intrinsics=depth['intrinsics'],
                    rgb=depth['rgb'])
report = {'input': str(args.image), 'object': args.object,
          'detections': len(objects['masks']), 'points': len(points),
          'depth_kind': depth['depth_kind'], 'grid': list(depth['depth'].shape),
          'units': 'relative estimated scale, not metres'}
(output / 'selected_cloud.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
