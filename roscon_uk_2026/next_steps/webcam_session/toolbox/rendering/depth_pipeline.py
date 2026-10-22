"""One DA3 prediction supplies aligned RGB, relative depth and a colored cloud."""
import json
from pathlib import Path
import time

import cv2
import numpy as np

from models import DepthModel
from roscon_uk_2026.next_steps.webcam_session.geometry import masked_depth_points


class DepthPipeline:
    def __init__(self, resolution=336, calibration=None):
        self.model = DepthModel(resolution=resolution)
        self.calibration = None
        if calibration is not None:
            data = json.loads(Path(calibration).read_text())
            self.calibration = np.array(data['camera_matrix'], dtype=np.float64)
            self.image_size = tuple(data['image_size'])
            self.maps = cv2.initUndistortRectifyMap(
                self.calibration, np.array(data['distortion']), None,
                self.calibration, self.image_size, cv2.CV_32FC1)

    def __call__(self, frame, state, controls):
        start = time.perf_counter()
        if self.calibration is not None:
            if (frame.shape[1], frame.shape[0]) != self.image_size:
                raise ValueError(f'Calibration requires image size {self.image_size}')
            frame = cv2.remap(frame, *self.maps, interpolation=cv2.INTER_LINEAR)
        prediction = self.model(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        inference_ms = (time.perf_counter()-start)*1000
        depth = prediction['depth']
        rgb = prediction['rgb']
        if self.calibration is not None:
            # DA3's default single-image upper_bound_resize has no crop.
            intrinsics = self.calibration.copy()
            intrinsics[0] *= depth.shape[1] / self.image_size[0]
            intrinsics[1] *= depth.shape[0] / self.image_size[1]
            prediction['intrinsics'] = intrinsics
        mask = np.isfinite(depth) & (depth > 0)
        stride = max(1, int(controls.get('stride', 2)))
        selected = np.zeros_like(mask)
        selected[::stride, ::stride] = mask[::stride, ::stride]
        points, pixels = masked_depth_points(depth, selected, prediction['intrinsics'], far=1e6)
        finite = depth[mask]
        limits = np.percentile(finite, [2,98]) if finite.size else [0,1]
        scaled = np.nan_to_num((depth-limits[0])/max(limits[1]-limits[0], 1e-6))
        depth_image = cv2.applyColorMap(np.uint8(np.clip(scaled,0,1)*255),cv2.COLORMAP_TURBO)
        return dict(image=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR), depth_image=depth_image,
                    points=points, colors=rgb.reshape(-1,3)[pixels].copy(),
                    depth=depth, intrinsics=prediction['intrinsics'],
                    inference_ms=inference_ms, geometry_and_color_ms=(time.perf_counter()-start)*1000-inference_ms,
                    depth_kind='relative estimate; not metres')
