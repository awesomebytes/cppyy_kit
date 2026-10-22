"""Retained GPU models. Inputs are contiguous uint8 RGB arrays."""
from pathlib import Path

import cv2
import numpy as np
import torch
from depth_anything_3.api import DepthAnything3
from ultralytics import YOLOE
from ultralytics.nn.text_model import MobileCLIPTS

ROOT = Path(__file__).resolve().parents[5]
MODEL_DIR = ROOT / "build/perception_toolbox/models"


class DepthModel:
    """DA3-SMALL relative depth; returned grid corresponds to processed_image."""

    def __init__(self, device="cuda", resolution=336):
        self.model = DepthAnything3.from_pretrained(
            "depth-anything/DA3-SMALL",
            revision="e08cab65ca0ec38e7826075418411ab90cab4da3"
        ).to(device)
        self.resolution = resolution

    def __call__(self, rgb):
        prediction = self.model.inference([rgb], process_res=self.resolution)
        return {
            "depth": prediction.depth[0],
            "confidence": prediction.conf[0],
            "intrinsics": prediction.intrinsics[0],
            "rgb": prediction.processed_images[0],
            "depth_kind": "relative_estimate",
        }


class ObjectModel:
    """YOLOE small; change text classes once, then reuse the model each frame."""

    def __init__(self, classes=("person", "cup", "bottle"), device="cuda"):
        MODEL_DIR.mkdir(parents=True, exist_ok=True)
        self.model = YOLOE(str(MODEL_DIR / "yoloe-11s-seg.pt")).to(device)
        self.model.model.clip_model = MobileCLIPTS(
            device=torch.device(device), weight=str(MODEL_DIR / "mobileclip_blt.ts")
        )
        self.set_classes(classes)

    def set_classes(self, classes):
        names = list(classes)
        self.model.set_classes(names, self.model.model.get_text_pe(names, cache_clip_model=True))

    def __call__(self, rgb, track=False):
        operation = self.model.track if track else self.model.predict
        options = {"persist": True, "tracker": "bytetrack.yaml"} if track else {}
        result = operation(
            cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), imgsz=640,
            retina_masks=True, verbose=False, **options,
        )[0]
        masks = (
            result.masks.data.cpu().numpy().astype(bool)
            if result.masks is not None
            else np.empty((0, *rgb.shape[:2]), dtype=bool)
        )
        return {
            "boxes": result.boxes.xyxy.cpu().numpy(),
            "track_ids": (result.boxes.id.cpu().numpy().astype(int)
                          if result.boxes.id is not None else None),
            "scores": result.boxes.conf.cpu().numpy(),
            "labels": [result.names[int(i)] for i in result.boxes.cls],
            "masks": masks,
            "overlay_rgb": result.plot()[..., ::-1].copy(),
        }
