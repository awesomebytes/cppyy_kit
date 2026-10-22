"""Retained EdgeSAM encoder/decoder sessions and AprilTag 3 detection."""
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
from pupil_apriltags import Detector

ASSETS = Path(__file__).resolve().parents[5] / "build/perception_toolbox/segmentation"


class EdgeSAM:
    """Encode a BGR frame once, then query foreground/background image points."""

    def __init__(self, assets=ASSETS, threads=4):
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        self.encoder = ort.InferenceSession(str(assets / "edge_sam_3x_encoder.onnx"), options)
        self.decoder = ort.InferenceSession(str(assets / "edge_sam_3x_decoder.onnx"), options)
        self.features = None

    def set_image(self, bgr):
        self.height, self.width = bgr.shape[:2]
        scale = 1024 / max(self.height, self.width)
        self.resized = (int(self.width * scale + 0.5), int(self.height * scale + 0.5))
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        image = cv2.resize(rgb, self.resized, interpolation=cv2.INTER_LINEAR).astype(np.float32)
        image = (image - np.array([123.675, 116.28, 103.53], np.float32)) / np.array([58.395, 57.12, 57.375], np.float32)
        tensor = np.zeros((1, 3, 1024, 1024), np.float32)
        tensor[0, :, :self.resized[1], :self.resized[0]] = image.transpose(2, 0, 1)
        self.features = self.encoder.run(None, {"image": tensor})[0]

    def predict(self, points, labels=None):
        """Points are original-image (x,y); labels 1=foreground, 0=background."""
        if self.features is None:
            raise RuntimeError("Call set_image before predict")
        coords = np.asarray(points, dtype=np.float32).reshape(1, -1, 2).copy()
        coords *= np.array([self.resized[0] / self.width, self.resized[1] / self.height], np.float32)
        labels = np.ones(coords.shape[:2], np.float32) if labels is None else np.asarray(labels, np.float32).reshape(1, -1)
        scores, masks = self.decoder.run(None, {"image_embeddings": self.features, "point_coords": coords, "point_labels": labels})
        best = int(np.argmax(scores[0]))
        mask = cv2.resize(masks[0, best], (1024, 1024))[:self.resized[1], :self.resized[0]]
        return cv2.resize(mask, (self.width, self.height)) > 0, float(scores[0, best])


def apriltag_detector(threads=2):
    """Native AprilTag 3 detector; pass contiguous uint8 grayscale to detect()."""
    return Detector(families="tag36h11", nthreads=threads)
