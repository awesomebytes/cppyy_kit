"""Repository example: segment an image and detect an official AprilTag fixture."""
import argparse
import hashlib
import json
from time import perf_counter

import cv2
import numpy as np

from segment import ASSETS, EdgeSAM, apriltag_detector

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("image")
parser.add_argument("--point", nargs=2, type=float)
args = parser.parse_args()
image = cv2.imread(args.image)
if image is None:
    raise FileNotFoundError(args.image)
point = args.point or [image.shape[1] / 2, image.shape[0] / 2]
model = EdgeSAM()
start = perf_counter()
model.set_image(image)
encode_ms = (perf_counter() - start) * 1000
start = perf_counter()
mask, score = model.predict([point])
decode_ms = (perf_counter() - start) * 1000
output = image.copy()
output[mask] = (output[mask] * 0.5 + np.array([0, 255, 0]) * 0.5).astype(np.uint8)
cv2.imwrite(str(ASSETS / "segmented.jpg"), output)
cv2.imwrite(str(ASSETS / "mask.png"), mask.astype(np.uint8) * 255)
tag = cv2.imread(str(ASSETS / "tag36_11_00000.png"), cv2.IMREAD_GRAYSCALE)
tag = cv2.resize(tag, (400, 400), interpolation=cv2.INTER_NEAREST)
tag = cv2.copyMakeBorder(tag, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
cv2.imwrite(str(ASSETS / "printable-tag.png"), tag)
detector = apriltag_detector()
start = perf_counter()
detections = detector.detect(tag)
tag_ms = (perf_counter() - start) * 1000
result = {
    "image": args.image, "point_xy": point,
    "backend": model.encoder.get_providers(),
    "encoder_first_ms": encode_ms, "decoder_first_ms": decode_ms,
    "mask_pixels": int(mask.sum()), "model_score": score,
    "apriltag_ids": [d.tag_id for d in detections], "apriltag_ms": tag_ms,
    "sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in ASSETS.glob("*.onnx")},
}
(ASSETS / "result.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
