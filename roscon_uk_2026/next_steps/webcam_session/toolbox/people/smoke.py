"""Run prepared models on a saved image without opening the webcam."""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from adapters import People
from fetch import CACHE, ROOT

parser = argparse.ArgumentParser()
parser.add_argument('--image', type=Path, default=ROOT / 'build/webcam_session/rotation_0.jpg')
args = parser.parse_args()
image = cv2.imread(str(args.image))
if image is None:
    raise ValueError(f'Cannot read {args.image}')
cv2.setNumThreads(2)
report = {'image': str(args.image), 'shape': list(image.shape), 'opencv': cv2.__version__, 'operations': {}}
for operation in ('face', 'hand', 'pose', 'segmentation'):
    start = time.perf_counter()
    model = People(operation)
    output = model(image)
    first_ms = (time.perf_counter() - start) * 1000
    timings = []
    for _ in range(10):
        start = time.perf_counter()
        output = model(image)
        timings.append((time.perf_counter() - start) * 1000)
    report['operations'][operation] = {
        'load_and_first_ms': first_ms,
        'warm_median_ms': float(np.median(timings)),
        'warm_p95_ms': float(np.percentile(timings, 95)),
        'calls': len(timings),
        'outputs': len(output),
    }
    print(operation, report['operations'][operation], flush=True)
(CACHE / 'smoke.json').write_text(json.dumps(report, indent=2) + '\n')
