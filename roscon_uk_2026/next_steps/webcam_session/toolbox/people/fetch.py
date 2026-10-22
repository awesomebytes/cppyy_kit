"""Download the selected official OpenCV Zoo models, verifying Git LFS hashes."""
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.request

ROOT = Path(__file__).resolve().parents[5]
CACHE = ROOT / 'build/perception_toolbox/people'
ZOO = CACHE / 'opencv_zoo'
REVISION = '48b10fa505aeb51fbd127fc60a5c660040e823b7'
MODELS = {
    'face': 'face_detection_yunet/face_detection_yunet_2023mar.onnx',
    'palm': 'palm_detection_mediapipe/palm_detection_mediapipe_2023feb.onnx',
    'hand': 'handpose_estimation_mediapipe/handpose_estimation_mediapipe_2023feb.onnx',
    'person': 'person_detection_mediapipe/person_detection_mediapipe_2023mar.onnx',
    'pose': 'pose_estimation_mediapipe/pose_estimation_mediapipe_2023mar.onnx',
    'segmentation': 'human_segmentation_pphumanseg/human_segmentation_pphumanseg_2023mar.onnx',
}


def fetch():
    records = {}
    for name, relative in MODELS.items():
        pointer = subprocess.check_output(
            ['git', '-C', str(ZOO), 'show', f'{REVISION}:models/{relative}'], text=True)
        expected = pointer.split('oid sha256:')[1].splitlines()[0]
        path = ZOO / 'models' / relative
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            url = f'https://media.githubusercontent.com/media/opencv/opencv_zoo/{REVISION}/models/{relative}'
            urllib.request.urlretrieve(url, path)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != expected:
            raise ValueError(f'Hash mismatch: {path}')
        records[name] = {'path': str(path.relative_to(ROOT)), 'sha256': digest, 'bytes': path.stat().st_size}
        print(name, path.stat().st_size, flush=True)
    (CACHE / 'assets.json').write_text(json.dumps(records, indent=2) + '\n')


if __name__ == '__main__':
    fetch()
