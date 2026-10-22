"""Fetch and verify the face assets used by the saved interactive notebook."""
import hashlib
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[5]
ASSETS = (
    ('face_landmarker.task',
     'https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task',
     '64184e229b263107bc2b804c6625db1341ff2bb731874b0bcc2fe6544e0bc9ff'),
    ('canonical_face_model.obj',
     'https://raw.githubusercontent.com/google-ai-edge/mediapipe/v0.10.21/mediapipe/modules/face_geometry/data/canonical_face_model.obj',
     '8bac80443397e113f41a8b565ea72c59390bc031d9defab289dba7bc0c54e618'),
)


def prepare():
    destination = ROOT / 'build/webcam_session'
    destination.mkdir(parents=True, exist_ok=True)
    for name, url, expected in ASSETS:
        path = destination / name
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            temporary = path.with_suffix(path.suffix + '.download')
            try:
                urllib.request.urlretrieve(url, temporary)
                if hashlib.sha256(temporary.read_bytes()).hexdigest() != expected:
                    raise ValueError(f'Checksum mismatch: {name}')
                temporary.replace(path)
            finally:
                temporary.unlink(missing_ok=True)
        print(f'Verified {name}')


if __name__ == '__main__':
    prepare()
