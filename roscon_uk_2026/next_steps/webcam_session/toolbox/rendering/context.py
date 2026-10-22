"""Print live-demo instructions and local inventory without opening devices."""
from importlib import metadata, util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[5]
ASSETS = (
    'build/webcam_session/calibration_refined.json',
    'build/webcam_session/face_landmarker.task',
    'roscon_uk_2026/next_steps/webcam_session/models/shape_predictor_68_face_landmarks.dat',
    'build/pipeline/models/*.task',
    'build/perception_toolbox/models/*.pt',
    'build/perception_toolbox/models/*.ts',
    'build/perception_toolbox/segmentation/*.onnx',
    'build/perception_toolbox/huggingface/hub/models--depth-anything--*/snapshots/*/config.json',
)


def main():
    print(Path(__file__).with_name('AGENT_CONTEXT.md').read_text())
    print('\n## Current command environment (verify the live kernel separately)')
    print(f'Python: {sys.version.split()[0]}\nExecutable: {sys.executable}\nRepository: {ROOT}')
    for name in ('cppyy_kit', 'cppyy', 'marimo', 'viser', 'dlib', 'torch', 'mediapipe', 'cv2'):
        spec = util.find_spec(name)
        print(f'{name}: {spec.origin if spec else "not import-discoverable here"}')
    for name in ('torch', 'dlib', 'marimo', 'viser', 'numpy'):
        try:
            print(f'{name} version: {metadata.version(name)}')
        except metadata.PackageNotFoundError:
            pass
    print('\n## Existing local assets (presence, not an inference check)')
    for pattern in ASSETS:
        matches = sorted(ROOT.glob(pattern))
        if not matches:
            print(f'Not found: {pattern}')
        for path in matches:
            print(f'{path.relative_to(ROOT)} ({path.stat().st_size} bytes)')


if __name__ == '__main__':
    main()
