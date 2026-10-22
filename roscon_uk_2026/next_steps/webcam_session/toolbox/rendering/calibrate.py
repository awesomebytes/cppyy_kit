"""Capture a phone checkerboard and save webcam intrinsics with OpenCV."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from .camera import open_camera

PATTERN = (8, 5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--output', type=Path,
                        default=Path('build/webcam_session/calibration.json'))
    args = parser.parse_args()
    capture = open_camera(args.camera)
    grid = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    grid[:, :2] = np.mgrid[:PATTERN[0], :PATTERN[1]].T.reshape(-1, 2)
    observations = []
    message = 'SPACE: capture | ENTER: solve (15+ views) | Q: quit'
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError('Camera stopped returning frames')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray, PATTERN, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_FAST_CHECK)
            if found:
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
                                           30, 0.001))
                cv2.drawChessboardCorners(frame, PATTERN, corners, True)
            cv2.putText(frame, f'{len(observations)} views | {message}', (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 255, 0), 1)
            cv2.imshow('Webcam calibration', frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == 32 and found:
                observations.append(corners.copy())
                print(f'Captured view {len(observations)}. Change tilt and position.', flush=True)
            if key in (10, 13) and len(observations) >= 15:
                size = (gray.shape[1], gray.shape[0])
                rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
                    [grid] * len(observations), observations, size, None, None)
                errors = []
                for observed, rotation, translation in zip(observations, rotations, translations):
                    projected, _ = cv2.projectPoints(grid, rotation, translation, matrix, distortion)
                    errors.append(float(np.sqrt(np.mean(np.sum((observed - projected)**2, axis=2)))))
                result = dict(camera=args.camera, image_size=list(size), camera_matrix=matrix.tolist(),
                              distortion=distortion.tolist(), rms_pixels=float(rms),
                              per_view_rms_pixels=errors, views=len(observations),
                              pattern_inner_corners=list(PATTERN), square_size_units=1,
                              note='Intrinsics calibration; board translations are in square widths.')
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(result, indent=2) + '\n')
                np.savez(args.output.with_suffix('.npz'), corners=np.array(observations), grid=grid)
                print(f'Saved {args.output}; RMS {rms:.3f} pixels. Inspect per-view errors.', flush=True)
                message = f'Saved: RMS {rms:.3f}px | Q: quit'
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
