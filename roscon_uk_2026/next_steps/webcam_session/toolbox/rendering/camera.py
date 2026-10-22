"""Open the Linux webcam with a fixed frame-rate request."""
import fcntl
import struct
import warnings

import cv2

VIDIOC_S_CTRL = 0xC008561C
V4L2_CID_EXPOSURE_AUTO_PRIORITY = 0x009A0903


def open_camera(device=0, fps=30):
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f'Cannot open camera {device}')
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    capture.set(cv2.CAP_PROP_FPS, fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    # Keep automatic exposure, but prevent it from reducing the capture rate.
    # This hardware control remains set after closing the device.
    try:
        with open(f'/dev/video{device}', 'rb+', buffering=0) as handle:
            fcntl.ioctl(handle, VIDIOC_S_CTRL,
                        struct.pack('Ii', V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0))
    except OSError as error:
        warnings.warn(f'Variable exposure frame rate could not be disabled: {error}')
    return capture
