"""Retained CPU people models. Images are uint8 BGR; outputs use image pixels."""
from pathlib import Path
import sys

import cv2

from fetch import MODELS, ZOO

for relative in MODELS.values():
    sys.path.insert(0, str(ZOO / 'models' / Path(relative).parent))

from mp_handpose import MPHandPose
from mp_palmdet import MPPalmDet
from mp_persondet import MPPersonDet
from mp_pose import MPPose
from pphumanseg import PPHumanSeg


class People:
    """Load one operation once, then call it on successive frames."""

    def __init__(self, operation='face'):
        self.operation = operation
        paths = {key: str(ZOO / 'models' / value) for key, value in MODELS.items()}
        if operation == 'face':
            self.detector = cv2.FaceDetectorYN.create(paths['face'], '', (320, 320))
        elif operation == 'hand':
            self.detector = MPPalmDet(paths['palm'])
            self.landmarks = MPHandPose(paths['hand'])
        elif operation == 'pose':
            self.detector = MPPersonDet(paths['person'])
            self.landmarks = MPPose(paths['pose'])
        elif operation == 'segmentation':
            self.detector = PPHumanSeg(paths['segmentation'])
        else:
            raise ValueError('Choose face, hand, pose or segmentation')

    def __call__(self, bgr):
        if self.operation == 'face':
            self.detector.setInputSize((bgr.shape[1], bgr.shape[0]))
            _, faces = self.detector.detect(bgr)
            return [] if faces is None else faces
        if self.operation == 'segmentation':
            return self.detector.infer(bgr)
        return [result for item in self.detector.infer(bgr)
                if (result := self.landmarks.infer(bgr, item)) is not None]
