"""
Responsible for camera tracking
"""

import numpy as np
import cv2


class Tracker:
    __slots__ = ("camera", "latest_image", "latest_position", "blob_detector")

    def __init__(self):
        self.camera = cv2.VideoCapture(0)
        self.latest_image = None
        self.latest_position = None
        params = cv2.SimpleBlobDetector.Params()
        params.minThreshold = 200
        params.maxThreshold = 250
        self.blob_detector = cv2.SimpleBlobDetector.create(params)

    def update(self):
        ok, frame = self.camera.read()
        if not ok:
            return False

        self.latest_image = frame

        keypoints = self.blob_detector.detect(self.latest_image)

        if not keypoints:
            return False

        # TODO: real logic
        self.latest_position = keypoints[0]

        return True

    def get_image(self):
        return self.latest_image

    def get_position(self):
        return self.latest_position
