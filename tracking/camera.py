"""
Responsible for camera tracking
"""

import numpy as np
import cv2


# Constants from calibration routine
mtx = np.array(
    [
        [711.57891517, 0.0, 480.45785036],
        [0.0, 712.53778636, 303.15145412],
        [0.0, 0.0, 1.0],
    ]
)

dist = np.array(
    [[-2.53347121e-01, 1.15335368e-02, 1.12339388e-04, 8.78180118e-04, 2.79723320e-01]]
)

newcameramtx = np.array(
    [
        [648.00105646, 0.0, 481.45773943],
        [0.0, 648.51300997, 303.400218],
        [0.0, 0.0, 1.0],
    ]
)

class Tracker:
    __slots__ = ("camera", "latest_image", "latest_position", "blob_detector")

    def __init__(self):
        self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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


        frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)

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
