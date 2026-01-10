"""
Responsible for camera tracking
"""

from typing import NamedTuple, Sequence
import numpy as np
import cv2

DEBUG = False


class TrackerKeypoints(NamedTuple):
    po: cv2.KeyPoint
    px: cv2.KeyPoint
    py: cv2.KeyPoint


class WorldTransform(NamedTuple):
    x: float
    y: float
    theta: float


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


def farthest_pair(points: Sequence[cv2.KeyPoint]) -> tuple[int, int]:
    indices = None
    max_dist = 0
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            (pix, piy) = points[i].pt
            (pjx, pjy) = points[j].pt
            dist_squared = (pix - pjx) ** 2 + (piy - pjy) ** 2
            if dist_squared > max_dist:
                max_dist = dist_squared
                indices = (i, j)

    assert indices is not None

    return indices


def keypoints_to_tracker_keypoints(
    keypoints: Sequence[cv2.KeyPoint],
) -> TrackerKeypoints:
    """
    Given 3 keypoints, we want to find:

    py
    |
    |
    po---px
    """

    # find farthest pair
    # this is the pair of px py
    # so po is the other one
    indices = farthest_pair(keypoints)

    po_i = (0 + 1 + 2) - (indices[0] + indices[1])
    po = keypoints[po_i].pt

    pa = keypoints[indices[0]].pt
    pb = keypoints[indices[1]].pt

    # Now take (pa - po) cross (pb - po) to order px py
    po_pa = (pa[0] - po[0], pa[1] - po[1])
    po_pb = (pb[0] - po[0], pb[1] - po[1])
    cross_product = po_pa[0] * po_pb[1] - po_pa[1] * po_pb[0]

    # because image coordinate system, cross product is negative <=> pa == px
    px_i, py_i = indices if cross_product < 0 else (indices[1], indices[0])

    px = keypoints[px_i]
    py = keypoints[py_i]

    return TrackerKeypoints(keypoints[po_i], px, py)


def get_world_scale(tracker_keypoints: TrackerKeypoints) -> tuple[float, float]:
    vx = np.array(
        [
            tracker_keypoints.px.pt[0] - tracker_keypoints.po.pt[0],
            tracker_keypoints.px.pt[1] - tracker_keypoints.po.pt[1],
        ]
    )
    vy = np.array(
        [
            tracker_keypoints.py.pt[0] - tracker_keypoints.po.pt[0],
            tracker_keypoints.py.pt[1] - tracker_keypoints.po.pt[1],
        ]
    )

    # Find the scaling for x and y that would make vx and vy unit length

    a = np.vstack([vx**2, vy**2])
    b = np.ones(2)

    scales_squared = np.linalg.solve(a, b)
    return np.sqrt(scales_squared)


def point_to_world_transform(
    point: np.typing.ArrayLike, world_scale: tuple[int, int], warp_size: tuple[int, int]
) -> np.typing.ArrayLike:
    return (point - (np.array(warp_size) / 2)) * world_scale * (1, -1)


def tracker_keypoints_to_world_transform(
    tracker_keypoints: TrackerKeypoints, warp_size: tuple[int, int]
) -> WorldTransform:
    world_scale = get_world_scale(tracker_keypoints)

    po_img = np.array(tracker_keypoints.po.pt)
    px_img = np.array(tracker_keypoints.px.pt)
    py_img = np.array(tracker_keypoints.py.pt)

    po_world = point_to_world_transform(po_img, world_scale, warp_size)
    px_world = point_to_world_transform(px_img, world_scale, warp_size)
    py_world = point_to_world_transform(py_img, world_scale, warp_size)

    vx = px_world - po_world
    vy = py_world - po_world

    theta_px = np.atan2(vx[0], vx[1])
    theta_py = np.atan2(vy[0], vy[1]) - (np.pi / 2)

    theta = (theta_px + theta_py) / 2

    return WorldTransform(x=po_world[0], y=po_world[1], theta=theta)


def make_detector():
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200

    # Filter by Area.
    params.filterByArea = False
    params.minArea = 10

    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.87

    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    return detector


class Tracker:
    __slots__ = (
        "camera",
        "latest_image",
        "latest_position",
        "blob_detector",
        "warp_matrix",
        "warp_size",
    )

    def __init__(self):
        if not DEBUG:
            self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
            # self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            # self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.latest_image = None
        self.latest_position = None

        self.blob_detector = make_detector()

        self.warp_size = (500, 500)
        self.set_warp([299, 101], [681, 102], [700, 364], [263, 356])

    def update(self):
        if DEBUG:
            # frame = cv2.imread("calibration_images/image.jpg")
            # frame = cv2.imread("image.jpg")
            frame = cv2.imread("image.jpg", cv2.IMREAD_GRAYSCALE)
        else:
            ok, frame = self.camera.read()
            if not ok:
                return False
            frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)

        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.warpPerspective(
            frame, self.warp_matrix, self.warp_size, flags=cv2.INTER_LINEAR
        )
        self.latest_image = frame
        keypoints = self.blob_detector.detect(frame)

        if not keypoints:
            return False

        if len(keypoints) != 3:
            return False

        tracker_keypoints = keypoints_to_tracker_keypoints(keypoints)
        world_transform = tracker_keypoints_to_world_transform(
            tracker_keypoints, self.warp_size
        )

        self.latest_position = world_transform

        return True

    def get_image(self):
        return self.latest_image

    def get_position(self):
        return self.latest_position

    def set_warp(self, p1, p2, p3, p4):
        in_points = np.float32([p1, p2, p3, p4])
        out_points = np.float32(
            [
                [0, 0],
                [self.warp_size[0], 0],
                [self.warp_size[0], self.warp_size[1]],
                [0, self.warp_size[1]],
            ]
        )

        self.warp_matrix = cv2.getPerspectiveTransform(in_points, out_points)
