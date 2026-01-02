import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

num1 = 9
num2 = 6

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((num2 * num1, 3), np.float32)
objp[:, :2] = np.mgrid[0:num1, 0:num2].T.reshape(-1, 2)
# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

images = glob.glob("calibration_images/*.jpg")

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (num1, num2), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        # cv.drawChessboardCorners(img, (7, 6), corners2, ret)
        # cv.imshow("img", img)
        # cv.waitKey(500)

# cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("Camera matrix : \n")
print(mtx)
print("dist : \n")
print(dist)

img = cv.imread("calibration_images/image.jpg")
h, w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

dst = cv.undistort(img, mtx, dist, None, newcameramtx)

print("New camera matrix : \n")
print(newcameramtx)

cv.imshow("undistorted", dst)

cv.waitKey(0)
