#!/usr/bin/python3
import numpy as np
import cv2
import os
from picamera2 import Picamera2
from libcamera import controls

# Toggle image viewing
VISUALIZE = 0

# Resolution
RES_RAW = (2328, 1748)
RES_1080 = (1920, 1080)
RES_720 = (1280, 720)
RES = RES_RAW


def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder, filename))
        if img is not None:
            images.append(img)
    return images


if VISUALIZE:
    cv2.startWindowThread()

# Create picam stream
# picam2 = Picamera2()
# picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": RES_720}))
# picam2.start()
# picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 3.0})  # Focus distance 1/3 [m]

# Create aruco dictionary, board, detection params and initialize matrices
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
charuco_board = cv2.aruco.CharucoBoard_create(5, 7, 0.035, 0.0175, aruco_dict)
aruco_params = cv2.aruco.DetectorParameters_create()
first_cal = True
cam_mat = np.zeros((3, 3), dtype=np.float32)
dist_coeffs = np.zeros(4, dtype=np.float32)

all_cal_frames = load_images_from_folder("calib_frames/{}/".format(RES[1]))

print("Detecting Charuco board...")
all_corners = []
all_ids = []
decimator = 0

# Sub pixel corner detection criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
for frame in all_cal_frames:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

    if len(corners) > 0:
        # Sub pixel detection
        for corner in corners:
            cv2.cornerSubPix(gray, corner,
                             winSize=(3, 3),
                             zeroZone=(-1, -1),
                             criteria=criteria)
        res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, charuco_board)
        if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator % 1 == 0:
            all_corners.append(res2[1])
            all_ids.append(res2[2])
    decimator += 1

print("Calibrating...")
cameraMatrixInit = np.array([[1000., 0., RES[0]/2.],
                             [0., 1000., RES[1]/2.],
                             [0., 0., 1.]])

distCoeffsInit = np.zeros((5, 1))
flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
#flags = (cv2.CALIB_RATIONAL_MODEL)
(ret, camera_matrix, distortion_coefficients0,
 rotation_vectors, translation_vectors,
 stdDeviationsIntrinsics, stdDeviationsExtrinsics,
 perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                  charucoCorners=all_corners,
                  charucoIds=all_ids,
                  board=charuco_board,
                  imageSize=RES,
                  cameraMatrix=cameraMatrixInit,
                  distCoeffs=distCoeffsInit,
                  flags=flags,
                  criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

print("Calibration completed")
np.save("cam_mat_{}.npy".format(RES[1]), camera_matrix)
np.save("dist_coeffs_{}.npy".format(RES[1]), distortion_coefficients0)
with open("std_dev_int_{}.csv".format(RES[1]), "ab") as f:
    f.write(b"\n")
    np.savetxt(f, stdDeviationsIntrinsics, delimiter=',')
with open("pve_{}.csv".format(RES[1]), "ab") as f:
    f.write(b"\n")
    np.savetxt(f, perViewErrors, delimiter=',')

