#!/usr/bin/python3
import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import controls

# Toggle image viewing
VISUALIZE = 1

# Resolution
RES_RAW = (2328, 1748)
RES_1080 = (1920, 1080)
RES_720 = (1280, 720)
RES = RES_RAW

if VISUALIZE:
    cv2.startWindowThread()

# Create picam stream
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": RES}))
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 4.0})  # Focus distance 1/3 [m]

# Create aruco dictionary, board, detection params and initialize matrices
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
charuco_board = cv2.aruco.CharucoBoard_create(5, 7, 0.035, 0.0175, aruco_dict)
aruco_params = cv2.aruco.DetectorParameters_create()
i = 0

while True:
    # Get image
    in_img = picam2.capture_array()
    out_img = np.copy(in_img)
    (corners, ids, rejects) = cv2.aruco.detectMarkers(in_img, aruco_dict, parameters=aruco_params)

    # If any markers were found:
    if ids is not None:
        (ret, charuco_corners, charuco_ids) = cv2.aruco.interpolateCornersCharuco(corners, ids, in_img, charuco_board)

        if VISUALIZE:
            cv2.aruco.drawDetectedMarkers(out_img, corners, ids, borderColor=(0, 255, 0))
            if charuco_ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(out_img, charuco_corners, charuco_ids, cornerColor=(255, 0, 0))

    if VISUALIZE:
        cv2.imshow("Camera", out_img)
        key = cv2.waitKey(50)
        if key == 27:
            cv2.imwrite("calib_frames/{}/im{}.jpg".format(RES[1], i), in_img)
            i += 1
            print("Saved frame")


