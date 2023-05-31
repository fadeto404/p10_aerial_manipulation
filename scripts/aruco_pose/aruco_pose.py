#!/usr/bin/python3
import numpy as np

import time
import cv2
import serial

from picamera2 import Picamera2
from libcamera import controls

# Toggle image viewing
VISUALIZE = 1

# Resolutions
RES_RAW = (2328, 1748)
RES_1080 = (1920, 1080)
RES_720 = (1280, 720)
RES = RES_RAW


def rvec2quat(rot_vec):
    """
    Convert rotation vector (compact axis-angle representation) to unit quaternion
    :param rot_vec: ndarray
    :return quat:
    """
    theta = np.linalg.norm(rot_vec)
    rot_vec = rot_vec / theta
    quat = np.ones(4, dtype=np.float32)
    quat[0:3] = np.sin(0.5 * theta) * rot_vec
    quat[3] = np.cos(0.5 * theta)
    quat *= 1 / np.linalg.norm(quat)
    # quat = np.append(np.sin(0.5*theta)*rot_vec, np.cos(0.5*theta), 0)
    return quat


def print_serial_data(ser, num_measurements):
    start = time.time()
    while ser.in_waiting < num_measurements * 28:
        continue
    mid = time.time()
    print(mid - start)
    received = ser.read(num_measurements * 28)
    end = time.time()
    print(end - start)
    print(np.frombuffer(received, np.float32))

def print_pose_measurement(id, quat, tvec):
    print("Marker: {}".format(id))
    print("Quaternion: ")
    print(quat)
    print("Norm: {}".format(np.linalg.norm(quat)))
    print("Translation: ")
    print(tvec)


def main():
    if VISUALIZE:
        cv2.startWindowThread()

    # Setup serial communication with teensy
    ser = serial.Serial('/dev/ttyACM0', 1000000)

    # Create picam stream
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": RES}, queue=False))
    picam2.start()
    picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 4.0})  # Focus distance 1/3 [m]

    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters_create()

    marker_width = 0.053  # Side length of markers [m]

    object_pts = np.asarray([[-marker_width / 2, marker_width / 2, 0],
                             [marker_width / 2, marker_width / 2, 0],
                             [marker_width / 2, -marker_width / 2, 0],
                             [-marker_width / 2, -marker_width / 2, 0]])

    rvec = np.zeros(3, dtype=np.float32)
    tvec = np.zeros(3, dtype=np.float32)

    # Load calibration coefficients
    cam_mat = np.load("cam_mat_{}.npy".format(RES[1]))
    dist_coeffs = np.load("dist_coeffs_{}.npy".format(RES[1]))

    while True:
        # Get image
        in_img = picam2.capture_array()
        (corners, ids, rejects) = cv2.aruco.detectMarkers(in_img, aruco_dict, parameters=aruco_params)
        if ids is not None:
            num_measurements = ids.shape[0]
            measurements = np.zeros((7 * num_measurements), dtype=np.float32)
            for i in range(0, num_measurements):
                # cv2.solvePnP(object_pts, corners[i], cam_mat, dist_coeffs, rvec, tvec)
                (rvec, tvec, obj_pts) = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_width, cam_mat,
                                                                            dist_coeffs, rvec, tvec, object_pts)
                quat = rvec2quat(rvec)
                tvec_new = np.reshape(tvec.astype(np.float32), 3)
                measurements[i * 7:(i + 1) * 7] = np.concatenate([quat, tvec_new], 0)
                if VISUALIZE:
                    cv2.drawFrameAxes(in_img, cam_mat, dist_coeffs, rvec, tvec, 0.1)
                # print_pose_measurement(ids[i], quat, tvec_new)

            # Construct byte string of measurements and send
            bin_data = num_measurements.to_bytes(1, "big") + measurements.tobytes()
            ser.write(bin_data)
            ser.flush()
            # print_serial_data(ser, num_measurements)

            if VISUALIZE:
                cv2.aruco.drawDetectedMarkers(in_img, corners, ids, borderColor=(0, 255, 0))

        if VISUALIZE:
            cv2.imshow("Camera", in_img)
            key = cv2.waitKey(1)
            if key == 27:
                cv2.imwrite("img.jpg", in_img)
                cv2.destroyWindow("Camera")
                break


if __name__ == '__main__':
    main()
