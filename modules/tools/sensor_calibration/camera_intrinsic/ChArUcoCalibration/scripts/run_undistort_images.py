import os
import sys
import numpy as np
import cv2
import cv2.aruco
import json
from typing import Generator, Tuple, Optional

from calibration import FisheyeCalibrator, PinholeCalibrator


def pinhole_undist_main():
    data_root_path = "/workspaces/CodexOpen/install/bin/data/SensorCalibration/IntrinsicCalib"

    instaCamPinhole = PinholeCalibrator(
        ARUCO_DICT, SQUARES_VERTICALLY, SQUARES_HORIZONTALLY, SQUARE_LENGTH, MARKER_LENGTH,
        root_path=data_root_path
    )

    instaCamPinhole.load_camera_parameters(
        file_path=os.path.join(data_root_path, 'pinhole_calibration.json'))

    for image_name, image in instaCamPinhole.raw_images.items():
        instaCamPinhole.undistort_image(image, image_name, calibration_filename = 'pinhole_calibration.json', balance = 1, show_image = False, save_image = True)


def fisheye_undist_main():
    data_root_path = "/workspaces/CodexOpen/install/bin/data/SensorCalibration/FisheyeCalibration"

    instaCamFisheye = FisheyeCalibrator(
        ARUCO_DICT, SQUARES_VERTICALLY, SQUARES_HORIZONTALLY, SQUARE_LENGTH, MARKER_LENGTH,
        root_path=data_root_path
    )

    instaCamFisheye.load_camera_parameters(
        file_path=os.path.join(data_root_path, 'fisheye_calibration.json'))

    for image_name, image in instaCamFisheye.raw_images.items():
        instaCamFisheye.undistort_image(image, image_name, calibration_filename = 'fisheye_calibration.json', balance = 1, show_image = False, save_image = True)


if __name__ == '__main__':
    print('Running undistort script')

    ARUCO_DICT = cv2.aruco.DICT_5X5_100
    SQUARES_VERTICALLY = 12
    SQUARES_HORIZONTALLY = 9
    SQUARE_LENGTH = 0.06
    MARKER_LENGTH = 0.045
    CURRENT_PATH = os.path.dirname(os.path.abspath(__file__))

    # pinhole_undist_main()

    fisheye_undist_main()
