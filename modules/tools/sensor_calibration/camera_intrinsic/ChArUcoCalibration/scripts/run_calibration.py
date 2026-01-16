import os
import sys
import numpy as np
import cv2
import cv2.aruco
import json
from typing import Generator, Tuple, Optional

from calibration import FisheyeCalibrator, PinholeCalibrator


def pinhole_calib_main():
    data_root_path = "/workspaces/CodexOpen/install/bin/data/SensorCalibration/IntrinsicCalib"

    instaCamPinhole = PinholeCalibrator(
        ARUCO_DICT, SQUARES_VERTICALLY, SQUARES_HORIZONTALLY, SQUARE_LENGTH, MARKER_LENGTH,
        root_path=data_root_path
    )

    instaCamPinhole.calibrate()

    instaCamPinhole.export_camera_params_colmap()


def fisheye_calib_main():
    data_root_path = "/workspaces/CodexOpen/install/bin/data/SensorCalibration/FisheyeCalibration"

    instaCamFisheye = FisheyeCalibrator(
        ARUCO_DICT, SQUARES_VERTICALLY, SQUARES_HORIZONTALLY, SQUARE_LENGTH, MARKER_LENGTH,
        root_path=data_root_path
    )

    instaCamFisheye.calibrate()

    instaCamFisheye.export_camera_params_colmap()


if __name__ == '__main__':
    print('Running calibration script')

    ARUCO_DICT = cv2.aruco.DICT_5X5_100
    SQUARES_VERTICALLY = 12
    SQUARES_HORIZONTALLY = 9
    SQUARE_LENGTH = 0.06
    MARKER_LENGTH = 0.045
    CURRENT_PATH = os.path.dirname(os.path.abspath(__file__))

    pinhole_calib_main()

    # fisheye_calib_main()
