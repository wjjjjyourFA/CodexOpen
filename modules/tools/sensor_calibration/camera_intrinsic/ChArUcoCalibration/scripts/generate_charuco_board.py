import os
import numpy as np
import cv2
import cv2.aruco
import json
from typing import Generator, Tuple, Optional

from calibration import FisheyeCalibrator, PinholeCalibrator


if __name__ == '__main__':
    ARUCO_DICT = cv2.aruco.DICT_5X5_100
    # 标定板参数
    # 黑白格数目
    SQUARES_HORIZONTALLY = 12
    SQUARES_VERTICALLY = 9
    SQUARE_LENGTH = 0.06
    MARKER_LENGTH = 0.045
    CURRENT_PATH = os.path.dirname(os.path.abspath(__file__))

    data_root_path = "/workspaces/CodexOpen/install/bin/data/SensorCalibration/FisheyeCalibration"

    instaCam = FisheyeCalibrator(
        ARUCO_DICT, SQUARES_HORIZONTALLY, SQUARES_VERTICALLY, SQUARE_LENGTH, MARKER_LENGTH,
        root_path=data_root_path
    )

    instaCam.generate_charuco_board()
