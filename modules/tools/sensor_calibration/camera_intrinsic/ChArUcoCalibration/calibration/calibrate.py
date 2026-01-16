import os
import numpy as np
import cv2
import cv2.aruco
import json
from typing import Generator, Tuple, Optional


class CharucoCalibrator:
    def __init__(self, aruco_dict: int, squares_horizontally: int, squares_vertically: int, square_length: float, marker_length:float, root_path:str = None):
        """
        Initializes the CharucoCalibrator with the specified parameters.

        Args:
            aruco_dict (int): The ArUco dictionary to use.
            squares_vertically (int): Number of squares vertically in the ChArUco board.
            squares_horizontally (int): Number of squares horizontally in the ChArUco board.
            square_length (float): Length of each square in the ChArUco board.
            marker_length (float): Length of each ArUco marker in the ChArUco board.
            calibration_images_dir (str, optional): Directory containing calibration images.
            raw_images_dir (str, optional): Directory containing raw images to be processed.
        """
        # 选择字典类型
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.squares_horizontally = squares_horizontally
        self.squares_vertically = squares_vertically
        self.square_length = square_length
        self.marker_length = marker_length

        # self.board = cv2.aruco.CharucoBoard((self.squares_vertically, self.squares_horizontally), self.square_length, self.marker_length, self.aruco_dict)
        # opencv 4.6
        self.board = cv2.aruco.CharucoBoard_create(self.squares_horizontally, self.squares_vertically, self.square_length, self.marker_length, self.aruco_dict)

        # 创建检测参数
        # self.params = cv2.aruco.DetectorParameters()
        # opencv 4.6
        self.params = cv2.aruco.DetectorParameters_create()

        # self.cur_dir = os.path.dirname(os.path.abspath(__file__))
        self.cur_dir = root_path
        # 用于检查 去畸变 效果
        self.raw_image_dir = os.path.join(root_path, 'selected')
        # 标定用的图片
        self.calibration_image_dir = os.path.join(root_path, 'selected')
        # self.calibration_image_dir = os.path.join(root_path, 'calib_images')
        self.calib_file_path = ""

        self.K = np.zeros((3, 3))
        self.D = None

    def image_generator(self, folder_path) -> Generator[Tuple[str, np.ndarray], None, None]:
        """
        Generates NumPy arrays of images from the specified directory in sorted order.

        Args:
            folder_path (str): The path to the directory containing images.

        Yields:
            Tuple[str, np.ndarray]: A tuple containing the filename and the image as a NumPy array.
        """
        image_paths = [os.path.join(folder_path, f) for f in sorted(os.listdir(folder_path)) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.tiff'))]
        for img_path in image_paths:
            try:
                yield os.path.basename(img_path), cv2.imread(img_path)
            except Exception as e:
                print(f"Error loading image {img_path}: {e}")

    def calibration_images(self) -> Generator[Tuple[str, np.ndarray], None, None]:
        """
        Generator function that yields images from the calibration images directory.

        Yields:
            Tuple[str, np.ndarray]: A tuple containing the filename and the image as a NumPy array.
        """
        """
        不需要一次性加载所有图片到内存；
        可以边读边处理；
        适合大规模数据集、视频帧、批处理。
        """
        yield from self.image_generator(self.calibration_image_dir)

    def raw_images(self) -> Generator[Tuple[str, np.ndarray], None, None]:
        """
        Generator function that yields images from the raw images directory.

        Yields:
            Tuple[str, np.ndarray]: A tuple containing the filename and the image as a NumPy array.
        """
        yield from self.image_generator(self.raw_image_dir)

    def generate_blank_board(self) -> None:
        """
        Creates a blank black board and saves it as data/blank_board.png.
        """
        size_ratio = self.squares_horizontally / self.squares_vertically
        img_width = 640
        img_height = int(640 * size_ratio)

        blank_board = np.zeros((img_height, img_width, 3), dtype=np.uint8)

        # output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/calibration/blank_board.png')
        output_path = os.path.join(self.cur_dir, 'blank_board.png')

        cv2.imwrite(output_path, blank_board)
        print(f"Blank board saved to {output_path}")

    def generate_charuco_board(self) -> None:
        """
        This function creates a ChArUco board and saves it data/charuco_board.png.
        """
        size_ratio = self.squares_vertically / self.squares_horizontally

        # opencv 3.x
        # charuco_img = cv2.aruco.CharucoBoard.generateImage(self.board, (640, int(640*size_ratio)), marginSize=20)

        # opencv 4.6
        img_width = 640
        img_height = int(round(img_width * size_ratio))

        normal_img = np.ones((img_height, img_width), dtype=np.uint8) * 255
        square_length_px = img_width / self.squares_horizontally
        for y in range(self.squares_vertically):
            for x in range(self.squares_horizontally):
                # 检查是否要画黑格
                if (x + y) % 2 == 0:
                    x1 = int(round(x * square_length_px))
                    y1 = int(round(y * square_length_px))
                    x2 = int(round((x + 1) * square_length_px))
                    y2 = int(round((y + 1) * square_length_px))
                    # 防止越界
                    x2 = min(x2, img_width)
                    y2 = min(y2, img_height)
                    normal_img[y1:y2, x1:x2] = 0  # 黑格子
        # cv2.imshow("normal_img.png", normal_img)

        charuco_img = cv2.aruco.drawPlanarBoard(self.board, (img_width, img_height), None, marginSize=7, borderBits=1)
        # cv2.imshow("charuco_board.png", charuco_img)

        assert charuco_img.shape == normal_img.shape

        # 直接把 normal_img 的黑格保留，charuco_img 的 marker 叠加生成 mask，把 marker 区域识别出来（非白色部分）
        marker_mask = charuco_img < 255  # marker 区域为 True

        combined_img = normal_img.copy()
        # marker 区域覆盖到 normal_img
        combined_img[marker_mask] = charuco_img[marker_mask]

        cv2.imshow("combined_board.png", combined_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/calibration/charuco_board.png')
        output_path = os.path.join(self.cur_dir, 'charuco_board.png')

        cv2.imwrite(output_path, combined_img)
        print(f"ChArUco board saved to {output_path}")

    def detect_aruco_markers(self, image: np.ndarray, image_name: str = None, graysale=True, verbose=True) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Detects ArUco markers in an image.

        Args:
            image (np.ndarray): The input image as a NumPy array.
            image_name (str, optional): Name of the image for logging purposes.
            graysale (bool): Whether to convert the image to grayscale. Defaults to True.
            verbose (bool): Whether to print detection results. Defaults to True.

        Returns:
            Optional[Tuple[np.ndarray, np.ndarray]]: A tuple of (ids, corners) if markers are detected, otherwise None.
        """

        # ArUco 检测可以用灰度图，提高速度，因为颜色信息对标记检测没用。
        if graysale:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        """ 检测图像中的 ArUco 标记
        marker_corners：一个 list，每个元素是一个标记的 4 个角点坐标 [[ [x0, y0], [x1, y1], [x2, y2], [x3, y3] ]]
        marker_ids：检测到的标记 ID，对应 self.aruco_dict 里的编号。
        rejected_corners：被检测算法排除掉的候选角点。
        """
        # opencv 4.5 || 4.6
        marker_corners, marker_ids, rejected_corners = cv2.aruco.detectMarkers(image, self.aruco_dict, parameters=self.params)
        # opencv 4.7
        # detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)
        # marker_corners, marker_ids, rejected_corners = detector.detectMarkers(image)

        if marker_ids is not None and len(marker_ids) > 0:
            # 对初步检测到的标记进行精细化处理（refine）
            refined_corners, refined_ids, _, _ = cv2.aruco.refineDetectedMarkers(
                image, self.board, marker_corners, marker_ids, rejected_corners,
                parameters=self.params
            )
            # 用精细化结果替换原来的角点和 ID。
            marker_corners = refined_corners
            marker_ids = refined_ids


            if verbose:
                print(f"Step 1: {len(marker_ids)} ArUco markers detected in {image_name}")

            return marker_ids, marker_corners
        else:
            if verbose:
                print(f"Step 1: No ArUco markers detected in {image_name}")

            return None, None

    def detect_charuco_corners(self, image: np.ndarray, image_name: str = None, grayscale=True, verbose=True) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Detects ChArUco corners in an image.

        Args:
            image (np.ndarray): The input image as a NumPy array.
            image_name (str, optional): Name of the image for logging purposes.
            grayscale (bool): Whether to convert the image to grayscale. Defaults to True.
            verbose (bool): Whether to print detection results. Defaults to True.

        Returns:
            Optional[Tuple[np.ndarray, np.ndarray]]: A tuple of (charuco_ids, charuco_corners) if corners are detected, otherwise None.
        """

        marker_ids, marker_corners = self.detect_aruco_markers(image, image_name, grayscale, verbose=verbose)

        # 输入：只知道部分 ArUco 标记位置
        # 输出：完整的 ChArUco 角点网格（2D 像素坐标）
        if marker_ids is not None and len(marker_ids) > 0:
            # 在图像上绘制检测到的 ArUco 标记边框和 ID
            image_marked = cv2.aruco.drawDetectedMarkers(image.copy(), marker_corners, marker_ids)
            # 显示结果图像
            cv2.imshow("Detected ArUco Markers", image_marked)
            cv2.waitKey(1)

            # 根据 ArUco 标记的角点信息，插值计算出 ChArUco 棋盘的角点位置（Chessboard corners）。
            charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, image, self.board)

            if charuco_retval:
                num_charuco_corners = len(charuco_corners)
            else:
                num_charuco_corners = 0

            if verbose:
                print(f"Step 2 : Detected {num_charuco_corners} charuco corners in {image_name}")

            return charuco_ids, charuco_corners
        else:
            if verbose:
                print(f"Step 2 : No charuco corners detected in {image_name}")

            return None, None

    def show_aruco_markers(self, image: np.ndarray, image_name:str = None, window_size = (480,480), verbose=True) -> None:
        """
        Displays the detected ArUco markers in an image.

        Args:
            image (np.ndarray): The input image as a NumPy array.
            image_name (str, optional): Name of the image for logging purposes.
            window_size (tuple): A tuple of (width, height) for the window size. Defaults to (480, 480).
            verbose (bool): Whether to print detection results. Defaults to True.
        """
        image_copy = image.copy()

        marker_corners, marker_ids, rejected_corners = cv2.aruco.detectMarkers(image_copy, self.aruco_dict, parameters=self.params)

        if marker_ids is not None and len(marker_ids) > 0:
            # 在图像上绘制检测到的 ArUco 标记边框和 ID
            image_marked = cv2.aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)

            # 显示结果图像
            cv2.imshow("Detected ArUco Markers", cv2.resize(image_marked, window_size))
            cv2.waitKey(1)

    def show_charuco_corners(self, image: np.ndarray, image_name:str = None, window_size = (480,480),verbose = True) -> None:
        """
        Displays the detected ChArUco corners in an image.

        Args:
            image (np.ndarray): The input image as a NumPy array.
            image_name (str, optional): Name of the image for logging purposes.
            window_size (tuple): A tuple of (width, height) for the window size. Defaults to (480, 480).
            verbose (bool): Whether to print detection results. Defaults to True.
        """
        image_copy = image.copy()
        charuco_ids, charuco_corners = self.detect_charuco_corners(image_copy, image_name,verbose=verbose)
        cv2.aruco.drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids)
        cv2.imshow("Detected Charuco Corners", cv2.resize(image_copy, window_size))

        if verbose:
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def save_camera_parameters(self, file_path):
        """
        Saves camera intrinsic parameters to a JSON file.

        Args:
            file_path (str): Path to save the camera parameters.
        """

        json_data = {
            'K': self.K.tolist(),
            'D': self.D.tolist()
        }

        json.dump(json_data, open(file_path, 'w'))

    def load_camera_parameters(self, file_path: str = None) -> None:
        """
        Loads camera intrinsic parameters from a JSON file.

        Args:
            file_path (str): Path to the JSON file containing camera parameters.

        Returns:
            tuple: A tuple containing the camera matrix (K) and distortion coefficients (D).
        """
        with open(file_path, 'r') as f:
            camera_intrinsics = json.load(f)

        self.K = np.array(camera_intrinsics['K'])
        self.D = np.array(camera_intrinsics['D'])


class FisheyeCalibrator(CharucoCalibrator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def calibrate(self, grayscale = True, calibration_filename ='fisheye_calibration.json', window_size=(480,480), verbose=False) -> None:
        """
        Performs Fisheye camera calibration using ChArUco markers.

        Args:
            grayscale (bool): Whether to convert images to grayscale. Defaults to True.
            calibration_filename (str): Filename to save the calibration results. Defaults to 'fisheye_calibration.json'.
            window_size (tuple): Size of the window for displaying images. Defaults to (480, 480).
            verbose (bool): Whether to print detailed information during calibration. Defaults to False.
        """

        all_charuco_corners = []
        all_charuco_ids = []

        for image_name, image in self.calibration_images():
            charuco_ids, charuco_corners = self.detect_charuco_corners(image=image, image_name=image_name, grayscale=grayscale, verbose=verbose)

            if charuco_ids is not None and len(charuco_ids) > 4:
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
            else:
                print(f"Step 3 : Could not interpolate corners for {image_name}")
                # self.show_aruco_markers(image, image_name=image_name, window_size=window_size, verbose=verbose)

        if not all_charuco_corners:
            print("Step 4 : No charuco corners detected in any images.")
            return

        self.K = np.zeros((3, 3))
        self.D = np.zeros((4, 1))

        print("Starting Fisheye Calibration...")

        """ `cv2.aruco.calibrateCameraCharuco()` 鱼眼相机的手动实现版本
        """
        object_points = []
        image_points = []

        # opencv 3.x
        # objPoints = self.board.getChessboardCorners()
        # opencv 4.6
        objPoints = self.board.chessboardCorners

        count = 0
        for corners, ids in zip(all_charuco_corners, all_charuco_ids):
            if len(corners) >= 4:
                # 根据检测到的角点索引选择对应 3D 点
                obj_points = objPoints[ids]
                object_points.append(obj_points)
                image_points.append(corners)
                count += 1
        print('Number of images used for calibration: ', count)

        calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW

        retval, self.K, self.D, rvecs, tvecs = cv2.fisheye.calibrate(object_points, image_points, image.shape[:2], self.K, self.D, flags=calibration_flags)

        print('K: ', "\n", self.K)
        print('Distortion coefficients: ', "\n", self.D)

        # output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'../../data/calibration/camera_intrinsics/{calibration_filename}')
        output_path = os.path.join(self.cur_dir, f'{calibration_filename}')
        self.calib_file_path = output_path

        self.save_camera_parameters(output_path)
        print(f"Camera calibration data saved to {output_path}")

    def undistort_image(self, image: np.ndarray, image_name:str = None, calibration_filename = 'fisheye_calibration.json', balance = 1, show_image = True, save_image = True, output_path: str = None,window_size = (480,480)) -> np.ndarray:
        """
        Undistorts a fisheye image using the calibrated parameters.

        Args:
            image (np.ndarray): The input image to undistort.
            image_name (str, optional): Name of the image for saving purposes.
            calibration_filename (str): Filename of the calibration file. Defaults to 'fisheye_calibration.json'.
            balance (float): Balance parameter for undistortion. Defaults to 1.
            show_image (bool): Whether to display the undistorted image. Defaults to True.
            save_image (bool): Whether to save the undistorted image. Defaults to True.
            output_path (str, optional): Path to save the undistorted image.
            window_size (tuple): Size of the window for displaying images. Defaults to (480, 480).

        Returns:
            np.ndarray: The undistorted image.
        """
        try:
            # calibration_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'../../data/calibration/camera_intrinsics/{calibration_filename}')
            calibration_path = os.path.join(self.cur_dir, f'{calibration_filename}')
            self.calib_file_path = calibration_path

            self.load_camera_parameters(calibration_path)

            h, w = image.shape[:2]

            # balance = 1 时， 严格去畸变
            new_K = self.K.copy()
            new_K[0,0] *= balance  # Scale fx
            new_K[1,1] *= balance  # Scale fy

            undistorted_image = cv2.fisheye.undistortImage(image, self.K, self.D, Knew=new_K, new_size=(w,h))

            if show_image:
                cv2.imshow("Undistorted Image", cv2.resize(undistorted_image, window_size))
                cv2.waitKey(0)
                # cv2.destroyAllWindows()

            if save_image:
                if output_path is None:
                    # output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'../../data/undistorted_images/{image_name}')
                    output_path = os.path.join(self.cur_dir, f'undistorted/{image_name}')
                cv2.imwrite(output_path, undistorted_image)
                # print(f"Undistorted image saved to {output_path}")

            return undistorted_image
        except Exception as e:
            print(f"Error undistorting image {image_name}: {e}")
            return None

    def export_camera_params_colmap(self, calibration_path: str = None) -> None:
        """
        Exports camera parameters in COLMAP format.
        """
        if calibration_path is None:
            # calibration_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'../../data/calibration/camera_intrinsics/fisheye_calibration.json')
            calibration_path = self.calib_file_path

        self.load_camera_parameters(calibration_path)
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        k1 = self.D[0][0]
        k2 = self.D[1][0]
        k3 = self.D[2][0]
        k4 = self.D[3][0]
        print('Camera parameters for COLMAP:')
        print(f"OPENCV_FISHEYE: {fx}, {fy}, {cx}, {cy}, {k1}, {k2}, {k3}, {k4}")


class PinholeCalibrator(CharucoCalibrator):
    def calibrate(self, grayscale = True, calibration_filename ='pinhole_calibration.json', window_size=(480,480),verbose = False) -> None:
        """
        Performs pinhole camera calibration using ChArUco markers.

        Args:
            grayscale (bool): Whether to convert images to grayscale. Defaults to True.
            calibration_filename (str): Filename to save the calibration results. Defaults to 'pinhole_calibration.json'.
            window_size (tuple): Size of the window for displaying images. Defaults to (480, 480).
            verbose (bool): Whether to print detailed information during calibration. Defaults to False.
        """

        all_charuco_corners = []
        all_charuco_ids = []

        for image_name, image in self.calibration_images():
            charuco_ids, charuco_corners = self.detect_charuco_corners(image=image, image_name=image_name, grayscale=grayscale, verbose=verbose)
            if charuco_ids is not None and len(charuco_ids) > 4:
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
            else:
                print(f"Step 3 : Could not interpolate corners for {image_name}")
                # self.show_aruco_markers(image,image_name = image_name, window_size=window_size, verbose=verbose)

        if not all_charuco_corners:
            print("Step 4 : No charuco corners detected in any images.")
            return

        self.K = np.zeros((3, 3))
        self.D = np.zeros((5, 1))

        print("Starting Fisheye Calibration...")

        print('Number of images used for calibration: ', len(all_charuco_corners))

        """ `cv2.aruco.calibrateCameraCharuco()` 是 OpenCV 专门为 **ChArUco 棋盘板（Aruco + Chessboard 混合板）** 提供的高层标定接口。
        # 1. 自动将 `charuco_corners`（2D图像坐标）和 `charuco_ids`（角点ID）转换为 `object_points`（3D棋盘世界坐标）；
        # 2. 自动调用底层的 `cv2.calibrateCamera()` 去计算相机的内外参数。
        """
        retval, self.K, self.D, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(all_charuco_corners, all_charuco_ids, self.board, image.shape[:2], None, None, flags=0)

        print('K: ', "\n", self.K)
        print('Distortion coefficients: ', "\n", self.D)

        # output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'../../data/calibration/camera_intrinsics/{calibration_filename}')
        output_path = os.path.join(self.cur_dir, f'{calibration_filename}')
        self.calib_file_path = output_path

        self.save_camera_parameters(output_path)
        print(f"Camera calibration data saved to {output_path}")

    def undistort_image(self, image: np.ndarray, image_name:str = None, calibration_filename = 'pinhole_calibration.json', balance = 1, show_image = True, save_image = True, output_path: str = None,window_size = (480,480)) -> np.ndarray:
        """
        Undistorts a pinhole camera image using the calibrated parameters.

        Args:
            image (np.ndarray): The input image to undistort.
            image_name (str, optional): Name of the image for saving purposes.
            calibration_filename (str): Filename of the calibration file. Defaults to 'pinhole_calibration.json'.
            balance (float): Balance parameter for undistortion. Defaults to 1.
            show_image (bool): Whether to display the undistorted image. Defaults to True.
            save_image (bool): Whether to save the undistorted image. Defaults to True.
            output_path (str, optional): Path to save the undistorted image.
            window_size (tuple): Size of the window for displaying images. Defaults to (480, 480).

        Returns:
            np.ndarray: The undistorted image.
        """
        try:
            # calibration_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'../../data/calibration/camera_intrinsics/{calibration_filename}')
            calibration_path = os.path.join(self.cur_dir, f'{calibration_filename}')
            self.calib_file_path = output_path

            self.load_camera_parameters(calibration_path)

            for image_name, image in self.raw_images():

                h, w = image.shape[:2]

                # 这种方式 即使 balance = 1， 也不是严格去畸变，慎用。
                new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), balance)
                undistorted_image = cv2.undistort(image, self.K, self.D, None, new_K)

                if show_image:
                    cv2.imshow("Undistorted Image", cv2.resize(undistorted_image, window_size))
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                if save_image:
                    if output_path is None:
                        # output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'../../data/undistorted_images/{image_name}')
                        output_path = os.path.join(self.cur_dir, f'undistorted/{image_name}')
                    cv2.imwrite(output_path, undistorted_image)
                    print(f"Undistorted image saved to {output_path}")

                return undistorted_image

        except Exception as e:
            print(f"Error undistorting image {image_name}: {e}")
            return None

    def export_camera_params_colmap(self, calibration_path: str = None) -> None:
        """
        Exports camera parameters in COLMAP format.
        """
        if calibration_path is None:
            # calibration_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'../../data/calibration/camera_intrinsics/pinhole_calibration.json')
            calibration_path = self.calib_file_path

        self.load_camera_parameters(calibration_path)
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        k1 = self.D[0][0]
        k2 = self.D[0][1]
        p1 = self.D[0][2]
        p2 = self.D[0][3]
        k3 = self.D[0][4]
        print('Camera parameters for COLMAP:')
        print(f"OPENCV: {fx}, {fy}, {cx}, {cy}, {k1}, {k2}, {p1}, {p2}")


if __name__ == '__main__':
    print('Running calibration script')

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

    # instaCam.generate_charuco_board()

    instaCam.calibrate()

    # for name, image in instaCam.raw_images():
    #     instaCam.undistort_image(image, image_name=name, show_image=False)


