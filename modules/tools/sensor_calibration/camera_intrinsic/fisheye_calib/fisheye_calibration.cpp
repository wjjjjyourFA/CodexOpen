#include "modules/tools/sensor_calibration/camera_intrinsic/fisheye_calib/fisheye_calibration.h"

namespace tools = jojo::tools;
namespace common = apollo::cyber::common;

FisheyeCalibration::FisheyeCalibration() {
  camera_intrinsic_ = cv::Mat::zeros(3, 3, CV_64F);
  camera_dist_      = cv::Mat::zeros(4, 1, CV_64F);
}

bool FisheyeCalibration::Calibrate() {
  selected_file_names.clear();

  std::vector<cv::Point3f> object_corners;
  object_corners.reserve(hps_.board_size.width * hps_.board_size.height);
  // generate 3D corner points for 单应性矩阵
  // Z = 0 的 标定板坐标系的 三维平面点
  for (int i = 0; i < hps_.board_size.height; i++) {
    for (int j = 0; j < hps_.board_size.width; j++) {
      object_corners.emplace_back(
          cv::Point3f(j * hps_.grid_size, i * hps_.grid_size, 0.0f));
    }
  }

  // pick image
  cv::Mat init_img = cv::imread(img_dir_path_ + file_names[0], 0);
  img_size_        = init_img.size();

  image_selector.init(img_size_.width, img_size_.height, hps_.board_size.width,
                      hps_.board_size.height);

  // detect chessboard corner
  int total_image_num = file_names.size();
  // successfully corner-detected image num
  int detected_image_num = 0;
  for (size_t i = 0; i < file_names.size(); i++) {
    cv::Mat input_image;
    // 灰度读取
    // input_image = cv::imread(img_dir_path_ + file_names[i], 0);
    // 彩色读取
    input_image = cv::imread(img_dir_path_ + file_names[i], cv::IMREAD_COLOR);
    if (input_image.empty()) continue;
    // img_size_ = input_image.size();

    cv::Mat gray;
    cv::cvtColor(input_image, gray, cv::COLOR_BGR2GRAY);

    // detect corner
    std::vector<cv::Point2f> image_corners;
    bool whether_found = cv::findChessboardCorners(
        gray, hps_.board_size, image_corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    // refining pixel coordinates for given 2d points
    if (whether_found) {
      // whether select this image for calibration
      if (image_selector.addImage(image_corners)) {
        selected_file_names.push_back(file_names[i]);
        std::cout << "Select image --- " << file_names[i] << std::endl;

        // cv::imwrite(selected_image_path_ + file_names[i],
        //             cv::imread(img_dir_path_ + file_names[i]));
        cv::imwrite(selected_image_path_ + file_names[i], input_image);

        static cv::TermCriteria criteria(
            cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);

        // cornerSubPix 要求 单通道灰度图。
        cv::cornerSubPix(gray, image_corners, cv::Size(5, 5), cv::Size(-1, -1),
                         criteria);

        this->addPoints(image_corners, object_corners);
        detected_image_num++;

        // display corner detect
        cv::drawChessboardCorners(input_image, hps_.board_size, image_corners,
                                  whether_found);

        cv::imwrite(detected_image_path_ + file_names[i], input_image);
        // cv::imshow(file_names[i], input_image);

        std::string name = "Corners";
        cv::namedWindow(name, cv::WINDOW_NORMAL);
        cv::resizeWindow(name, 960, 540);
        cv::imshow(name, input_image);
        cv::waitKey(50);
      }
      if (image_selector.status()) break;
    }
  }
  cv::destroyAllWindows();

  std::cout << "Calculating camera parameters..." << std::endl;

  // check min valid num of images
  if (detected_image_num < hps_.reference_img_num) {
    std::cerr << "[WARNING]Detected image num less than minium requirement.\n";
  }

  if (object_points_.size() < hps_.minpts) {
    std::cout
        << "Not enough valid images for calibration. Need at least 10, got "
        << object_points_.size() << std::endl;
    // return false;
  }

  std::cout << "object_points :  " << object_points_.size()
            << " . image_points : " << image_points_.size() << std::endl;

  this->InitCalibrationResult();

  int flags1 = 0;
  // flags1 |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
  flags1 |= cv::fisheye::CALIB_FIX_SKEW;  // 不要尝试估计 skew
  // 鱼眼相机模型，参数耦合，必须给 主点 自由度
  // flags1 |= cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;
  // flags1 |= cv::fisheye::CALIB_FIX_FOCAL_LENGTH;
  // flags1 |= cv::fisheye::CALIB_FIX_K3 | cv::fisheye::CALIB_FIX_K4;
  flags1 |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;

  // calibrate the camera
  rvecs_.clear();
  tvecs_.clear();
  double rms = cv::fisheye::calibrate(object_points_, image_points_, img_size_,
                                      camera_intrinsic_, camera_dist_, rvecs_,
                                      tvecs_, flags1);
  std::cout << "[Stage 1] RMS: " << rms << std::endl;

  // 剔除异常图像 重新收集合格点
  std::vector<std::vector<cv::Point3f>> obj_pts_good;
  std::vector<std::vector<cv::Point2f>> img_pts_good;
  this->FilterPts(object_points_, image_points_, obj_pts_good, img_pts_good);
  // std::cout << "After filtero . bject_points :  " << obj_pts_good.size()
  //           << " . image_points : " << img_pts_good.size() << std::endl;

  int flags2 = 0;
  flags2 |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
  flags2 |= cv::fisheye::CALIB_FIX_SKEW;
  // flags2 |= cv::fisheye::CALIB_FIX_FOCAL_LENGTH;
  flags2 |= cv::fisheye::CALIB_FIX_K3 | cv::fisheye::CALIB_FIX_K4;

  rvecs_.clear();
  tvecs_.clear();
  rms = cv::fisheye::calibrate(obj_pts_good, img_pts_good, img_size_,
                               camera_intrinsic_, camera_dist_, rvecs_, tvecs_,
                               flags2);
  std::cout << "[Stage 2] RMS: " << rms << std::endl;

  // 鱼眼相机 这里很容易被全过滤，因此不进行二阶段的过滤
  // this->FilterPts(obj_pts_good, img_pts_good, obj_pts_good, img_pts_good, 0.8);

  int flags3 = 0;
  flags3 |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
  flags3 |= cv::fisheye::CALIB_FIX_SKEW;

  for (int iter = 0; iter < 3; ++iter) {
    rvecs_.clear();
    tvecs_.clear();
    rms = cv::fisheye::calibrate(obj_pts_good, img_pts_good, img_size_,
                                 camera_intrinsic_, camera_dist_, rvecs_,
                                 tvecs_, flags3);
  }
  std::cout << "[Stage 3] RMS: " << rms << std::endl;

  // print result
  // std::cout << "\n==>CALIBRATION RESULT\n";
  std::cout << "Input " << total_image_num << " images\n";
  std::cout << "Detected " << detected_image_num << " images\n";
  std::cout << "\ncamera intrinsic: \n";
  std::cout << camera_intrinsic_.at<double>(0, 0) << "\t"
            << camera_intrinsic_.at<double>(0, 1) << "\t"
            << camera_intrinsic_.at<double>(0, 2) << std::endl;
  std::cout << camera_intrinsic_.at<double>(1, 0) << "\t"
            << camera_intrinsic_.at<double>(1, 1) << "\t"
            << camera_intrinsic_.at<double>(1, 2) << std::endl;
  std::cout << camera_intrinsic_.at<double>(2, 0) << "\t"
            << camera_intrinsic_.at<double>(2, 1) << "\t"
            << camera_intrinsic_.at<double>(2, 2) << std::endl;
  std::cout << "\ndist: \n";
  for (int i = 0; i < camera_dist_.rows; i++) {
    for (int j = 0; j < camera_dist_.cols; j++) {
      std::cout << camera_dist_.at<double>(i, j) << " ";
    }
  }
  std::cout << "\n" << std::endl;

  // undistort and save images
  undistortImages(selected_file_names);

  return true;
}

bool FisheyeCalibration::undistortImages(
    const std::vector<std::string>& image_names) {
  common::CreateDir(undistort_image_path_);

  // 严格
  cv::fisheye::initUndistortRectifyMap(camera_intrinsic_, camera_dist_,
                                       cv::Mat(), camera_intrinsic_, img_size_,
                                       CV_16SC2, map1, map2);

  // 调优
  // cv::initUndistortRectifyMap(camera_intrinsic_, camera_dist_, cv::Mat(),
  //                             new_camera_intrinsic_, img_size_, CV_32FC1, map1,
  //                             map2);

  for (int i = 0; i < image_names.size(); i++) {
    cv::Mat input_image          = cv::imread(img_dir_path_ + image_names[i]);
    std::string output_file_path = undistort_image_path_ + image_names[i];

    cv::Mat undistorted_image;
    cv::remap(input_image, undistorted_image, map1, map2, cv::INTER_LINEAR);

    cv::imwrite(output_file_path, undistorted_image);
  }

  return true;
}

bool FisheyeCalibration::InitCalibrationResult() {
  // 1. 初值设定，按板格和像素近似计算 fx/fy
  // mm -> m
  double fx_guess =
      img_size_.width * 0.5 / (hps_.board_size.width * hps_.grid_size * 0.001);
  double fy_guess = fx_guess;
  // MATLAB 风格
  // double fx_guess = 0.8 * img_size_.width;
  // double fy_guess = fx_guess;
  double cx_guess = img_size_.width * 0.5;
  double cy_guess = img_size_.height * 0.5;

  // clang-format off
  // camera_intrinsic_ = cv::Mat::eye(3, 3, CV_64F);
  camera_intrinsic_ = (cv::Mat_<double>(3, 3) << fx_guess, 0, cx_guess, 
                                                 0, fy_guess, cy_guess, 
                                                 0, 0, 1);
  // k1,k2,k3,k4
  camera_dist_ = cv::Mat::zeros(4, 1, CV_64F);
  // clang-format on

  return true;
}

void FisheyeCalibration::FilterPts(
    const std::vector<std::vector<cv::Point3f>>& object_points,
    const std::vector<std::vector<cv::Point2f>>& image_points,
    std::vector<std::vector<cv::Point3f>>& obj_pts_good,
    std::vector<std::vector<cv::Point2f>>& img_pts_good, double err_threshold) {
  std::vector<std::vector<cv::Point3f>> obj_pts;
  std::vector<std::vector<cv::Point2f>> img_pts;

  for (size_t i = 0; i < image_points.size(); ++i) {
    // 基本一致性检查
    if (object_points[i].empty() || image_points[i].empty() ||
        object_points[i].size() != image_points[i].size()) {
      continue;
    }

    std::vector<cv::Point2f> projected;
    cv::fisheye::projectPoints(object_points[i], projected, rvecs_[i],
                               tvecs_[i], camera_intrinsic_, camera_dist_);

    // 计算重投影误差
    double err = 0;
    for (size_t j = 0; j < projected.size(); ++j) {
      err += cv::norm(projected[j] - image_points[i][j]);
    }
    err /= projected.size();

    // threshold in pixels
    if (err < err_threshold) {  // MATLAB 常用 0.5 ~ 1.0 px
      obj_pts.push_back(object_points[i]);
      img_pts.push_back(image_points[i]);
    }
  }

  obj_pts_good = obj_pts;
  img_pts_good = img_pts;
}

bool FisheyeCalibration::isSupportedImageFormat(const std::string& extension) {
  std::string lower_ext = extension;
  std::transform(lower_ext.begin(), lower_ext.end(), lower_ext.begin(),
                 ::tolower);
  return std::find(supported_extensions.begin(), supported_extensions.end(),
                   lower_ext) != supported_extensions.end();
}