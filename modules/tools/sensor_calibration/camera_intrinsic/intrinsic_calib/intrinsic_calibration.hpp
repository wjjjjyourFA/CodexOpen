#pragma once

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <iomanip>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "cyber/common/file.h"
#include "modules/tools/sensor_calibration/common/yaml_writer.h"
#include "modules/tools/sensor_calibration/camera_intrinsic/intrinsic_calib/auto_image_picker.hpp"
#include "modules/tools/sensor_calibration/camera_intrinsic/intrinsic_calib/config/runtime_config.h"

struct IntrinsicCalibrationHyperparams {
  // 标定板参数设置 mm
  // 注意只能用于求 内参数
  // length of one side of a small square
  int grid_size = 50;  // ==> square_size

  // board_size: number of inner corners (cols, rows)
  // 标定板板内的角点数量
  // Size(width, height)  列数在前，行数在后
  cv::Size board_size = cv::Size(15, 17);  // ==> pattern_size | corner_size

  // number of pictures used for calibration
  int reference_img_num = 15;

  int minpts = 10;
};

class IntrinsicCalibration {
 public:
  IntrinsicCalibration() {
    // camera_dist_ = cv::Mat::zeros(5, 1, CV_64F);  // 强制5个畸变参数
    // camera_intrinsic_ = cv::Mat::eye(3, 3, CV_64F);
  }
  ~IntrinsicCalibration() {}

  bool Init(const std::string& img_dir_path,
            const int& grid_size    = 50,  // in milimeter 50mm
            const int& corner_width = 15, const int& corner_height = 17);

  bool Init(std::shared_ptr<jojo::tools::RuntimeConfig> param);

  virtual bool Calibrate();

  // bool addSingleImage(const std::string &img_path);

  bool undistortSingleImage(const std::string& image_path,
                            const std::string& output_image_path);

  bool getCameraIntrinsicParam(cv::Mat& camera_intrinsic,
                               cv::Mat& camera_dist) {
    camera_intrinsic = camera_intrinsic_.clone();
    camera_dist      = camera_dist_.clone();
    return true;
  }

  bool getCameraIntrinsicParam(
      std::vector<std::vector<double>>& camera_intrinsic,
      std::vector<double> camera_dist) {
    camera_intrinsic.clear();
    camera_dist.clear();
    for (int i = 0; i < 3; i++) {
      camera_intrinsic.push_back(std::vector<double>());
      camera_intrinsic_.row(i).copyTo(camera_intrinsic[i]);
    }
    camera_dist_.col(0).copyTo(camera_dist);
    return true;
  }

  double getCameraFx() { return camera_intrinsic_.at<double>(0, 0); }
  double getCameraFy() { return camera_intrinsic_.at<double>(1, 1); }

  void SaveCalibrationResult();

 protected:
  virtual bool SetImageFiles(const std::string& img_dir_path);

  std::shared_ptr<jojo::tools::RuntimeConfig> param_;

  IntrinsicCalibrationHyperparams hps_;

  AutoImagePicker image_selector;

  // input image path
  std::string img_dir_path_;
  // output undistorted image path
  std::string undistort_image_path_;
  std::string selected_image_path_;
  std::string detected_image_path_;
  std::string calib_result_file_;

  // input image names
  std::vector<std::string> file_names;
  std::vector<std::string> selected_file_names;

  // intrinsic params
  cv::Mat camera_intrinsic_;  // cameraMatrix
  cv::Mat camera_dist_;  // distCoeffs
  cv::Size img_size_;
  // auto pinhole intrinsic params
  cv::Mat new_camera_intrinsic_;

  // save 3D position of corners in each chessboard
  std::vector<std::vector<cv::Point3f>> object_points_;
  // save 2D pixel position of corners in each chessboard
  std::vector<std::vector<cv::Point2f>> image_points_;

  // extrinsic rotation and translation
  // camera or chessboard?
  std::vector<cv::Mat> R_mats_, t_mats_;  // ==> rvecs, tvecs
  cv::Mat map1, map2;

  // option param
  // int calibration_option_;
  // bool init_undistort_;

 protected:
  // bool checkReprojectionError() {};
  virtual bool undistortImages(const std::vector<std::string>& image_names);

  void addPoints(const std::vector<cv::Point2f>& image_corners,
                 const std::vector<cv::Point3f>& object_corners) {
    image_points_.push_back(image_corners);
    object_points_.push_back(object_corners);
  }

  virtual bool InitCalibrationResult();

  virtual void FilterPts(
      const std::vector<std::vector<cv::Point3f>>& object_points,
      const std::vector<std::vector<cv::Point2f>>& image_points,
      std::vector<std::vector<cv::Point3f>>& obj_pts_good,
      std::vector<std::vector<cv::Point2f>>& img_pts_good,
      double err_threshold = 1.0);
};