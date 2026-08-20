#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "modules/tools/sensor_calibration/camera_intrinsic/intrinsic_calib/intrinsic_calibration.hpp"

class FisheyeCalibration : public IntrinsicCalibration {
 public:
  FisheyeCalibration();
  ~FisheyeCalibration() {}

  bool Calibrate() override;

  std::vector<std::string> getSupportedFormats() const {
    return supported_extensions;
  }

 protected:
  std::vector<cv::Vec3d> rvecs_, tvecs_;

 private:
  bool undistortImages(const std::vector<std::string>& image_names) override;

  bool InitCalibrationResult() override;

  void FilterPts(const std::vector<std::vector<cv::Point3f>>& object_points,
                 const std::vector<std::vector<cv::Point2f>>& image_points,
                 std::vector<std::vector<cv::Point3f>>& obj_pts_good,
                 std::vector<std::vector<cv::Point2f>>& img_pts_good,
                 double err_threshold = 1.0) override;

  // 支持的图片格式
  std::vector<std::string> supported_extensions = {".jpg", ".jpeg", ".png",
                                                   ".bmp", ".tiff", ".tif"};

  bool isSupportedImageFormat(const std::string& extension);
};
