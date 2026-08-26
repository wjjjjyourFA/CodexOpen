/*
 * Single-scene pipeline adapted from hku-mars/FAST-Calib (GPLv2).
 */

#include "fast_calib/fast_calib.h"

#include <iomanip>
#include <iostream>
#include <string>

#include <pcl/registration/transformation_estimation_svd.h>

#include "calibration_internal.h"

namespace jojo {
namespace tools {
namespace fast_calib {
namespace {

bool PrefixError(const std::string& context,
                 const std::string& detail,
                 std::string* error) {
  if (error != nullptr) {
    *error = context + ": " + detail;
  }
  return false;
}

}  // namespace

bool RunSingleCalibration(
    const Params& params,
    const cv::Mat& image,
    const pcl::PointCloud<PointXYZRing>::ConstPtr& cloud,
    LidarType lidar_type,
    SingleCalibrationResult* result,
    std::string* error) {
  if (result == nullptr) {
    return PrefixError("single-scene calibration", "result pointer is null",
                       error);
  }
  if (!EnsureOutputDirectory(params.output_path, error)) {
    return false;
  }
  if (image.empty()) {
    return PrefixError("single-scene calibration", "input image is empty",
                       error);
  }
  if (cloud == nullptr || cloud->empty()) {
    return PrefixError("single-scene calibration",
                       "input LiDAR cloud is empty", error);
  }

  SingleCalibrationResult calibration;
  QrDetector qr_detector(params);
  pcl::PointCloud<pcl::PointXYZ>::Ptr detected_camera_centers(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::string detail;
  if (!qr_detector.Detect(image, detected_camera_centers, &detail)) {
    return PrefixError("camera target detection", detail, error);
  }

  LidarDetector lidar_detector(params);
  pcl::PointCloud<pcl::PointXYZ>::Ptr detected_lidar_centers(
      new pcl::PointCloud<pcl::PointXYZ>);
  bool lidar_ok = false;
  switch (lidar_type) {
    case LidarType::kSolidState:
      lidar_ok = lidar_detector.DetectSolidState(
          cloud, detected_lidar_centers, &detail);
      break;
    case LidarType::kMechanical:
      lidar_ok = lidar_detector.DetectMechanical(
          cloud, detected_lidar_centers, &detail);
      break;
    default:
      return PrefixError("LiDAR target detection", "unknown LiDAR type",
                         error);
  }
  calibration.lidar_debug = lidar_detector.debug_clouds();
  if (!lidar_ok) {
    return PrefixError("LiDAR target detection", detail, error);
  }

  if (!SortPatternCenters(detected_camera_centers,
                          calibration.camera_centers, false, &detail)) {
    return PrefixError("camera center sorting", detail, error);
  }
  if (!SortPatternCenters(detected_lidar_centers,
                          calibration.lidar_centers, true, &detail)) {
    return PrefixError("LiDAR center sorting", detail, error);
  }
  if (!SaveTargetHoleCenters(calibration.lidar_centers,
                             calibration.camera_centers,
                             params.output_path, &detail)) {
    return PrefixError("center record", detail, error);
  }

  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
                                                  pcl::PointXYZ>
      estimator;
  estimator.estimateRigidTransformation(*calibration.lidar_centers,
                                        *calibration.camera_centers,
                                        calibration.transform);
  AlignPointCloud(calibration.lidar_centers,
                  calibration.aligned_lidar_centers,
                  calibration.transform);
  calibration.rmse = ComputeRmse(calibration.camera_centers,
                                 calibration.aligned_lidar_centers);
  if (calibration.rmse < 0.0) {
    return PrefixError("single-scene calibration", "RMSE computation failed",
                       error);
  }

  std::cout << "[Result] RMSE: " << std::fixed << std::setprecision(4)
            << calibration.rmse << " m" << std::endl;
  std::cout
      << "[Result] Single-scene calibration: extrinsic parameters "
         "T_cam_lidar =\n"
      << std::fixed << std::setprecision(6) << calibration.transform
      << std::endl;

  ProjectPointCloudToImage(cloud, calibration.transform,
                           qr_detector.camera_matrix(),
                           qr_detector.distortion_coefficients(), image,
                           calibration.colored_cloud);
  calibration.annotated_image = qr_detector.annotated_image().clone();
  if (!SaveSingleCalibrationResults(params, calibration, &detail)) {
    return PrefixError("single-scene result output", detail, error);
  }

  *result = std::move(calibration);
  return true;
}

}  // namespace fast_calib
}  // namespace tools
}  // namespace jojo
