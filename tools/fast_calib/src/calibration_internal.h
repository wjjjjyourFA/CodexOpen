#ifndef TOOLS_FAST_CALIB_SRC_CALIBRATION_INTERNAL_H_
#define TOOLS_FAST_CALIB_SRC_CALIBRATION_INTERNAL_H_

#include <string>
#include <vector>

#include <opencv2/aruco.hpp>

#include "fast_calib/fast_calib.h"

namespace jojo {
namespace tools {
namespace fast_calib {

class QrDetector {
 public:
  explicit QrDetector(const Params& params);

  bool Detect(const cv::Mat& image,
              pcl::PointCloud<pcl::PointXYZ>::Ptr centers,
              std::string* error);

  const cv::Mat& annotated_image() const { return annotated_image_; }
  const cv::Mat& camera_matrix() const { return camera_matrix_; }
  const cv::Mat& distortion_coefficients() const {
    return distortion_coefficients_;
  }

 private:
  cv::Point2f ProjectPoint(const cv::Point3f& point) const;

  Params params_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Mat annotated_image_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
};

class LidarDetector {
 public:
  explicit LidarDetector(const Params& params) : params_(params) {}

  bool DetectMechanical(
      const pcl::PointCloud<PointXYZRing>::ConstPtr& cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr centers,
      std::string* error);
  bool DetectSolidState(
      const pcl::PointCloud<PointXYZRing>::ConstPtr& cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr centers,
      std::string* error);

  const LidarDebugClouds& debug_clouds() const { return debug_clouds_; }

 private:
  void ClearDebugClouds();

  Params params_;
  LidarDebugClouds debug_clouds_;
};

bool SortPatternCenters(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr output,
    bool lidar_axes,
    std::string* error);

std::vector<std::vector<int>> BuildCombinations(int count, int selection_size);

bool IsTargetRectangle(const std::vector<pcl::PointXYZ>& candidates,
                       double target_width,
                       double target_height);

void AlignPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr output,
    const Eigen::Matrix4f& transformation);

double ComputeRmse(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& first,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& second);

void ProjectPointCloudToImage(
    const pcl::PointCloud<PointXYZRing>::ConstPtr& cloud,
    const Eigen::Matrix4f& transformation,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coefficients,
    const cv::Mat& image,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud);

bool SaveTargetHoleCenters(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& lidar_centers,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& camera_centers,
    const std::string& output_path,
    std::string* error);

bool SaveSingleCalibrationResults(const Params& params,
                                  const SingleCalibrationResult& result,
                                  std::string* error);

bool EnsureOutputDirectory(const std::string& output_path,
                           std::string* error);

}  // namespace fast_calib
}  // namespace tools
}  // namespace jojo

#endif  // TOOLS_FAST_CALIB_SRC_CALIBRATION_INTERNAL_H_
