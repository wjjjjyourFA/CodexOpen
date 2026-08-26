/*
 * FAST-Calib core integration.
 *
 * The calibration algorithm originates from hku-mars/FAST-Calib and remains
 * subject to the GPLv2 license shipped with this module.
 */

#ifndef TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_FAST_CALIB_H_
#define TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_FAST_CALIB_H_

#define PCL_NO_PRECOMPILE

#include <cstdint>
#include <string>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace jojo {
namespace tools {
namespace fast_calib {

constexpr int kTargetCircleCount = 4;
constexpr double kGeometryTolerance = 0.08;

struct EIGEN_ALIGN16 PointXYZRing {
  PCL_ADD_POINT4D;
  std::uint16_t ring = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum class LidarType : int {
  kUnknown = 0,
  kSolidState = 1,
  kMechanical = 2,
};

// Defaults intentionally match the original FAST-Calib parameter loader.
struct Params {
  double x_min = 1.5;
  double x_max = 3.0;
  double y_min = -1.5;
  double y_max = 2.0;
  double z_min = -0.5;
  double z_max = 2.0;

  double fx = 1215.31801774424;
  double fy = 1214.72961288138;
  double cx = 1047.86571859677;
  double cy = 745.068353101898;
  double k1 = -0.33574781188503;
  double k2 = 0.10996870793601;
  double p1 = 0.000157303079833973;
  double p2 = 0.000544930726278493;

  double marker_size = 0.2;
  double delta_width_qr_center = 0.55;
  double delta_height_qr_center = 0.35;
  double delta_width_circles = 0.5;
  double delta_height_circles = 0.4;
  double circle_radius = 0.12;
  int min_detected_markers = 3;

  std::string image_path =
      "/home/chunran/calib_ws/src/fast_calib/data/image.png";
  std::string bag_path =
      "/home/chunran/calib_ws/src/fast_calib/data/input.bag";
  std::string lidar_topic = "/livox/lidar";
  std::string output_path =
      "/home/chunran/calib_ws/src/fast_calib/output";
};

struct LidarDebugClouds {
  pcl::PointCloud<PointXYZRing>::Ptr filtered{
      new pcl::PointCloud<PointXYZRing>};
  pcl::PointCloud<PointXYZRing>::Ptr plane{
      new pcl::PointCloud<PointXYZRing>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned{
      new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr edge{
      new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr centers_z0{
      new pcl::PointCloud<pcl::PointXYZ>};
};

struct SingleCalibrationResult {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  double rmse = 0.0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_centers{
      new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_centers{
      new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers{
      new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud{
      new pcl::PointCloud<pcl::PointXYZRGB>};
  cv::Mat annotated_image;
  LidarDebugClouds lidar_debug;
};

struct MultiCalibrationResult {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  double rmse = 0.0;
};

// Runs the original single-scene algorithm and writes all original output
// artifacts below params.output_path.
bool RunSingleCalibration(
    const Params& params,
    const cv::Mat& image,
    const pcl::PointCloud<PointXYZRing>::ConstPtr& cloud,
    LidarType lidar_type,
    SingleCalibrationResult* result,
    std::string* error);

// Reads the last three blocks from circle_center_record.txt and writes
// multi_calib_result.txt below output_path.
bool RunMultiSceneCalibration(const std::string& output_path,
                              MultiCalibrationResult* result,
                              std::string* error);

}  // namespace fast_calib
}  // namespace tools
}  // namespace jojo

POINT_CLOUD_REGISTER_POINT_STRUCT(
    jojo::tools::fast_calib::PointXYZRing,
    (float, x, x)(float, y, y)(float, z, z)(std::uint16_t, ring, ring))

#endif  // TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_FAST_CALIB_H_
