#pragma once

#include <deque>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "modules/common/math/math_utils.h"

#include "modules/perception/common/config/sensor_extrinsics.h"

#include "modules/common_struct/sensor_msgs/GnssData.h"
#include "modules/common_struct/sensor_msgs/ImuData.h"
#include "modules/common_struct/localization_msgs/OdometryData.h"

// #include "modules/perception/tools/pcl/pcl_viewer.h"

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

namespace jojo {
namespace perception {
namespace lidar {

inline Eigen::Quaterniond RpyToQuat(
    const jojo::common_struct::OrientationAngles& o) {
  Eigen::AngleAxisd roll(o.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(o.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(o.azimuth, Eigen::Vector3d::UnitZ());

  return yaw * pitch * roll;
}

// Lidar 去畸变，运动补偿
class MotionCompensator {
 public:
  MotionCompensator() {};
  ~MotionCompensator() {};

  void SetExtrinsicMatrix(const Eigen::Matrix4f& extrinsic_matrix);

  void Init();

  // cloud 已重建的 organized 点云
  // deskew_time_ratio 补偿参考时间在扫描中的位置 (0~1)
  void UndistortPointCloudByImu(
      PointCloudXYZI::Ptr cloud,
      const std::deque<jojo::common_struct::ImuData>& imu_data,
      uint64_t cloud_start_time, uint64_t cloud_end_time,
      double deskew_time_ratio, bool visualize);

  void UndistortPointCloudByGnss(
      PointCloudXYZI::Ptr cloud,
      const std::deque<jojo::common_struct::GnssData>& pose_data,
      uint64_t cloud_start_time, uint64_t cloud_end_time,
      double deskew_time_ratio, bool visualize);

  void UndistortPointCloudByOdom(
      PointCloudXYZI::Ptr cloud,
      const std::deque<jojo::common_struct::OdomData>& pose_data,
      uint64_t cloud_start_time, uint64_t cloud_end_time,
      double deskew_time_ratio, bool visualize);

 protected:
  // rslidar 128
  size_t total_rows = 128;
  size_t total_cols = 1800;

  PointCloudXYZI::Ptr ori_cloud_ = nullptr;

  bool InterpolatePoseBase(double timestamp,
                           const jojo::common_struct::OdomData& pose1,
                           const jojo::common_struct::OdomData& pose2,
                           Eigen::Matrix4d& pose_out);

  bool InterpolatePose(const std::deque<jojo::common_struct::OdomData>& poses,
                       double timestamp, Eigen::Matrix4d& pose_out);

 private:
  pcl::visualization::PCLVisualizer::Ptr viewer_ = nullptr;

  Eigen::Matrix4f extrinsic_matrix_;
  Eigen::Matrix4f extrinsic_matrix_inv_;
  Eigen::Matrix4d extrinsic_matrix_d;
  Eigen::Matrix4d extrinsic_matrix_inv_d;
};

}  // namespace lidar
}  // namespace perception
}  // namespace jojo