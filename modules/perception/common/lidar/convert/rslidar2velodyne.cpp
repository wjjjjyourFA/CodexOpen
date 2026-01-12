#include "modules/perception/common/lidar/convert/rslidar2velodyne.h"

bool RsToVd(pcl::PointCloud<robosense_ros::PointIF>::Ptr point_rs_,
            pcl::PointCloud<velodyne_ros::PointXYZIR>::Ptr point_vd_,
            bool structured) {
  size_t point_num = point_rs_->size();
  if (point_num == 0) return false;

  point_vd_->clear();
  point_vd_->resize(point_num);
  point_vd_->header = point_rs_->header;
  point_vd_->width  = point_rs_->width;
  point_vd_->height = point_rs_->height;
  // point_vd_->is_dense = point_rs_->is_dense;
  point_vd_->is_dense = false;

  for (size_t i = 0; i < point_num; i++) {
    const auto& pt = point_rs_->points[i];
    auto& dst      = point_vd_->points[i];

    if (has_nan(pt)) {
      dst.x = dst.y = dst.z = std::numeric_limits<float>::quiet_NaN();
      dst.intensity         = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    dst.x         = pt.x;
    dst.y         = pt.y;
    dst.z         = pt.z;
    dst.intensity = pt.intensity;

    // remap ring id
    // /* way 1
    if (point_rs_->height == 16) {
      dst.ring = RING_ID_MAP_RS16[i / point_rs_->width];
    } else if (point_rs_->height == 128) {
      dst.ring = RING_ID_MAP_RUBY[i % point_rs_->height];
      // */
      // way 2
      // dst.ring = pt.ring;
    }
  }

  return true;
}

bool RsToVd(pcl::PointCloud<robosense_ros::PointIF>::Ptr point_rs_,
            pcl::PointCloud<velodyne_ros::PointXYZIRT>::Ptr point_vd_,
            bool structured) {
  size_t point_num = point_rs_->size();
  if (point_num == 0) return false;

  point_vd_->clear();
  point_vd_->resize(point_num);
  point_vd_->header = point_rs_->header;
  point_vd_->width  = point_rs_->width;
  point_vd_->height = point_rs_->height;
  // point_vd_->is_dense = point_rs_->is_dense;
  point_vd_->is_dense = false;

  auto min_point_it = std::min_element(
      point_rs_->points.begin(), point_rs_->points.end(),
      [](const robosense_ros::PointIF& a, const robosense_ros::PointIF& b) {
        return a.timestamp < b.timestamp;
      });

  double& time_start = min_point_it->timestamp;

  for (size_t i = 0; i < point_num; i++) {
    const auto& pt = point_rs_->points[i];
    auto& dst      = point_vd_->points[i];

    if (has_nan(pt)) {
      dst.x = dst.y = dst.z = std::numeric_limits<float>::quiet_NaN();
      dst.intensity         = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    dst.x         = pt.x;
    dst.y         = pt.y;
    dst.z         = pt.z;
    dst.intensity = pt.intensity;

    // remap ring id
    // /* way 1
    if (point_rs_->height == 16) {
      dst.ring = RING_ID_MAP_RS16[i / point_rs_->width];
    } else if (point_rs_->height == 128) {
      dst.ring = RING_ID_MAP_RUBY[i % point_rs_->height];
    }
    // */
    // way 2
    // dst.ring = pt.ring;

    // velodyne 每个点存的是帧内时间
    dst.time = pt.timestamp - time_start;
  }

  return true;
}