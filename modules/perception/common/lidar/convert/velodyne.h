/* -*- mode: C++ -*- */

#ifndef __VELODYNE_POINTCLOUD_POINT_TYPES_H
#define __VELODYNE_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

#include "modules/perception/common/base/pcl_extra/point_types.h"

// Velodyne 的 ring 字段本身就是顺序的（0,1,2,...），但顺序“不是按物理从上到下”。
// 1. ring 数值是连续的、从 0 递增 → 是顺序的 (0~15 / 0~31 / 0~63 / 0~127)
// 2. Velodyne 的“激光束物理顺序”与 “ring 数值顺序”不一样

// clang-format off
namespace velodyne_ros {
/** Euclidean Velodyne coordinate, including intensity and ring number. */
struct PointXYZIR {
  PCL_ADD_POINT4D;        // quad-word XYZ
  PCL_ADD_INTENSITY;      // laser intensity reading
  // float intensity = 0.0;
  uint16_t ring = 0;      // laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

struct PointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring = 0;
  float time = 0.0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    velodyne_ros::PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)(uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    velodyne_ros::PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)(uint16_t, ring, ring)
    (float, time, time))
// clang-format on

bool VdToPcl(pcl::PointCloud<velodyne_ros::PointXYZIR>::Ptr point_vd_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_,
             bool structured = true);

bool VdToPcl(pcl::PointCloud<velodyne_ros::PointXYZIRT>::Ptr point_vd_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_,
             bool structured = true);

// 数据集构建
bool VdToPcl(pcl::PointCloud<velodyne_ros::PointXYZIRT>::Ptr point_vd_,
             pcl::PointCloud<pcl::PointXYZIRT>::Ptr point_pcl_);

#endif  // __VELODYNE_POINTCLOUD_POINT_TYPES_H
