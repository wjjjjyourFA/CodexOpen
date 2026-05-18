#ifndef __DJI_LIVOX_H__
#define __DJI_LIVOX_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "modules/perception/tools/pcl/point_types.h"

// clang-format off
namespace livox_ros {
  
struct PointXYZIRT {
  PCL_ADD_POINT4D
  // uint8_t reflectivity = 0;
  float intensity = 0.0;
  uint8_t line = 0;
  // uint32_t offset_time = 0.0;
  double timestamp = 0.0;
  uint8_t tag = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace livox_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    livox_ros::PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)
    // (std::uint8_t, reflectivity, reflectivity)(std::uint8_t, line, line)
    // (uint32_t, offset_time, offset_time))
    (float, intensity, intensity)(std::uint8_t, line, line)
    (double, timestamp, timestamp))

// clang-format on

bool LvToPcl(pcl::PointCloud<livox_ros::PointXYZIRT>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZIRT>::Ptr point_pcl_);

#endif
