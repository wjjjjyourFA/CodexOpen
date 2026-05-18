#ifndef HUGIN_H_
#define HUGIN_H_

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace hugin {

struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float range;
  float azimuth;
  float elevation;
  float doppler;  // range_rate
  float rcs;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 保证内存对齐
};

struct alignas(16) PointCloud {
  uint64_t timestamp = 0;

  pcl::PointCloud<hugin::Point> data;
};

}  // namespace hugin

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    hugin::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, range, range)(float, azimuth, azimuth)
    (float, elevation, elevation)(float, doppler, doppler)(float, rcs, rcs))
// clang-format on

// 避免多重定义，让多个 .cpp 文件共享变量
// extern hugin::PointCloud hugin_pointcloud;

#endif
