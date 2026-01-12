#ifndef __PCL_TYPES
#define __PCL_TYPES

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/register_point_struct.h>
#include <pcl/common/io.h>

// clang-format off
namespace pcl {
/*一个具有XYZ、intensity、ring的点云类型*/
struct EIGEN_ALIGN16 PointXYZIR {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  std::uint16_t ring = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*一个具有XYZ、intensity、timestamp、ring的点云类型*/
struct EIGEN_ALIGN16 PointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  std::uint16_t ring = 0;
  double timestamp = 0.0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*一个具有XYZ、RGB、intensity、ring的点云类型*/
struct EIGEN_ALIGN16 PointXYZIRRGB {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  std::uint16_t ring = 0;
  PCL_ADD_RGB;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}  // namespace pcl

// namespace pcl 注册pcl数据类型
POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)(uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)(uint16_t, ring, ring)
    (double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZIRRGB, (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)(uint16_t, ring, ring)
    (uint32_t, rgb, rgb))
// clang-format on

// 将 PointXYZ 点云转换为 PointXYZI 点云，默认强度值为 0.0f
inline static bool ConvertXYZtoXYZI(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud,
    float default_intensity = 0.0f) {
  // 检查输入
  if (!input_cloud || input_cloud->empty()) {
    return false;
  }

  // 若 output_cloud 未分配，则分配空间
  if (!output_cloud) {
    output_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  // 直接复制 x, y, z 和 meta 信息（header, width, height, is_dense）
  pcl::copyPointCloud(*input_cloud, *output_cloud);

  // 给所有点赋默认 intensity
  for (auto& p : output_cloud->points) {
    p.intensity = default_intensity;
  }

  return true;
}

#endif  // __PCL_TYPES
