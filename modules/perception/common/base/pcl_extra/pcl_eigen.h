#ifndef __PCL_EIGEN_H
#define __PCL_EIGEN_H

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>  // 提供 pcl::traits::fieldList

#include <Eigen/Geometry>  // Eigen::Quaterniond, Eigen::Affine3d
#include <pcl/common/transforms.h>  // pcl::getTranslationAndEulerAngles

#include "modules/common/math/math_utils.h"
#include "modules/common/math/math_utils_extra.h"

template <typename PointT>
inline void pcl_to_eigen(const typename pcl::PointCloud<PointT>::Ptr& pcl_cloud,
                         Eigen::Matrix<float, 4, Eigen::Dynamic>& eigen_cloud) {
  const size_t size = pcl_cloud ? pcl_cloud->points.size() : 0;
  eigen_cloud.resize(4, size);  // 4行，x,y,z,1齐次坐标

  for (int i = 0; i < size; ++i) {
    eigen_cloud(0, i) = pcl_cloud->points[i].x;
    eigen_cloud(1, i) = pcl_cloud->points[i].y;
    eigen_cloud(2, i) = pcl_cloud->points[i].z;
    eigen_cloud(3, i) = 1.0f;
  }
}

// from lbk ColorPoint
// 返回值是 弧度，不是度数
inline double GetYaw(const Eigen::Quaterniond& q) {
  Eigen::Affine3d transform = Eigen::Affine3d(q);
  double x, y, z, r, p, yaw;
  pcl::getTranslationAndEulerAngles(transform, x, y, z, r, p, yaw);
  // std::cout<<"yaw: "<< yaw <<std::endl;
  return yaw;
}

template <typename PointT>
inline void point_to_eigen(const PointT& point,
                           Eigen::Vector4f& transformed_point) {
  transformed_point << point.x, point.y, point.z, 1.0f;
}

template <typename PointT>
inline void point_mm_to_eigen_cm(const PointT& point,
                                 Eigen::Vector4f& transformed_point) {
  transformed_point << point.x * 10.0f, point.y * 10.0f, point.z * 10.0f, 1.0f;
}

template <typename PointT>
inline void point_mm_to_eigen_m(const PointT& point,
                                Eigen::Vector4f& transformed_point) {
  transformed_point << point.x * 1000.0f, point.y * 1000.0f, point.z * 1000.0f,
      1.0f;
}

template <typename PointT>
inline void point_m_to_eigen_cm(const PointT& point,
                                Eigen::Vector4f& transformed_point) {
  transformed_point << point.x / 100.0f, point.y / 100.0f, point.z / 100.0f,
      1.0f;
}

#include "modules/common_struct/basic_msgs/EulerAngles.h"

namespace common_struct = jojo::common_struct;

// 四元数转欧拉角 ZYX 顺序 单位：弧度
// #include "modules/perception/common/algorithm/image_processing/util/utils.h"
// 两个函数等价，cv_utils.h 中的 角度输出是倒序
inline common_struct::EulerAngles QuaternionToEuler(const Eigen::Quaterniond& q) {
  // 将四元数转为旋转矩阵
  Eigen::Matrix3d R = q.toRotationMatrix();

  double roll, pitch, yaw;

  // 标准 ZYX 顺序
  // 防止数值超界
  // C++ 17
  // pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  pitch = std::asin(apollo::common::math::Clamp(-R(2, 0), -1.0, 1.0));

  if (std::abs(R(2, 0)) < 0.99999) {  // 非奇异
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw  = std::atan2(R(1, 0), R(0, 0));
  } else {  // 奇异点
    roll = 0;
    yaw  = std::atan2(-R(0, 1), R(1, 1));
  }

  return common_struct::EulerAngles{yaw, pitch, roll};
}

#endif  //
