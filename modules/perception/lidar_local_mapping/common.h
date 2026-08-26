#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

// TODO：可添加 旋转 度量
inline float Distance(const Eigen::Matrix4f& A, const Eigen::Matrix4f& B) {
  Eigen::Vector3f t1 = A.block<3, 1>(0, 3);
  Eigen::Vector3f t2 = B.block<3, 1>(0, 3);
  return (t1 - t2).norm();
}