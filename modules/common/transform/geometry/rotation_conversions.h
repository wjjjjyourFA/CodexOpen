#pragma once

#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace jojo {
namespace common {
namespace transform {

// YPR ↔ Rotation
template <typename T>
Eigen::Matrix<T, 3, 3> YPR2RotationZYX(const Eigen::Matrix<T, 3, 1>& ypr) {
  // ZYX
  // return R = Rot_z(yaw) * Rot_y(pitch) * Rot_x(roll)

  T yaw   = ypr.x();
  T pitch = ypr.y();
  T roll  = ypr.z();

  T cy = cos(yaw);
  T sy = sin(yaw);
  T cp = cos(pitch);
  T sp = sin(pitch);
  T cr = cos(roll);
  T sr = sin(roll);

  Eigen::Matrix<T, 3, 3> R;

  R(0, 0) = cp * cy;
  R(0, 1) = sp * sr * cy - cr * sy;
  R(0, 2) = sp * cr * cy + sr * sy;

  R(1, 0) = cp * sy;
  R(1, 1) = sp * sr * sy + cr * cy;
  R(1, 2) = sp * cr * sy - sr * cy;

  R(2, 0) = -sp;
  R(2, 1) = cp * sr;
  R(2, 2) = cp * cr;

  return R;
}

template <typename T>
Eigen::Matrix<T, 3, 3> YPR2RotationZXY(const Eigen::Matrix<T, 3, 1>& ypr) {
  // ZXY
  // return R = Rot_z(yaw) * Rot_x(pitch) * Rot_y(roll)

  T yaw   = ypr.x();  // Z
  T pitch = ypr.y();  // X
  T roll  = ypr.z();  // Y

  T crz = std::cos(yaw);
  T srz = std::sin(yaw);
  T crx = std::cos(pitch);
  T srx = std::sin(pitch);
  T cry = std::cos(roll);
  T sry = std::sin(roll);

  Eigen::Matrix<T, 3, 3> R;

  /* way 1
  Eigen::Matrix<T, 3, 3> Rx, Ry, Rz, R;

  // clang-format off
  Rz << crz, -srz, 0,
        srz,  crz, 0,
          0,    0, 1;

  Rx << 1,   0,    0,
        0, crx, -srx,
        0, srx,  crx;

  Ry <<  cry, 0, sry,
           0, 1,   0,
        -sry, 0, cry;
  // clang-format on

  R = Rz * Rx * Ry;
  */

  // 直接展开（比 Rx*Ry*Rz 更高效）
  R(0, 0) = crz * cry - srz * srx * sry;
  R(0, 1) = -srz * crx;
  R(0, 2) = crz * sry + srz * srx * cry;

  R(1, 0) = srz * cry + crz * srx * sry;
  R(1, 1) = crz * crx;
  R(1, 2) = srz * sry - crz * srx * cry;

  R(2, 0) = -crx * sry;
  R(2, 1) = srx;
  R(2, 2) = crx * cry;

  return R;
}

// Rotation ↔ Quaternion
void RotationToQuaternion(const Eigen::Matrix3d& R, Eigen::Quaterniond& q);

void RotationToQuaternion(const Eigen::Matrix3d& R, Eigen::Vector4d& q);

// Rotation ↔ Euler
// Z → X → Y 顺序（Z1X2Y3）不是常见的 Z → Y → X（ZYX）
template <typename T>
void RotationToEulerZXY(const Eigen::Matrix<T, 3, 3>& R,
                        Eigen::Matrix<T, 3, 1>& ypr) {
  // Z1X2Y3, applicable for our coordinate (X->right, Y->forward, Z->upward)
  // Z → X → Y 顺序（Z1X2Y3）不是常见的 Z → Y → X（ZYX）

  // ypr[0] = yaw; ypr[1] = pitch; ypr[2] = roll

  ypr(1) = std::asin(std::min(std::max(R(2, 1), T{-1}), T{1}));  // pitch
  T cp   = std::cos(ypr(1));

  if (std::abs(cp) > T(1e-6)) {
    ypr(2) = -std::atan2(R(2, 0), R(2, 2));  // roll
    ypr(0) = -std::atan2(R(0, 1), R(1, 1));  // yaw
  } else {
    // 万向锁（pitch ≈ ±90°）
    ypr(2) = T(0);
    ypr(0) = -std::atan2(-R(1, 0), R(0, 0));
  }
}

// 常见的 Z → Y → X（ZYX）
template <typename T>
void RotationToEulerZYX(const Eigen::Matrix<T, 3, 3>& R,
                        Eigen::Matrix<T, 3, 1>& ypr) {
  // 标准 ZYX（工业默认）

  // ypr = [yaw, pitch, roll]

  ypr(1) = std::asin(std::min(std::max(-R(2, 0), T{-1}), T{1}));  // pitch
  T cp   = std::cos(ypr(1));

  if (std::abs(cp) > T(1e-6)) {
    ypr(2) = std::atan2(R(2, 1), R(2, 2));  // roll
    ypr(0) = std::atan2(R(1, 0), R(0, 0));  // yaw
  } else {
    // 接近万向锁
    ypr(2) = T(0);
    ypr(0) = std::atan2(-R(0, 1), R(1, 1));
  }
}

}  // namespace transform
}  // namespace common
}  // namespace jojo
