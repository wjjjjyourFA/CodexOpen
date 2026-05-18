// # This represents a vector in free space.
// # x1e-2
// int32 x          # in cm
// int32 y
// int32 z

// # 为了联通CAN总线，推算采用 0.001rad 为单位
// # 姿态角（单位：0.001rad）
// # 使用时 x1e-3 以获得实际值
// int32 azimuth    # 偏航角（-180 至 180) 实际是 0.001rad
// int32 pitch      # 俯仰角（-90 至 90)
// int32 roll       # 横滚角（-180 至 180）

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace jojo {
namespace common_struct {

struct Pose6D {
  double x{};
  double y{};
  double z{};
  double roll{};
  double pitch{};
  double azimuth{};  // yaw

  Pose6D() = default;
  Pose6D(double _x, double _y, double _z, double _roll, double _pitch,
         double _azimuth)
      : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), azimuth(_azimuth) {}
};

struct SE3Pose {
  double time;

  Eigen::Vector3d pos;  // 平移
  Eigen::Quaterniond rot;  // 旋转

  // 转 4x4 矩阵（float，给PCL用）
  Eigen::Matrix4f matrix() const {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    T.block<3, 3>(0, 0) = rot.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = pos.cast<float>();

    return T;
  }

  // 转 double 矩阵（算法用）
  Eigen::Matrix4d matrix_d() const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T.block<3, 3>(0, 0) = rot.toRotationMatrix();
    T.block<3, 1>(0, 3) = pos;

    return T;
  }

  // 从矩阵构造
  static SE3Pose FromMatrix(const Eigen::Matrix4d& T) {
    SE3Pose pose;
    pose.pos = T.block<3, 1>(0, 3);
    pose.rot = Eigen::Quaterniond(T.block<3, 3>(0, 0));
    return pose;
  }

  // 逆变换
  SE3Pose inverse() const {
    SE3Pose inv;

    inv.rot = rot.conjugate();
    inv.pos = -(inv.rot * pos);

    return inv;
  }

  // 点变换
  Eigen::Vector3d transform(const Eigen::Vector3d& p) const {
    return rot * p + pos;
  }

  // 组合（位姿叠加）
  SE3Pose operator*(const SE3Pose& other) const {
    SE3Pose res;

    res.rot = rot * other.rot;
    res.pos = rot * other.pos + pos;

    return res;
  }
};

}  // namespace common_struct
}  // namespace jojo