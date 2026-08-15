#ifndef GNSS_DATA_H
#define GNSS_DATA_H

#pragma once

#include <cstdint>
#include <string>

#include "modules/common_struct/basic_msgs/GeoPoint.h"
#include "modules/common_struct/basic_msgs/GeoVelocity.h"
#include "modules/common_struct/basic_msgs/OrientationAngles.h"
#include "modules/common_struct/basic_msgs/Pose6D.h"
#include "modules/common_struct/basic_msgs/VectorPoint.h"

// namespace cstruct = jojo::common_struct;

namespace jojo {
namespace common_struct {

struct GnssData {
  int info{};
  int week{};
  std::uint64_t time{};

  // longitude latitude altitude;
  GeoPoint position;

  // gauss
  Vector2f gauss_point;

  // azimuth pitch roll
  OrientationAngles orientation;

  Vector3f gyro;
  Vector3f acc;

  // east north up velocity
  GeoVelocity velocity;

  int main_satellite_num{};
  int vice_satellite_num{};
  int status{};
  int age{};

  int warning{};
  std::string check_sum;
};

inline SE3Pose ConvertGnssToPose(const GnssData& src,
                                 bool input_is_ned = false) {
  // static constexpr double DEG2RAD = M_PI / 180.0;

  SE3Pose pose;

  pose.time = static_cast<double>(src.time);
  // =========================
  // 1. 位置（假设已是ENU / map坐标）
  // =========================
  pose.pos = Eigen::Vector3d(src.gauss_point.x, src.gauss_point.y,
                             src.position.altitude);

  double roll  = src.orientation.roll;
  double pitch = src.orientation.pitch;
  double yaw   = src.orientation.azimuth;
  // =========================
  // 2. 欧拉角（deg → rad）
  // =========================
  // 角度单位恢复（角度制）-> 弧度制
  // double roll  = src.orientation.roll * DEG2RAD;
  // double pitch = src.orientation.pitch * DEG2RAD;
  // double yaw   = src.orientation.azimuth * DEG2RAD;

  // std::cout << std::fixed << std::setprecision(6) << "yaw pitch roll: " << yaw
  //           << " " << pitch << " " << roll << std::endl;

  // =========================
  // ⚠️ 关键：GNSS航向角修正
  // =========================
  // 常见GNSS: NED
  // - 0° = 北
  // - 顺时针增加
  //
  // Eigen: ENU
  // - 0 = x轴
  // - 逆时针为正
  //
  // 👉 转换为 ENU (x东 y北 z上)：
  if (input_is_ned) {
    // static constexpr double M_PI_2 = M_PI / 2;
    pitch = -pitch;
    yaw   = M_PI_2 - yaw;
  }

  // =========================
  // 3. ZYX顺序：yaw（Z）→ pitch（Y）→ roll（X）
  // =========================
  Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());

  pose.rot = Rz * Ry * Rx;
  pose.rot.normalize();

  // std::cout << "pose.pos: " << pose.pos << std::endl;
  // std::cout << "pose.rot (x,y,z,w): " << pose.rot.coeffs().transpose() << std::endl;
  // Eigen::Vector3d euler = pose.rot.toRotationMatrix().eulerAngles(2, 1, 0);
  // std::cout << "yaw pitch roll: " << euler.transpose() << std::endl;

  return pose;
}

}  // namespace common_struct
}  // namespace jojo

#endif  // GNSS_DATA_H
