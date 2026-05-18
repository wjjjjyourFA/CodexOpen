#ifndef ODOMETRY_DATA_H
#define ODOMETRY_DATA_H

#pragma once

#include <cstdint>
#include <string>

#include "modules/common_struct/basic_msgs/OrientationAngles.h"
#include "modules/common_struct/basic_msgs/VectorPoint.h"
#include "modules/common_struct/basic_msgs/Pose6D.h"

// namespace cstruct = jojo::common_struct;

namespace jojo {
namespace common_struct {

struct OdomData {
  std::uint64_t time;

  Vector3f position;
  // azimuth pitch roll
  OrientationAngles orientation;

  // velocity
  Vector3f velocity;

  double speed;
};

inline SE3Pose ConvertOdomToPose(const OdomData& src, bool input_is_ned = false) {
  // static constexpr double DEG2RAD = M_PI / 180.0;

  SE3Pose pose;

  pose.pos = Eigen::Vector3d(src.position.x, src.position.y, src.position.z);

  double roll  = src.orientation.roll;
  double pitch = src.orientation.pitch;
  double yaw   = src.orientation.azimuth;

  if (input_is_ned) {
    pitch = -pitch;
    yaw   = M_PI_2 - yaw;
  }

  Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());

  pose.rot = Rz * Ry * Rx;
  pose.rot.normalize();

  return pose;
}

}  // namespace common_struct
}  // namespace jojo

#endif  // ODOMETRY_DATA_H
