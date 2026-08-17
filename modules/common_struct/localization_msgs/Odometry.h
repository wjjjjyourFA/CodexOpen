#pragma once

#include <array>
#include <string>

#include "modules/common_struct/basic_msgs/Header.h"
#include "modules/common_struct/basic_msgs/Pose.h"
#include "modules/common_struct/basic_msgs/VectorPoint.h"

namespace jojo {
namespace common_struct {

struct Odometry {
  Header header;
  std::string child_frame_id;
  Pose pose;
  Vector3d linear_velocity;
  Vector3d angular_velocity;
  std::array<double, 36> pose_covariance{};
  std::array<double, 36> twist_covariance{};
};

}  // namespace common_struct
}  // namespace jojo
