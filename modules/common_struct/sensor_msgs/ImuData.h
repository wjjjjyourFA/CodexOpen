#ifndef IMU_DATA_H
#define IMU_DATA_H

#pragma once

#include <cstdint>

#include "modules/common_struct/basic_msgs/VectorPoint.h"

// namespace cstruct = jojo::common_struct;

namespace jojo {
namespace common_struct {

struct ImuData {
  // std::uint64_t time;
  double time{};

  Vector3d gyro;  // [rad/s]
  Vector3d acc;  // [m/s^2]
};

}  // namespace common_struct
}  // namespace jojo

#endif
