#ifndef IMU_DATA_H
#define IMU_DATA_H

#pragma once

#include <cstdint>

#include "modules/common_struct/basic_msgs/VectorPoint.h"

namespace cstruct = jojo::common_struct;

struct ImuData {
  std::uint64_t time;

  cstruct::Vector3f gyro;
  cstruct::Vector3f acc;
};

#endif
