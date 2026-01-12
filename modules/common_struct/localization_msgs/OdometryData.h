#ifndef ODOMETRY_DATA_H
#define ODOMETRY_DATA_H

#pragma once

#include <string>

#include "modules/common_struct/basic_msgs/OrientationAngles.h"
#include "modules/common_struct/basic_msgs/VectorPoint.h"

namespace cstruct = jojo::common_struct;

struct OdomData {
  uint64_t time;
  
  cstruct::Vector3f position;
  // azimuth pitch roll
  cstruct::OrientationAngles orientation;

  // velocity
  cstruct::Vector3f velocity;

  double speed;
};

#endif  // ODOMETRY_DATA_H
