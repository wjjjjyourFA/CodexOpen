#ifndef GNSS_DATA_H
#define GNSS_DATA_H

#pragma once

#include <cstdint>
#include <string>

#include "modules/common_struct/basic_msgs/GeoPoint.h"
#include "modules/common_struct/basic_msgs/VectorPoint.h"
#include "modules/common_struct/basic_msgs/OrientationAngles.h"
#include "modules/common_struct/basic_msgs/GeoVelocity.h"

namespace cstruct = jojo::common_struct;

struct GnssData {
  int info;
  int week;
  std::uint64_t time;

  // longitude latitude altitude;
  cstruct::GeoPoint position;

  // gauss
  cstruct::Vector2f gauss_point;

  // azimuth pitch roll
  cstruct::OrientationAngles orientation;

  cstruct::Vector3f gyro;
  cstruct::Vector3f acc;

  // east north up velocity
  cstruct::GeoVelocity velocity;

  int main_satellite_num;
  int vice_satellite_num;
  int status;
  int age;

  int warning;
  std::string check_sum = "";
};

#endif  // GNSS_DATA_H
