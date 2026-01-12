#ifndef RADAR_4D_DATA_H
#define RADAR_4D_DATA_H

#pragma once

#include <cstdint>
#include <vector>

#include "modules/common_struct/basic_msgs/VectorPoint.h"

namespace cstruct = jojo::common_struct;

//                x axis  ^
//                        | longitude_dist
//                        |
//                        |
//                        |
//          lateral_dist  |
//          y axis        |
//        <----------------
//        ooooooooooooo   //radar front surface

// 目标级信息
struct Radar4DObs {
  std::uint64_t time;

  std::int32_t obstacle_id;

  /*  relative position
  // longitude distance to the radar; (+) = forward; unit = m
  double longitude_dist;
  // lateral distance to the radar; (+) = left; unit = m
  double lateral_dist;

  double vertical_dist;

  */
  cstruct::Vector3f position;

  /*  relative velocity
  // longitude velocity to the radar; (+) = forward; unit = m/s
  double longitude_vel;
  // lateral velocity to the radar; (+) = left; unit = m/s
  double lateral_vel;

  double vertical_vel;
  */
  cstruct::Vector3f velocity;

  float azimuth;
  float elevation;

  float doppler;

  // obstacle Radar Cross-Section; unit = dBsm
  double rcs;

  // 0 = moving, 1 = stationary, 2 = oncoming, 3 = stationary candidate
  // 4 = unknown, 5 = crossing stationary, 6 = crossing moving, 7 = stopped
  std::int32_t dynprop;  // dynamic_property

  float probexist;  // prob_of_exist

  // 0: point; 1: car; 2: truck; 3: pedestrian; 4: motorcycle; 5: bicycle; 6:
  // wide; 7: unknown
  std::int32_t obstacle_class;

  cstruct::Vector3f position_rms;
  cstruct::Vector3f velocity_rms;

  float azimuth_rms;
  float elevation_rms;
  float doppler_rms;
};

struct Radar4DData {
  std::uint64_t time;

  std::vector<Radar4DObs> obs;
};

#endif
