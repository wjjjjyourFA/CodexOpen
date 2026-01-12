#ifndef RADAR_DATA_H
#define RADAR_DATA_H

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

/* range rate
2D 传统毫米波雷达（如 Continental ARS408, Delphi ESR, Bosch MRR）通常只能通过多普勒效应测得沿波束方向的速度。
它的波束宽度较大、角分辨率低，无法区分“左右方向”的速度分量。
毫米波雷达的原始观测值是：
  参数	       说明
  range	      距离（m）
  azimuth	    水平角（°）
  range_rate	径向速度（m/s）
  RCS	        反射强度
*/

// 目标级信息
struct RadarObs {
  std::uint64_t time;

  std::int32_t obstacle_id;

  /*  relative position
  // longitude distance to the radar; (+) = forward; unit = m
  double longitude_dist;
  // lateral distance to the radar; (+) = left; unit = m
  double lateral_dist;
  */
  cstruct::Vector2f position;

  /*  relative velocity
  // longitude velocity to the radar; (+) = forward; unit = m/s
  double longitude_vel;
  // lateral velocity to the radar; (+) = left; unit = m/s
  double lateral_vel;
  */
  cstruct::Vector2f velocity;

  float azimuth;

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

  cstruct::Vector2f position_rms;
  cstruct::Vector2f velocity_rms;

  float azimuth_rms;
  float doppler_rms;
};

struct RadarData {
  std::uint64_t time;

  std::vector<RadarObs> obs;
};

#endif
