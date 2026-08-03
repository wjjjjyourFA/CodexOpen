/*
 * @Author: HuangWei
 * @Date: 2024-04-09 15:07:02
 * @LastEditors: HuangWei
 * @LastEditTime: 2024-04-09 16:00:35
 * @FilePath: /chcnav_ros_0.48/misc/gauss_projection.h
 * @Description:
 *
 * Copyright (c) 2024 by JOJO, All Rights Reserved.
 */
#ifndef BLH2GAUSSXY_H
#define BLH2GAUSSXY_H

#pragma once

#include <math.h>
#include <stdint.h>

#include <iostream>
#include <regex>

#include "time.h"

namespace jojo {
namespace localization {
namespace common {

class GaussProjection {
 public:
  GaussProjection();
  ~GaussProjection();

  void SetCoordinateSystem(int coordinate_system);

  void Forward(double longitude, double latitude, double* gauss_x,
               double* gauss_y);

  void Inverse(double gauss_x, double gauss_y, double* longitude,
               double* latitude);

  // 地理坐标系（BLH，纬度、经度、高度） 转换为 平面坐标系（XY）.. lbk
  void blh2xy(double blh[], double pos[]);

  // .. lbk
  void xy2blh(double pos[], double blh[]);

  const double deg_to_rad = M_PI / 180.0;
  const double rad_to_deg = 180.0 / M_PI;
  // 3.1415926535898/180.0;
  // static double iPI = 0.0174532925199433;

 protected:
  const int ZoneWide = 6;  // 6度带宽

 private:
  // 长半轴 扁率
  double a, f;
  std::string coordinate_system_name;
};

}  // namespace common
}  // namespace localization
}  // namespace jojo

#endif
