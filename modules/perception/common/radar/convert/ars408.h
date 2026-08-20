#ifndef __ARS408_H__
#define __ARS408_H__

#include <vector>

#include <Eigen/Core>

/**** struct of the base point ****/
// 用于描述雷达点的结构体
struct RadarPoint {
  int id           = 0;  // 点的唯一标识符
  float x          = 0.0f;  // X坐标
  float y          = 0.0f;  // Y坐标
  float range      = 0.0f;
  float azimuth    = 0.0f;
  float range_rate = 0.0f;
  float velocity   = 0.0f;  // 速度（单位：m/s）
  float velocity_x = 0.0f;  // X方向速度（单位：m/s）
  float velocity_y = 0.0f;  // Y方向速度（单位：m/s）
  float rcs      = 0.0f;  // 雷达截面（Radar Cross Section），表示反射信号的强度
  int dyn_prop   = 0;  // 动态属性，例如是否为动态物体
  int prob_exist = 0;  // 存在的概率（0-100）
  int class_type = 0;  // 类别类型，例如：车辆、行人等

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // __ARS408_H__
