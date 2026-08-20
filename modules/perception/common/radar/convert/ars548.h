#ifndef __ARS548_H__
#define __ARS548_H__

#include <vector>

#include <Eigen/Core>

// 用于描述雷达四维点的结构体，包含空间坐标、径向速度等信息
struct Radar4DPoint {
  int id               = 0;  // 点的唯一标识符
  float x              = 0.0f;  // X坐标
  float y              = 0.0f;  // Y坐标
  float z              = 0.0f;  // Z坐标
  float range          = 0.0f;
  float range_rate     = 0.0f;  // 径向速度（单位：m/s）
  float azimuth        = 0.0f;
  float velocity       = 0.0f;  // 速度（单位：m/s）
  float range_rate_rms = 0.0f;  // 标准差，表示测量的精度或不确定性
  float v_diff         = 0.0f;  // 径向速度与车速在径向方向上的差
  int rcs              = 0;  // 雷达截面（Radar Cross Section）
  int dyn_prop         = 0;  // 动态属性
  int prob_exist       = 0;  // 存在的概率（0-100）
  int class_type       = 0;  // 类别类型，例如：车辆、行人等

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // __ARS548_H__
