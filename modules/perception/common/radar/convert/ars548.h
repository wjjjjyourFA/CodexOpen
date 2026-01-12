#ifndef __ARS548_H__
#define __ARS548_H__
#include <vector>

#include <Eigen/Core>

// 用于描述雷达四维点的结构体，包含空间坐标、径向速度等信息
struct Radar4DPoint {
  int id;  // 点的唯一标识符
  float x;  // X坐标
  float y;  // Y坐标
  float z;  // Z坐标
  float range;
  float range_rate;  // 径向速度（单位：m/s）
  float azimuth;
  float velocity;  // 速度（单位：m/s）
  float range_rate_rms;  // 标准差，表示测量的精度或不确定性
  float v_diff;  // 径向速度与车速在径向方向上的差
  int rcs;  // 雷达截面（Radar Cross Section）
  int dyn_prop;  // 动态属性
  int prob_exist;  // 存在的概率（0-100）
  int class_type;  // 类别类型，例如：车辆、行人等

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // __ARS548_H__