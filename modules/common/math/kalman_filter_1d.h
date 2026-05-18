#ifndef KALMAN_FILTER_1D_H
#define KALMAN_FILTER_1D_H

#pragma once

#include <iostream>
#include <iomanip>
#include <cmath>

#include <Eigen/Dense>

#include "modules/common/math/math_utils_extra.h"
#include "modules/common/math/unit_converter.h"

namespace jojo {
namespace common {
namespace math {

// 一维数据 kalman 滤波，假设过程转换和观测转换均为1，且过程控制输入为0
// !!! 目前专用于角度滤波，单位弧度，未扩展到多维数据
template <typename T>
class KalmanFilter1D {
 public:
  KalmanFilter1D() {
    F_.setIdentity();  // 状态转移矩阵 F = [1]
    Q_.setZero();      // 初始化过程噪声 不合理的默认值，只是占位
    H_.setIdentity();  // 观测矩阵 H = [1]
    R_.setZero();      // 初始化观测噪声 不合理的默认值，只是占位
    B_.setIdentity();  // 控制矩阵 B = [1]（有控制输入）

    x_.setZero();      // 当前状态估计 上一时刻后验估计
    P_.setIdentity();  // 当前状态协方差 后验协方差
    y_.setZero();      // 观测残差（Measurement residual）
    S_.setZero();      // 残差协方差（Residual covariance）
    K_.setZero();      // 卡尔曼增益（Kalman gain）
    
    I_.setIdentity();  // 单位矩阵
  }

  void SetStateEstimate(const T& x, const T& P = 1.0, const T& Q = 0.001,
                        const T& R = 10.0) {
    x_(0, 0) = x;
    P_(0, 0) = P;
    Q_(0, 0) = Q;
    R_(0, 0) = R;
    is_initialized_ = true;
  }

  KalmanFilter1D(const T& x, const T& P, const T& Q = 0.001, const T& R = 10.0)
      : KalmanFilter1D() {
    SetStateEstimate(x, P, Q, R);
  }

  virtual ~KalmanFilter1D() = default;

  void Predict(double u);

  void Correct(double z);

  // θ预测（角度状态）
  void PredictTheta(double pre_value, double cur_value);

  // 观测更新（融合观测角度）
  void CorrectTheta(double cur_value);

  T GetStateEstimate() const { return x_(0, 0); }

  bool IsInitialized() const { return is_initialized_; }

 private:
  Eigen::Matrix<T, 1, 1> x_;  // 状态估计
  Eigen::Matrix<T, 1, 1> P_;  // 协方差矩阵
  Eigen::Matrix<T, 1, 1> F_;  // 状态转移矩阵
  Eigen::Matrix<T, 1, 1> Q_;  // 过程噪声
  Eigen::Matrix<T, 1, 1> H_;  // 观测矩阵
  Eigen::Matrix<T, 1, 1> R_;  // 观测噪声
  Eigen::Matrix<T, 1, 1> B_;  // 控制矩阵

  Eigen::Matrix<T, 1, 1> y_;
  Eigen::Matrix<T, 1, 1> S_;
  Eigen::Matrix<T, 1, 1> K_;

  Eigen::Matrix<T, 1, 1> I_;  // 单位矩阵

  bool is_initialized_ = false;
};

template <typename T>
inline void KalmanFilter1D<T>::Predict(double u) {
  if (!is_initialized_) {
    std::cerr << "[KalmanFilter1D] Not initialized!\n";
    return;
  }

  Eigen::Matrix<T, 1, 1> u_vec;
  u_vec(0, 0) = static_cast<T>(u);

  // 状态预测
  // x_ 是上一时刻的后验估计值（后验角度）。
  // + d_theta：表示把“角度变化量”加到上一时刻的状态上，得到当前时刻的预测值（先验估计）。
  // 标准 KF 里，Predict 是根据 模型 F + 控制输入 B*u 预测状态，不直接依赖测量值。
  x_ = F_ * x_ + B_ * u_vec;

  // 协方差预测
  P_ = F_ * P_ * F_.transpose() + Q_;
}

template <typename T>
inline void KalmanFilter1D<T>::PredictTheta(double pre_value, double cur_value) {
  // pre_value：表示前一个时刻你手里已知的角度（通常是上一帧测量值，或者上一帧滤波后的状态值）。
  // cur_theta：表示当前时刻你想用来预测的角度（通常是当前测量值或者当前里程计/IMU预测值）。
  // AngleDiffSigned(pre, cur)：计算两个角度之间的增量 Δθ，考虑 ±π 周期性

  if (!is_initialized_) {
    std::cerr << "[KalmanFilter1D] Not initialized!\n";
    return;
  }

  // 1. 计算角度增量（语义层）
  // a. KF 的核心假设：误差是“零均值”的
  //    AngleDiff 必须返回 (-π, +π]（或等价区间），而不是 0 ~ 2π
  // b. 角度增量 Δθ = cur - pre，不是一个绝对角度，而是角度变化量
  const double dtheta = apollo::common::math::AngleDiff(pre_value, cur_value);

  // 2. 调用基础 Predict
  this->Predict(dtheta);

  // 3. 状态归一化（状态约束）
  x_(0, 0) = static_cast<T>(NormalizeAngle0To2Pi(x_(0, 0)));
}

template <typename T>
inline void KalmanFilter1D<T>::Correct(double z) {
  // ACHECK(is_initialized_);

  Eigen::Matrix<T, 1, 1> z_vec;
  z_vec(0, 0) = static_cast<T>(z);

  // 计算创新（测量残差），需处理角度周期性
  y_ = z_vec - H_ * x_;  // innovation

  // 卡尔曼增益
  S_ = H_ * P_ * H_.transpose() + R_;

  K_ = P_ * H_.transpose() * S_.inverse();

  // 更新状态
  x_ = x_ + K_ * y_;

  // 更新协方差
  P_ = (I_ - K_ * H_) * P_;
}

template <typename T>
inline void KalmanFilter1D<T>::CorrectTheta(double cur_value) {
  // ACHECK(is_initialized_);

  // 1. 计算创新（语义层）
  Eigen::Matrix<T, 1, 1> z_vec;
  z_vec(0, 0) = static_cast<T>(cur_value);

  // 计算创新（测量残差），需处理角度周期性
  y_ = z_vec - H_ * x_;  // innovation

  y_(0, 0) = static_cast<T>(apollo::common::math::AngleDiff(0.0, y_(0, 0)));

  // 2. 后续完全走原 Correct 数学
  // 卡尔曼增益
  S_ = H_ * P_ * H_.transpose() + R_;

  K_ = P_ * H_.transpose() * S_.inverse();

  // 更新状态
  x_ = x_ + K_ * y_;

  // 更新协方差
  P_ = (I_ - K_ * H_) * P_;

  // 3. 状态归一化
  x_(0, 0) = static_cast<T>(NormalizeAngle0To2Pi(x_(0, 0)));
}

}  // namespace math
}  // namespace common
}  // namespace jojo

#endif
