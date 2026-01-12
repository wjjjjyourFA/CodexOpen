#ifndef _MATH_UTILS_EXTRA_H
#define _MATH_UTILS_EXTRA_H

#include <cstdint>
#include <cfloat>   // DBL_MIN, DBL_EPSILON
#include <random>

#include <Eigen/Core>

#include "modules/common/math/math_utils.h"

#define M_3_2_PI (3 * M_PI) / 2  // 3/2 pi
#define M_2_1_PI (2 * M_PI)  // 2 pi

namespace jojo {
namespace common {
namespace math {
// using namespace apollo::common::math;

double distSquare(const double x1, const double y1, const double x2,
                  const double y2);

double dist(const double x1, const double y1, const double x2, const double y2);

/**
 * @brief Normalize angle to [0, 2PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
double NormalizeAngle0To2Pi(double angle);

// degree [-180, 180]
float normalize_angle(const float angle);

// degree [0, 360]
float normalize_angle_0to360(const float angle);

// Absolute value angle difference
// degree [-180, 180]
float angle_diff(const float from, const float to);

inline uint64_t TimestampDiff(uint64_t ts1, uint64_t ts2) {
  return (ts1 > ts2) ? (ts1 - ts2) : (ts2 - ts1);
}

// 返回的随机数范围是 [-1, 1]
/*
inline float RandomFloatNeg1To1() {
  float a = rand() % 101 / (float)50 - 1;
  return a;
}
*/

// 返回的随机数范围是 [-1, 1]
inline float RandomFloatNeg1To1() {
  static std::mt19937 gen(std::random_device{}());
  static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  return dist(gen);
}

// 返回的随机数范围是 [0, 1]
/*
inline float RandomFloat0To1() {
  float a = rand() % 101 / (float)100;
  return a;
}
*/

// 返回的随机数范围是 [0, 1]
inline float RandomFloat0To1() {
  static std::mt19937 gen(std::random_device{}());
  static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
  return dist(gen);
}

/* C++ 17 std::clamp
template <typename T>
inline T ClampValue(T value, T minVal, T maxVal) {
  return std::clamp(value, minVal, maxVal);
}
*/

inline Eigen::Matrix4f GetTransMatrix(int b_flu_none_rfu) {
  Eigen::Matrix4f trans_matrix = Eigen::Matrix4f::Identity();

  // clang-format off
  switch (b_flu_none_rfu) {
    case 0:  // RFUToFLU 右前上 转 前左上
      trans_matrix << 0, 1, 0, 0,
                     -1, 0, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;
    case 1:
      break;
    case 2:  // FLUToRFU 前左上 转 右前上
      trans_matrix << 0, -1, 0, 0,
                      1, 0, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;
      break;
    default:
      break;
  }
  // clang-format on

  return trans_matrix;
}

// Compare doubles by relative error.
// 在浮点数数值精度允许范围内，a 和 b 是否可以认为是相等的
inline bool double_equal(const double& a, const double& b, double RELATIVE_ERROR_FACTOR=100.0) {
  // trivial case
  if (a == b) return true;

  double abs_diff = fabs(a - b);
  double aa       = fabs(a);
  double bb       = fabs(b);
  double abs_max  = (aa > bb) ? aa : bb;

  if (abs_max < DBL_MIN) abs_max = DBL_MIN;

  return (abs_diff / abs_max) <= (RELATIVE_ERROR_FACTOR * DBL_EPSILON);
}

}  // namespace math
}  // namespace common
}  // namespace jojo

#endif  //
