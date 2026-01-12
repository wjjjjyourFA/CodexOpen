#ifndef UNIT_CONVERTER_H
#define UNIT_CONVERTER_H

#include <cmath>
#include <cstdint>
#include <limits>

namespace jojo {
namespace common {
namespace math {

class UnitConverter {
 public:
  UnitConverter();
  virtual ~UnitConverter() {};

  // 用于支持在编译期和运行期计算
  /// Internal representation of pi
  static double PI;  // π的常数值

  /// Internal representation of pi/2
  static double RAW_PI_2;  // π/2

  /// Used for converting angle units
  static double DEG_TO_RAD;

  /// Used for converting angle units
  static double RAD_TO_DEG;

  // ---- 角度 ↔ 弧度 ----
  // 将角度转换为弧度（角度转为弧度）
  static inline double angle_to_rad(double deg) { return deg * DEG_TO_RAD; }

  // 将弧度转换为角度（弧度转为角度）
  static inline double rad_to_angle(double rad) { return rad * RAD_TO_DEG; }

  // ---- 角度 ↔ 毫弧度（0.001 rad）----
  static inline int32_t rad_to_millirad(double rad) {
    return static_cast<int32_t>(std::llround(rad * 1000.0));
  }

  static inline int32_t angle_to_millirad(double deg) {
    return static_cast<int32_t>(std::llround(angle_to_rad(deg) * 1000.0));
  }

  static inline double millirad_to_rad(int millirad) {
    return static_cast<double>(millirad) / 1000.0;
  }

  static inline double millirad_to_angle(int millirad) {
    return static_cast<double>(rad_to_angle(millirad_to_rad(millirad)));
  }

  // ---- 米 ↔ 厘米 ----
  static inline int32_t meters_to_cm(double meters) {
    return static_cast<int32_t>(std::llround(meters * 100.0));
  }

  static inline double cm_to_meters(double cm) { return cm / 100.0; }

  // ---- 米 ↔ 毫米 ----
  static inline int32_t meters_to_mm(double meters) {
    return static_cast<int32_t>(std::llround(meters * 1000.0));
  }

  static inline double mm_to_meters(int mm) {
    return static_cast<double>(mm) / 1000.0;
  }

  // ---- 经纬度 ↔ x1e7 精度整数 ----
  static inline int32_t latlon_to_int(double deg) {
    return static_cast<int32_t>(std::llround(deg * 1e7));
  }

  static inline double int_to_latlon(int encoded) {
    return static_cast<double>(encoded) / 1e7;
  }
};

}  // namespace math
}  // namespace common
}  // namespace jojo

#endif