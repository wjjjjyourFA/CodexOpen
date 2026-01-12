#include "modules/common/math/math_utils_extra.h"

namespace jojo {
namespace common {
namespace math {

double distSquare(const double x1, const double y1, const double x2,
                  const double y2) {
  return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
}

double dist(const double x1, const double y1, const double x2,
            const double y2) {
  return sqrt(distSquare(x1, y1, x2, y2));
}

double NormalizeAngle0To2Pi(double angle) {
  angle = fmod(angle, 2 * M_PI);
  return angle >= 0 ? angle : angle + M_2_1_PI;
}

float normalize_angle(const float angle) {
  float a = std::fmod(angle + 180.0f, 360.0f);
  if (a < 0.0f) {
    a += 360.0f;
  }
  return a - 180.0f;
}

float normalize_angle_0to360(const float angle) {
  float a = std::fmod(angle, 360.0f);
  return a >= 0.0f ? a : a + 360.0f;
}

float angle_diff(const float from, const float to) {
  return normalize_angle(to - from);
}

}  // namespace math
}  // namespace common
}  // namespace jojo
