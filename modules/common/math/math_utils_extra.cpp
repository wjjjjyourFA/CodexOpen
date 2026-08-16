#include "modules/common/math/math_utils_extra.h"

#include <limits>

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

namespace {

std::uint64_t TimestampDistance(std::int64_t lhs, std::int64_t rhs) {
  return lhs >= rhs ? static_cast<std::uint64_t>(lhs) -
                          static_cast<std::uint64_t>(rhs)
                    : static_cast<std::uint64_t>(rhs) -
                          static_cast<std::uint64_t>(lhs);
}

}  // namespace

int FindNearestTimestampIdx(int64_t query_time,
                            const std::vector<int64_t>& sorted_time_db) {
  if (sorted_time_db.empty()) return -1;

  auto it = std::lower_bound(sorted_time_db.begin(), sorted_time_db.end(),
                             query_time);

  if (it == sorted_time_db.begin()) return 0;
  if (it == sorted_time_db.end()) {
    const std::size_t last = sorted_time_db.size() - 1U;
    return last <= static_cast<std::size_t>(std::numeric_limits<int>::max())
               ? static_cast<int>(last)
               : -1;
  }

  const std::size_t next_idx =
      static_cast<std::size_t>(it - sorted_time_db.begin());
  const std::size_t prev_idx = next_idx - 1U;
  const std::size_t result =
      TimestampDistance(query_time, sorted_time_db[prev_idx]) <=
              TimestampDistance(sorted_time_db[next_idx], query_time)
          ? prev_idx
          : next_idx;

  return result <= static_cast<std::size_t>(std::numeric_limits<int>::max())
             ? static_cast<int>(result)
             : -1;
}

}  // namespace math
}  // namespace common
}  // namespace jojo
