#include "tools/common/utils/rclcpp_utils.h"

#include <limits>
#include <stdexcept>

namespace jojo {
namespace tools {

uint64_t toMs(const builtin_interfaces::msg::Time& t) {
  // return static_cast<uint64_t>(t.sec) * 1000ULL +
  //        static_cast<uint64_t>(t.nanosec) / 1000000ULL;
  return rclcpp::Time(t).nanoseconds() / 1000000ULL;
}

double toSec(const builtin_interfaces::msg::Time& t) {
  return rclcpp::Time(t).seconds();
}

rclcpp::Time fromMs(uint64_t milliseconds) {
  constexpr uint64_t kNanosecondsPerMillisecond = 1000000ULL;
  constexpr uint64_t kMaxMilliseconds =
      static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) /
      kNanosecondsPerMillisecond;
  if (milliseconds > kMaxMilliseconds) {
    throw std::overflow_error("ROS timestamp exceeds int64 nanosecond range");
  }
  const uint64_t nanoseconds = milliseconds * kNanosecondsPerMillisecond;
  return rclcpp::Time(static_cast<int64_t>(nanoseconds));
}

}  // namespace tools
}  // namespace jojo
