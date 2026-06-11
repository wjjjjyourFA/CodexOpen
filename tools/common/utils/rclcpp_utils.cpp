#include "tools/common/utils/rclcpp_utils.h"

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

}  // namespace tools
}  // namespace jojo
