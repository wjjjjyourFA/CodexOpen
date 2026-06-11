#pragma once

#include <rclcpp/rclcpp.hpp>

namespace jojo {
namespace tools {

uint64_t toMs(const builtin_interfaces::msg::Time& t);

double toSec(const builtin_interfaces::msg::Time& t);

}  // namespace tools
}  // namespace jojo
