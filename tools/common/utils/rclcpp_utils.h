#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>

namespace jojo {
namespace tools {

uint64_t toMs(const builtin_interfaces::msg::Time& t);

double toSec(const builtin_interfaces::msg::Time& t);

rclcpp::Time fromMs(uint64_t milliseconds);

}  // namespace tools
}  // namespace jojo
