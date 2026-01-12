#ifndef LOG_EXTRA_H
#define LOG_EXTRA_H

#pragma once

#include <string>

#include "modules/common/environment_conf.h"

#if defined(ENABLE_ROS1)
#include <ros/ros.h>
#elif defined(ENABLE_ROS2)
#include <rclcpp/rclcpp.hpp>
#elif defined(ENABLE_DDS)
#else
#error                                                                         \
    "No logging system defined. Please define ENABLE_ROS1, ENABLE_ROS2, or ENABLE_DDS."
#endif

namespace jojo {
namespace common {
namespace logger {

#if defined(ENABLE_ROS1)
/* ROS1 日志封装
代码在 ROS 1 中没有报错的原因：
ROS_ERROR 本身不需要 logger 参数。
宏定义中的 logger 参数仅仅是占位符，调用时它会被忽略。
*/
#define LOG_ERROR(logger, fmt, ...) ROS_ERROR(fmt, ##__VA_ARGS__)
#define LOG_WARN(logger, fmt, ...) ROS_WARN(fmt, ##__VA_ARGS__)
#define LOG_INFO(logger, fmt, ...) ROS_INFO(fmt, ##__VA_ARGS__)
#define LOG_DEBUG(logger, fmt, ...) ROS_DEBUG(fmt, ##__VA_ARGS__)
#define LOG_INFO_COLOR(text, ...)                                              \
  ROS_INFO("\033[1;32m" fmt "\033[0m", ##__VA_ARGS__)

#elif defined(ENABLE_ROS2)
// ROS2 日志封装
#define LOG_ERROR(logger, fmt, ...)                                            \
  RCLCPP_ERROR(rclcpp::get_logger("~"), fmt, ##__VA_ARGS__)
#define LOG_WARN(logger, fmt, ...)                                             \
  RCLCPP_WARN(rclcpp::get_logger("~"), fmt, ##__VA_ARGS__)
#define LOG_INFO(logger, fmt, ...)                                             \
  RCLCPP_INFO(rclcpp::get_logger("~"), fmt, ##__VA_ARGS__)
#define LOG_DEBUG(logger, fmt, ...)                                            \
  RCLCPP_DEBUG(rclcpp::get_logger("~"), fmt, ##__VA_ARGS__)
#define LOG_INFO_COLOR(logger, fmt, ...)                                       \
  RCLCPP_INFO(logger, "\033[1;32m" fmt "\033[0m", ##__VA_ARGS__)

#elif defined(ENABLE_DDS)
// DDS 日志封装（根据具体实现添加， 还没有真正实现）
/* Add DDS-specific error logging here */
#define LOG_ERROR(logger, fmt, ...)
/* Add DDS-specific warning logging here */
#define LOG_WARN(logger, fmt, ...)
/* Add DDS-specific info logging here */
#define LOG_INFO(logger, fmt, ...)
/* Add DDS-specific debug logging here */
#define LOG_DEBUG(logger, fmt, ...)

#else
#error                                                                         \
    "No logging backend defined. Define ENABLE_ROS1, ENABLE_ROS2, or ENABLE_DDS."
#endif

/* 业务代码
#ifdef ENABLE_ROS2
auto logger = rclcpp::get_logger("function");
#else
void* logger = nullptr;  // ROS1 占位
#endif

LOG_INFO(logger, "Init success, id=%d", id);
LOG_WARN(logger, "Latency %.2f ms", latency);
*/

}  // namespace logger
}  // namespace common
}  // namespace jojo

#endif
