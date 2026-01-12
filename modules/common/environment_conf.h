#ifndef ENVIRONMENT_CONF_H
#define ENVIRONMENT_CONF_H

#pragma once

#include <string>
#include <algorithm>
#include <iostream>

#define ENABLE_ROS1
// #define ENABLE_ROS2
// #define ENABLE_DDS

#define RSLIDAR_OLD
// #define RSLIDAR_NEW
// #define VELODYNE

#if defined(ENABLE_ROS2)
inline std::string GetTopicToParamName(const std::string& topic) {
  std::string param_name = topic;
  // 去掉开头的 '/'（可选）
  if (!param_name.empty() && param_name[0] == '/') {
    param_name.erase(0, 1);
  }
  // 替换 '/' 为 '.'
  std::replace(param_name.begin(), param_name.end(), '/', '.');
  return param_name;
}
#endif

#endif  // ENVIRONMENT_CONF_H
