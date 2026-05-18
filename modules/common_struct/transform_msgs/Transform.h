#pragma once

#include <string>

#include "modules/common_struct/basic_msgs/Header.h"
#include "modules/common_struct/basic_msgs/Pose6D.h"

struct Transform {
  Header header;

  Pose6D pose;
};

struct TransformStamped {
  Header header;
  // header.frame_id = "map"

  // child_frame_id   = "base_link"
  std::string child_frame_id;

  Pose6D pose;

  // 构造函数
  TransformStamped() = default;
  TransformStamped(const Header& h, const std::string& child, const Pose6D& p)
      : header(h), child_frame_id(child), pose(p) {}

  // 输出为字符串（调试用）
  std::string ToString() const {
    char buf[256];
    snprintf(buf, sizeof(buf),
             "[%s] x=%.2f y=%.2f z=%.2f yaw=%.2f pitch=%.2f roll=%.2f time=%ld",
             child_frame_id.c_str(), pose.x, pose.y, pose.z, pose.azimuth,
             pose.pitch, pose.roll, static_cast<long>(header.timestamp));
    return std::string(buf);
  }
};
