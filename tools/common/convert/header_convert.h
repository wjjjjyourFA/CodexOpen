#ifndef HEADER_CONVERT_H
#define HEADER_CONVERT_H
#include "tools/common/convert/message_convert_base.h"

// DDS
#include <dds/build/stable_msgs/base/Header.hpp>
// ROS2
#include <std_msgs/msg/header.hpp>

typedef std_msgs::msg::Header ros2_header_type;
typedef dds::base::msg::Header dds_header_type;

using namespace jojo::cyber::io;

template <>
class Ros2ToDdsBase<ros2_header_type, dds_header_type> {
 public:
  dds_header_type Transform(const ros2_header_type& ros2_msg) const override {
    dds_header_type dds_msg;
    dds_msg.frame_id() = ros2_msg.frame_id;
    dds_msg.stamp().sec() = ros2_msg.stamp.sec;
    dds_msg.stamp().nanosec() = ros2_msg.stamp.nanosec;
    return dds_msg;
  }
};

class DdsToRos2Header
    : public DdsToRos2Base<ros2_header_type, dds_header_type> {
 public:
  using DdsToRos2Base::DdsToRos2Base;

  ros2_header_type Transform(const dds_header_type& dds_msg) {
    ros2_header_type ros2_msg;
    ros2_msg.frame_id = dds_msg.frame_id();
    ros2_msg.stamp.sec = dds_msg.stamp().sec();
    ros2_msg.stamp.nanosec = dds_msg.stamp().nanosec();

    return ros2_msg;
  }
};

#endif  // HEADER_CONVERT_H
