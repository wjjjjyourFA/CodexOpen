// Minimal generated-message compatibility for rosbag files containing
// livox_ros_driver/CustomMsg. It is used only when that ROS package is absent.

#ifndef ROS1_SRC_TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_ROS1_LEGACY_LIVOX_CUSTOM_MSG_H_
#define ROS1_SRC_TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_ROS1_LEGACY_LIVOX_CUSTOM_MSG_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/builtin_message_traits.h>
#include <ros/serialization.h>
#include <ros/types.h>
#include <std_msgs/Header.h>

namespace livox_ros_driver {

struct CustomPoint {
  std::uint32_t offset_time = 0;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  std::uint8_t reflectivity = 0;
  std::uint8_t tag = 0;
  std::uint8_t line = 0;
};

struct CustomMsg {
  std_msgs::Header header;
  std::uint64_t timebase = 0;
  std::uint32_t point_num = 0;
  std::uint8_t lidar_id = 0;
  boost::array<std::uint8_t, 3> rsvd{{0, 0, 0}};
  std::vector<CustomPoint> points;

  using Ptr = boost::shared_ptr<CustomMsg>;
  using ConstPtr = boost::shared_ptr<const CustomMsg>;
};

using CustomMsgPtr = CustomMsg::Ptr;
using CustomMsgConstPtr = CustomMsg::ConstPtr;

}  // namespace livox_ros_driver

namespace ros {
namespace message_traits {

template <>
struct IsFixedSize<livox_ros_driver::CustomPoint> : TrueType {};
template <>
struct IsMessage<livox_ros_driver::CustomPoint> : TrueType {};
template <>
struct HasHeader<livox_ros_driver::CustomPoint> : FalseType {};
template <>
struct MD5Sum<livox_ros_driver::CustomPoint> {
  static const char* value() { return "109a3cc548bb1f96626be89a5008bd6d"; }
  static const char* value(const livox_ros_driver::CustomPoint&) {
    return value();
  }
};
template <>
struct DataType<livox_ros_driver::CustomPoint> {
  static const char* value() { return "livox_ros_driver/CustomPoint"; }
  static const char* value(const livox_ros_driver::CustomPoint&) {
    return value();
  }
};
template <>
struct Definition<livox_ros_driver::CustomPoint> {
  static const char* value() {
    return "uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\n"
           "uint8 reflectivity\nuint8 tag\nuint8 line\n";
  }
};

template <>
struct IsFixedSize<livox_ros_driver::CustomMsg> : FalseType {};
template <>
struct IsMessage<livox_ros_driver::CustomMsg> : TrueType {};
template <>
struct HasHeader<livox_ros_driver::CustomMsg> : TrueType {};
template <>
struct MD5Sum<livox_ros_driver::CustomMsg> {
  static const char* value() { return "e4d6829bdfe657cb6c21a746c86b21a6"; }
  static const char* value(const livox_ros_driver::CustomMsg&) {
    return value();
  }
};
template <>
struct DataType<livox_ros_driver::CustomMsg> {
  static const char* value() { return "livox_ros_driver/CustomMsg"; }
  static const char* value(const livox_ros_driver::CustomMsg&) {
    return value();
  }
};
template <>
struct Definition<livox_ros_driver::CustomMsg> {
  static const char* value() {
    return "Header header\nuint64 timebase\nuint32 point_num\n"
           "uint8 lidar_id\nuint8[3] rsvd\nCustomPoint[] points\n";
  }
};

}  // namespace message_traits

namespace serialization {

template <>
struct Serializer<livox_ros_driver::CustomPoint> {
  template <typename Stream, typename Message>
  inline static void allInOne(Stream& stream, Message message) {
    stream.next(message.offset_time);
    stream.next(message.x);
    stream.next(message.y);
    stream.next(message.z);
    stream.next(message.reflectivity);
    stream.next(message.tag);
    stream.next(message.line);
  }
  ROS_DECLARE_ALLINONE_SERIALIZER
};

template <>
struct Serializer<livox_ros_driver::CustomMsg> {
  template <typename Stream, typename Message>
  inline static void allInOne(Stream& stream, Message message) {
    stream.next(message.header);
    stream.next(message.timebase);
    stream.next(message.point_num);
    stream.next(message.lidar_id);
    stream.next(message.rsvd);
    stream.next(message.points);
  }
  ROS_DECLARE_ALLINONE_SERIALIZER
};

}  // namespace serialization
}  // namespace ros

#endif  // ROS1_SRC_TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_ROS1_LEGACY_LIVOX_CUSTOM_MSG_H_
