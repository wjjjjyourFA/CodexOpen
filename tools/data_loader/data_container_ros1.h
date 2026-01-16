#ifndef DATA_CONTAINER_ROS1_HH
#define DATA_CONTAINER_ROS1_HH

#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"

#include "tools/data_loader/data_container.h"

namespace jojo {
namespace tools {

// 默认 publisher 类型
template <typename T>
class DataContainerRos1 : public DataContainer<T> {
 public:
  DataContainerRos1() = default;

  ros::Publisher pub;
  ros::Timer timer;

  void stop() override {
    this->timer.stop();
    ros::shutdown();
  }

  // 非 ros 消息，不能使用这个
  void publish() override { pub.publish(this->cur_data); }
};

// 特殊处理 sensor_msgs::ImagePtr 类型 ==> image_transport::Publisher
template <>
class DataContainerRos1<sensor_msgs::ImagePtr>
    : public DataContainer<sensor_msgs::ImagePtr> {
 public:
  DataContainerRos1() = default;

  image_transport::Publisher pub;
  ros::Timer timer;

  void stop() override {
    this->timer.stop();
    ros::shutdown();
  }

  void publish() override { pub.publish(this->cur_data); }
};

// 特殊处理 uint64_t 时间戳，临时读取数据并发布
template <>
class DataContainerRos1<uint64_t> : public DataContainer<uint64_t> {
 public:
  DataContainerRos1() = default;

  ros::Publisher pub;
  ros::Timer timer;

  void stop() override {
    this->timer.stop();
    ros::shutdown();
  }

  // 非 ros 消息，需要手动 pub
  void publish() override {
    std::cerr << "can not publish uint64_t" << std::endl;
  }
};

}  // namespace tools
}  // namespace jojo

#endif
