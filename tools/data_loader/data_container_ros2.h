#ifndef DATA_CONTAINER_ROS2_HH
#define DATA_CONTAINER_ROS2_HH

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.h>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "tools/data_loader/data_container.h"

namespace jojo {
namespace tools {

// 默认 publisher 类型
template <typename T>
class DataContainerRos2 : public DataContainer<T> {
 public:
  DataContainerRos2() = default;

  // rclcpp::PublisherBase::SharedPtr pub;
  typename rclcpp::Publisher<T>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;

  void stop() override {
    this->timer->cancel();
    rclcpp::shutdown();
  }

  // 非 ros 消息，不能使用这个
  void publish() override { pub->publish(this->cur_data); }
};

// 特殊处理 sensor_msgs::ImagePtr 类型 ==> image_transport::Publisher
template <>
class DataContainerRos2<sensor_msgs::msg::Image::SharedPtr>
    : public DataContainer<sensor_msgs::msg::Image::SharedPtr> {
 public:
  DataContainerRos2() = default;

  image_transport::Publisher pub;
  rclcpp::TimerBase::SharedPtr timer;

  void stop() override {
    this->timer->cancel();
    rclcpp::shutdown();
  }

  void publish() override { pub.publish(this->cur_data); }
};

// 特殊处理 uint64_t 时间戳，临时读取数据并发布
template <>
class DataContainerRos2<uint64_t> : public DataContainer<uint64_t> {
 public:
  DataContainerRos2() = default;

  typename rclcpp::PublisherBase::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;

  void stop() override {
    this->timer->cancel();
    rclcpp::shutdown();
  }

  // 非 ros 消息，需要手动 pub
  void publish() override {
    std::cerr << "can not publish uint64_t" << std::endl;
  }
};

}  // namespace tools
}  // namespace jojo

#endif
