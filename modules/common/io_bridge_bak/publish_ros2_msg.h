/*
 * @Author: HuangWei
 * @Date: 2024-04-18 10:33:40
 * @LastEditors: HuangWei
 * @LastEditTime: 2024-04-18 10:33:41
 * @FilePath: CodexOpen/cyber/io_bridge/publish_ros2_msg.h
 * @Description: 
 * 
 * Copyright (c) 2024 by JOJO && YanDan, All Rights Reserved. 
 */
#ifndef PUBLISH_ROS2_MSG_H
#define PUBLISH_ROS2_MSG_H
#include "cyber/io_bridge/pub_msg_base.h"

#ifdef ENABLE_ROS2
#include "rclcpp/rclcpp.hpp"

namespace jojo {
namespace cyber {
namespace io_bridge {

// 在ROS1的基础上，将函数实现归纳在一起
template <typename msg_type>
class PublishROS2Msg : public PublishMsgBase<msg_type> {
 public:
  // The Quality of Service settings for the publisher.
  // qos 默认为 10（代表可靠传输）==> 不是发布频率
  PublishROS2Msg(const std::shared_ptr<rclcpp::Node>& rcl_node,
                 const std::string& topic_name,
                 const rclcpp::QoS& qos = 10) {
    publisher_ = rcl_node->create_publisher<msg_type>(topic_name, qos);
  }
  virtual ~PublishROS2Msg() {}

  virtual void Publish(const msg_type& msg) { publisher_->publish(msg); }

 private:
  // typename ==> publisher_ 是一个指向 rclcpp::Publisher<msg_type>
  // 类型对象的共享指针
  typename rclcpp::Publisher<msg_type>::SharedPtr publisher_;
};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // ENABLE_ROS2

#endif  // PUBLISH_ROS2_MSG_H
