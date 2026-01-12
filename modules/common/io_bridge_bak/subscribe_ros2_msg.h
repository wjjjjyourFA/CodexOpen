#ifndef SUBSCRIBE_ROS2_MSG_H
#define SUBSCRIBE_ROS2_MSG_H
#include "cyber/io_bridge/sub_msg_base.h"

#ifdef ENABLE_ROS2
#include "rclcpp/rclcpp.hpp"

namespace jojo {
namespace cyber {
namespace io_bridge {

template <typename msg_type>
class SubscribeROS2Msg : public SubscribeMsgBase<msg_type> {
 public:
  using msg_type_ptr = typename msg_type::SharedPtr;

  // 声明自身的构造函数，然后调用基类 SubscribeMsgBase
  // 的构造函数，初始化基类的一些成员。
  SubscribeROS2Msg(const std::shared_ptr<rclcpp::Node> &rcl_node,
                   const std::string& topic_name,
                   const std::string& msg_info,
                   const rclcpp::QoS &qos = 10)
      : SubscribeMsgBase<msg_type>(topic_name, msg_info) {
    subscription_ = rcl_node->create_subscription<msg_type>(
        topic_name, qos,
        [this](const msg_type_ptr msg) { this->Callback(msg); });
  }

 protected:
  virtual void Callback(const msg_type_ptr msg);

  virtual void PrintCallbackInfo() {}

 private:
  // typename
  // 关键字通常用于模板编程中，用于标识某个名称是一个类型而不是变量或函数。
  typename rclcpp::Subscription<msg_type>::SharedPtr subscription_;
};

// 以下是类中函数的实现，==> .cpp
template <typename msg_type>
void SubscribeROS2Msg<msg_type>::Callback(const msg_type_ptr msg) {
  std::lock_guard<std::mutex> lock_gard(this->msg_mutex_);

  this->last_msg_time_ = MsgTime::GetSystemTime();

  // msg 是一个指向消息的智能指针，通过
  // std::move(*msg)，我们将消息的内容移动到类成员变量 msg_ 中。
  // 这样，类就可以在后续的操作中使用 msg_ 来访问接收到的消息。
  this->msg_ = std::move(*msg);

  this->optional_callback_func_(this->msg_);

  // 强制确保只接收到最新的消息
  // 至于处理新消息还是旧消息，交给后面的部分做
  this->new_signal_ = true;

  if (!this->receive_flag_) {
    this->receive_flag_ = true;
    std::cout << "ROS2 SUB ----> get " << this->msg_info_ << std::endl;
  }

  PrintCallbackInfo();
}

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#else
namespace jojo {
namespace cyber {
namespace io_bridge {

template <typename msg_type>
class SubscribeROS2Msg : public SubscribeMsgBase<msg_type> {};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // ENABLE_ROS2

#endif  // SUBSCRIBE_ROS2_MSG_H
