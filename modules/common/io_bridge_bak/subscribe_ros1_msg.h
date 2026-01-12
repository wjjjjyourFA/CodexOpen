#ifndef SUBSCRIBE_ROS1_MSG_H
#define SUBSCRIBE_ROS1_MSG_H
#include "cyber/io_bridge/sub_msg_base.h"

#ifdef ENABLE_ROS1
#include "ros/ros.h"
#include "ros/init.h"

namespace jojo {
namespace cyber {
namespace io_bridge {

template <typename msg_type>
class SubscribeROS1Msg : public SubscribeMsgBase<msg_type> {
 public:
  SubscribeROS1Msg(const std::shared_ptr<ros::NodeHandle> &handle_ptr,
                   const std::string &topic_name, const std::string &msg_info,
                   uint32_t queue_size);

 protected:
  virtual void Callback(const boost::shared_ptr<msg_type const> &msg);

  virtual void PrintCallbackInfo() {}

 private:
  ros::Subscriber subscriber_;
};

/********************* function of the base class
 * ***************************************/
// 以下是类中函数的实现，==> .cpp

template <typename msg_type>
SubscribeROS1Msg<msg_type>::SubscribeROS1Msg(
    const std::shared_ptr<ros::NodeHandle> &handle_ptr,
    const std::string &topic_name, const std::string &msg_info,
    uint32_t queue_size)
    : SubscribeMsgBase<msg_type>(topic_name, msg_info) {
  subscriber_ = handle_ptr->subscribe<msg_type>(
      topic_name, queue_size,
      boost::bind(&SubscribeROS1Msg<msg_type>::Callback, this, _1));
}

template <typename msg_type>
void SubscribeROS1Msg<msg_type>::Callback(
    const boost::shared_ptr<msg_type const> &msg) {
  std::lock_guard<std::mutex> lock_gard(this->msg_mutex_);

  this->last_msg_time_ = MsgTime::GetSystemTime();

  this->msg_ = std::move(*msg);

  // 用于外部程序在回调函数中设置如何处理数据
  this->optional_callback_func_(this->msg_);

  this->new_signal_ = true;

  if (!this->receive_flag_) {
    this->receive_flag_ = true;
    std::cout << "ROS1 SUB ----> get " << this->msg_info_ << std::endl;
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
class SubscribeROS1Msg : public SubscribeMsgBase<msg_type> {};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // ENABLE_ROS1

#endif  // SUBSCRIBE_ROS1_MSG_H
