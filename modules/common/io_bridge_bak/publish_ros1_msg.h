#ifndef PUBLISH_ROS1_MSG_H
#define PUBLISH_ROS1_MSG_H
#include "cyber/io_bridge/pub_msg_base.h"

#ifdef ENABLE_ROS1
#include "ros/ros.h"
#include "ros/init.h"

namespace jojo {
namespace cyber {
namespace io_bridge {

/*模板继承*/
// 20241125 std::unique_ptr <==> std::shared_ptr is ok
template <typename msg_type>
class PublishROS1Msg : public PublishMsgBase<msg_type> {
 public:
  PublishROS1Msg(const std::shared_ptr<ros::NodeHandle>& handle_ptr,
                 const std::string& topic_name, const uint32_t& queue_size);
  virtual ~PublishROS1Msg() {}

  virtual void Publish(const msg_type& msg);

 private:
  ros::Publisher publisher_;
};
// 以下是类中函数的实现，==> .cpp

// 构造函数
template <typename msg_type>
PublishROS1Msg<msg_type>::PublishROS1Msg(
    const std::shared_ptr<ros::NodeHandle>& handle_ptr,
    const std::string& topic_name, const uint32_t& queue_size) {
  publisher_ = handle_ptr->advertise<msg_type>(topic_name, queue_size);
};

template <typename msg_type>
void PublishROS1Msg<msg_type>::Publish(const msg_type& msg) {
  publisher_.publish(msg);
};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // ENABLE_ROS1

#endif  // PUBLISH_ROS1_MSG_H
