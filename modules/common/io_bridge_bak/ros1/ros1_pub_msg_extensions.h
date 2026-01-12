#ifndef ROS1_PUB_MSG_EXTENSIONS_H
#define ROS1_PUB_MSG_EXTENSIONS_H
#include "cyber/io_bridge/publish_ros1_msg.h"

#ifdef ENABLE_ROS1
// 20241122 派生类 image_transport
#include <image_transport/image_transport.h>
// #include <image_transport/transport_hints.h>

namespace jojo {
namespace cyber {
namespace io_bridge {

/*模板特化
 * 在这里不是用模板继承，后续处理会继续对基础模板进行继承，如果这里使用模板继承，会导致多层继承
 * 1.通用模板中的成员变量或成员函数不会自动出现在特化中。
 * 2.如果特化版本需要这些成员变量或函数，必须显式定义。*/
template <>
class PublishROS1Msg<sensor_msgs::Image>
    : public PublishMsgBase<sensor_msgs::Image> {
 public:
  PublishROS1Msg(const std::shared_ptr<ros::NodeHandle>& handle_ptr,
                 const std::string& topic_name, const uint32_t queue_size);
  virtual ~PublishROS1Msg() {}

  virtual void Publish(const sensor_msgs::Image& msg);

 private:
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher publisher_;
};

template <>
PublishROS1Msg<sensor_msgs::Image>::PublishROS1Msg(
    const std::shared_ptr<ros::NodeHandle>& handle_ptr,
    const std::string& topic_name, const uint32_t queue_size) {
  it_        = std::make_shared<image_transport::ImageTransport>(*handle_ptr);
  publisher_ = it_->advertise(topic_name, queue_size);
};

template <>
void PublishROS1Msg<sensor_msgs::Image>::Publish(
    const sensor_msgs::Image& msg) {
  publisher_.publish(msg);
};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // ENABLE_ROS1

#endif  // ROS1_PUB_WRAPPER_EXTENSIONS_H
