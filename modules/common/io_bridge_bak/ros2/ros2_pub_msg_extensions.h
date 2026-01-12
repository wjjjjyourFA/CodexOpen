#ifndef ROS2_PUB_MSG_EXTENSIONS_H
#define ROS2_PUB_MSG_EXTENSIONS_H
#include "cyber/io_bridge/publish_ros2_msg.h"

#ifdef ENABLE_ROS2
// #include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

namespace jojo {
namespace cyber {
namespace io_bridge {

/*模板特化*/
template <>
class PublishROS2Msg<sensor_msgs::msg::Image>
    : public PublishMsgBase<sensor_msgs::msg::Image> {
 public:
  // The Quality of Service settings for the publisher.
  PublishROS2Msg(const std::shared_ptr<rclcpp::Node>& rcl_node,
                 const std::string& topic_name, const rclcpp::QoS& qos = 10) {
    /* way 1
    it         = std::make_shared<image_transport::ImageTransport>(rcl_node);
    publisher_ = it->advertise(topic_name, qos);
    */

    // way 2
    const rmw_qos_profile_t& rmw_qos = qos.get_rmw_qos_profile();
    publisher_ =
        image_transport::create_publisher(rcl_node.get(), topic_name, rmw_qos);

    // RCLCPP_INFO(
    //     rcl_node->get_logger(),
    //     "Specialized template for sensor_msgs::msg::Image is being used.");
  }

  virtual void Publish(const sensor_msgs::msg::Image& msg) {
    if (publisher_.getTopic() != "") {  // 确保 publisher 有效
      publisher_.publish(msg);
    } else {
      throw std::runtime_error(
          "ros2 image_transport publisher is not initialized or topic is "
          "empty.");
    }
  }

 private:
  // way 1
  // std::shared_ptr<image_transport::ImageTransport> it;
  image_transport::Publisher publisher_;
};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // ENABLE_ROS2

#endif  // ROS2_PUB_MSG_EXTENSIONS_H
