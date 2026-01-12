#ifndef MESSAGE_CONVERT_BASE_H
#define MESSAGE_CONVERT_BASE_H
#include "cyber/io/publish_msg_wrapper.h"
#include "cyber/io/subscribe_msg_wrapper.h"

using namespace jojo::cyber::io;

template <typename ROS2Type, typename DDSType>
class Ros2ToDdsBase : public SubscribeROS2Msg<ROS2Type>,
                      public PublishDDSMsg<DDSType> {
 public:
  Ros2ToDdsBase(const std::shared_ptr<rclcpp::Node>& rcl_node,
                const std::string& sub_topic_name,
                uint32_t domain_id,
                const std::string& pub_topic_name,
                bool zcc_flag = false)
      : SubscribeROS2Msg<ROS2Type>(rcl_node, sub_topic_name, sub_topic_name),
        PublishDDSMsg<DDSType>(domain_id, pub_topic_name, zcc_flag) {
    auto func = [this](const ROS2Type& ros2_msg) {
      DDSType dds_msg = Transform(ros2_msg);

      this->Publish(dds_msg);
    };

    // 设置回调函数
    this->SetOptionalCallbackFunc(func);
  }

  virtual DDSType Transform(const ROS2Type& ros2_msg) const {
    static_assert(sizeof(ROS2Type) == 0,
                  "Transform function must be implemented for specific types");
  }

  virtual ~Ros2ToDdsBase() = default;
};

template <typename DDSType, typename ROS2Type>
class DdsToRos2Base : public SubscribeDDSMsg<DDSType>,
                      public PublishROS2Msg<ROS2Type> {
 public:
  DdsToRos2Base(const std::shared_ptr<rclcpp::Node>& rcl_node,
                const std::string& sub_topic_name,
                uint32_t domain_id,
                const std::string& pub_topic_name,
                bool zcc_flag = false)
      : SubscribeDDSMsg<DDSType>(sub_topic_name,
                                 sub_topic_name,
                                 domain_id,
                                 zcc_flag),
        PublishROS2Msg<ROS2Type>(rcl_node, pub_topic_name) {
    auto func = [this](const DDSType& dds_msg) {
      ROS2Type ros2_msg = Transform(dds_msg);

      this->Publish(ros2_msg);
    };

    // 设置回调函数
    this->SetOptionalCallbackFunc(func);
  }

  virtual ROS2Type Transform(const DDSType& dds_msg) const {
    static_assert(sizeof(DDSType) == 0,
                  "Transform function must be implemented for specific types");
  }
};

#endif  // MESSAGE_CONVERT_BASE_H