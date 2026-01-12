/*
 * @Author: HuangWei
 * @Date: 2024-02-21 18:19:08
 * @LastEditors: HuangWei
 * @LastEditTime: 2024-04-18 10:48:01
 * @FilePath: CodexOpen/cyber/io_bridge/publish_msg_wrapper.h
 * @Description: 
 * 
 * Copyright (c) 2024 by JOJO && YanDan, All Rights Reserved. 
 */
#ifndef PUBLISH_MSG_WRAPPER_H
#define PUBLISH_MSG_WRAPPER_H
#include "cyber/io_bridge/publish_dds_msg.h"
#include "cyber/io_bridge/publish_ros1_msg.h"
#include "cyber/io_bridge/publish_ros2_msg.h"

namespace jojo {
namespace cyber {
namespace io_bridge {

template <typename msg_type>
class PublishMsgWrapper {
 public:
  PublishMsgWrapper() {}
  ~PublishMsgWrapper() {}

#ifdef ENABLE_ROS1
  // publish for ROS1
  void InitROS1(const std::shared_ptr<ros::NodeHandle>& handle_ptr,
                const std::string& topic_name, const uint32_t& queue_size = 10) {
    SetNodeType(ROS1_NODE);
    pub_handle_ = 
	  std::make_shared<PublishROS1Msg<msg_type>>(handle_ptr, topic_name, queue_size);
    topic_name_ = topic_name;
  }
#endif  // ENABLE_ROS1

#ifdef ENABLE_ROS2
  // publish for ROS2
  void InitROS2(const std::shared_ptr<rclcpp::Node>& rcl_node,
                const std::string& topic_name, const rclcpp::QoS& qos = 10) {
    SetNodeType(ROS2_NODE);
    pub_handle_ =
        std::make_shared<PublishROS2Msg<msg_type>>(rcl_node, topic_name, qos);
    topic_name_ = topic_name;
  }
#endif  // ENABLE_ROS

#ifdef ENABLE_DDS
  // publish for DDS
  void InitDDS(uint32_t domain_id, const std::string& topic_name,
               bool zcc_flag = false, const std::string& network_interface = "") {
    if (zcc_flag) {
      SetNodeType(DDS_ZCC_NODE);
    } else {
      SetNodeType(DDS_NET_NODE);
    }

    pub_handle_ = 
	  std::make_shared<PublishDDSMsg<msg_type>>(domain_id, topic_name, zcc_flag, network_interface);
    topic_name_ = topic_name;
  }
#endif  // ENABLE_DDS

  void Publish(const msg_type& msg) { pub_handle_->Publish(msg); }

  NodeType GetNodeType() { return node_type_; }

  std::string topic_name_ = "";

 protected:
  void SetNodeType(NodeType node_type) { node_type_ = node_type; }
  std::shared_ptr<PublishMsgBase<msg_type>> pub_handle_;
  NodeType node_type_;
};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // PUBLISH_MSG_WRAPPER_H
