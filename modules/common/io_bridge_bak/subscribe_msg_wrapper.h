#ifndef SUBSCRIBE_MSG_WRAPPER_H
#define SUBSCRIBE_MSG_WRAPPER_H
#include <condition_variable>
#include <mutex>
#include <queue>

#include "cyber/io_bridge/subscribe_dds_msg.h"
#include "cyber/io_bridge/subscribe_ros1_msg.h"
#include "cyber/io_bridge/subscribe_ros2_msg.h"

namespace jojo {
namespace cyber {
namespace io_bridge {

// 为什么这里三个都要写？
// 在 C++
// 模板中，对于依赖于模板参数的嵌套类型，编译器无法确定它是否是一个类型名称，还是一个静态成员变量或函数。
// 需要使用 typename 关键字来明确告诉编译器，这是一个类型而不是其他类型的成员。
template <typename msg_type,
          typename sub_ros1_handle = SubscribeROS1Msg<msg_type>,
          typename sub_ros2_handle = SubscribeROS2Msg<msg_type>,
          typename sub_dds_handle = SubscribeDDSMsg<msg_type>>
class SubscribeMsgWrapper {
 public:
  SubscribeMsgWrapper() {}
  ~SubscribeMsgWrapper() {}

#ifdef ENABLE_ROS1
  // subscribe from ROS1
  void InitROS1(const std::shared_ptr<ros::NodeHandle>& handle_ptr,
                const std::string& topic_name,
                const std::string& msg_info,
                uint32_t queue_size = 10);
#endif  // ENABLE_ROS1

#ifdef ENABLE_ROS2
  // subscribe from ROS2
  void InitROS2(const std::shared_ptr<rclcpp::Node>& rcl_node,
                const std::string& topic_name,
                const std::string& msg_info,
                const rclcpp::QoS& qos = 10);
#endif  // ENABLE_ROS2

#ifdef ENABLE_DDS
  // subscribe from DDS
  void InitDDS(uint32_t domain_id,
               const std::string& topic_name,
               const std::string& msg_info,
               bool zcc_flag = false);
#endif  // ENABLE_DDS

  // 封装
  msg_type GetMsg() { return sub_handle_->GetMsg(); }

  bool GetNewMsg(msg_type& msg) { return sub_handle_->GetNewMsg(msg); }

  double GetLastMsgTime() { return sub_handle_->GetLastMsgTime(); }

  bool Got() { return sub_handle_->Got(); }

  void Clear() { sub_handle_->Clear(); }

  void Check(int miss_time_diff) { sub_handle_->Check(miss_time_diff); }

  void SetCallbackFunc(const std::function<void(const msg_type& msg)>& func){
    sub_handle_->SetOptionalCallbackFunc(func);
  }

  NodeType GetNodeType() { return node_type_; }
  
  std::string topic_name_ = "";

 protected:
  void SetNodeType(NodeType node_type) { node_type_ = node_type; }
  std::shared_ptr<SubscribeMsgBase<msg_type>> sub_handle_;
  NodeType node_type_;
};

// 为什么这里四个都要写？
// 在这个函数中，typename sub_ros2_handle 和 typename sub_dds_handle
// 是模板函数的模板参数， 虽然在该函数的具体实现中没有使用它们，但是根据 C++
// 的语法规定，它们必须在函数模板的声明中显式指定。 因为 C++
// 编译器在编译模板函数时需要知道所有模板参数的具体类型，即使在函数的实现中没有使用它们。
#ifdef ENABLE_ROS1
template <typename msg_type,
          typename sub_ros1_handle,
          typename sub_ros2_handle,
          typename sub_dds_handle>
void SubscribeMsgWrapper<msg_type,
                         sub_ros1_handle,
                         sub_ros2_handle,
                         sub_dds_handle>::
    InitROS1(const std::shared_ptr<ros::NodeHandle>& handle_ptr,
             const std::string& topic_name,
             const std::string& msg_info,
             uint32_t queue_size) {
  SetNodeType(ROS1_NODE);
  sub_handle_ = std::make_shared<sub_ros1_handle>(handle_ptr, topic_name,
                                                  msg_info, queue_size);
  topic_name_ = topic_name;
}
#endif  // ENABLE_ROS

#ifdef ENABLE_ROS2
template <typename msg_type,
          typename sub_ros1_handle,
          typename sub_ros2_handle,
          typename sub_dds_handle>
void SubscribeMsgWrapper<
    msg_type,
    sub_ros1_handle,
    sub_ros2_handle,
    sub_dds_handle>::InitROS2(const std::shared_ptr<rclcpp::Node>& rcl_node,
                              const std::string& topic_name,
                              const std::string& msg_info,
                              const rclcpp::QoS& qos) {
  SetNodeType(ROS2_NODE);
  sub_handle_ =
      std::make_shared<sub_ros2_handle>(rcl_node, topic_name, msg_info, qos);
  topic_name_ = topic_name;
}
#endif

#ifdef ENABLE_DDS
template <typename msg_type,
          typename sub_ros1_handle,
          typename sub_ros2_handle,
          typename sub_dds_handle>
void SubscribeMsgWrapper<msg_type,
                         sub_ros1_handle,
                         sub_ros2_handle,
                         sub_dds_handle>::InitDDS(uint32_t domain_id,
                                                  const std::string& topic_name,
                                                  const std::string& msg_info,
                                                  bool zcc_flag, const std::string& network_interface) {
  if (zcc_flag) {
    SetNodeType(DDS_ZCC_NODE);
  } else {
    SetNodeType(DDS_NET_NODE);
  }

  sub_handle_ = std::make_shared<sub_dds_handle>(topic_name, msg_info,
                                                 domain_id, zcc_flag, network_interface);
}
#endif  // ENABLE_DDS

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // SUBSCRIBEMSGWRAPPER_H
