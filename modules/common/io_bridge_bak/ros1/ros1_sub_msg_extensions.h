#ifndef ROS1_SUB_MSG_EXTENSIONS_H
#define ROS1_SUB_MSG_EXTENSIONS_H
#include "cyber/io_bridge/subscribe_ros1_msg.h"

#ifdef ENABLE_ROS1

/* 20250909 旧设想：这是 sensor data wrapper 的前提
它应该只负责将 ROS1 消息转到规定的格式，例如坐标系前左上
它是输入输出接口，它不应该负责对数据进一步处理，例如去畸变，就不属于它处理
第一种方案直接透传，有点浪费
第二种方案预处理，装换到需要的格式 */

namespace jojo {
namespace cyber {
namespace io_bridge {


}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  //ENABLE_ROS1

#endif  // ROS1_SUB_MSG_EXTENSIONS_H
