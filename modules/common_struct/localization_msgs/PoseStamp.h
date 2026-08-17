#pragma once

#include <cstdint>
#include <string>

#include "modules/common_struct/basic_msgs/Header.h"
#include "modules/common_struct/basic_msgs/Pose.h"

// namespace cstruct = jojo::common_struct;

namespace jojo {
namespace common_struct {

struct PoseStamped {
  // odom 数据本身提供的时间戳
  Header header;

  // 何时收到的 odom 数据，来源于当前系统时间
  std::uint64_t recv_timestamp{0};

  Pose pose;

  // 构造函数
  PoseStamped() = default;  // 所有成员会按照它们各自的默认构造规则来初始化
  PoseStamped(const Header& h, const std::uint64_t& ts, const Pose& p)
      : header(h), recv_timestamp(ts), pose(p) {}
};

/* PoseStamped() 默认构造出来的对象是完全干净的：
  header.timestamp = 0, header.seq = 0, header.frame_id = ""
  recv_timestamp = 0
  pose.position 三个值为 0.0，pose.orientation 为单位四元数
*/

}  // namespace common_struct
}  // namespace jojo
