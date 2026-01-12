#pragma once

#include <string>

#include "modules/common_struct/basic_msgs/Header.h"
#include "modules/common_struct/basic_msgs/Pose6D.h"

namespace cstruct = jojo::common_struct;

struct PoseStamped {
  // odom 数据本身提供的时间戳
  cstruct::Header header;

  // 何时收到的 odom 数据，来源于当前系统时间
  uint64_t recv_timestamp{0};

  cstruct::Pose6D pose;

  // 构造函数
  PoseStamped() = default;  // 所有成员会按照它们各自的默认构造规则来初始化
  PoseStamped(const cstruct::Header& h, const uint64_t& ts,
              const cstruct::Pose6D& p)
      : header(h), recv_timestamp(ts), pose(p) {}
};

/* PoseStamped() 默认构造出来的对象是完全干净的：
  header.timestamp = 0, header.seq = 0, header.frame_id = ""
  recv_timestamp = 0
  pose 六个值都是 0.0
*/