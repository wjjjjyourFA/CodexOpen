// # 为了联通CAN总线，推算采用 0.001rad 为单位
// # 姿态角（单位：0.001rad）
// # 使用时 x1e-3 以获得实际值
// int32 yaw      # z 偏航角，单位：0.001rad
// int32 pitch    # y 俯仰角，单位：0.001rad
// int32 roll     # x 横滚角，单位：0.001rad

#pragma once

namespace jojo {
namespace common_struct {

struct EulerAngles {  // rad
  double yaw;
  double pitch;
  double roll;
};

}  // namespace common_struct
}  // namespace jojo