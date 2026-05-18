// # 对应数值使用时 x1e-3 以获得实际值
// # x1e-2
// int32 east     # 东向速度 (cm/s)
// int32 north    # 北向速度 (cm/s)
// int32 up       # 垂直向上速度 (cm/s)

// int32 speed    # 速度总量，可选

// # 速度角 heading 与 姿态角 yaw 是不同的概念，使用时注意区分
// # 为了联通CAN总线，推算采用 0.001rad 为单位
// # 姿态角（单位：0.001rad）
// # 使用时 x1e-3 以获得实际值
// int32 heading  # 航向角（可选），单位：0.001rad

#pragma once

namespace jojo {
namespace common_struct {
  
struct GeoVelocity {
  double east;  // m/s
  double north;
  double up;
  double speed;  // m/s
};

}  // namespace common_struct
}  // namespace jojo