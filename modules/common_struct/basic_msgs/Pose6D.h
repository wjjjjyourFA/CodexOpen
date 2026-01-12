// # This represents a vector in free space.
// # x1e-2
// int32 x          # in cm
// int32 y
// int32 z

// # 为了联通CAN总线，推算采用 0.001rad 为单位
// # 姿态角（单位：0.001rad）
// # 使用时 x1e-3 以获得实际值
// int32 azimuth    # 偏航角（-180 至 180) 实际是 0.001rad
// int32 pitch      # 俯仰角（-90 至 90)
// int32 roll       # 横滚角（-180 至 180）

#pragma once

namespace jojo {
namespace common_struct {
  
struct Pose6D {
  double x{};
  double y{};
  double z{};
  double azimuth{};  // yaw
  double pitch{};
  double roll{};

  Pose6D() = default;
  Pose6D(double _x, double _y, double _z, double _azimuth, double _pitch,
         double _roll)
      : x(_x), y(_y), z(_z), azimuth(_azimuth), pitch(_pitch), roll(_roll) {}
};

}  // namespace common_struct
}  // namespace jojo