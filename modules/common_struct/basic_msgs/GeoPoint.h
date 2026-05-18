// # 对应数值使用时 x1e-3 以获得实际值
// # x1e-7
// int32 longitude    # Longitude 经度（-180 至 180）
// int32 latitude     # Latitude 纬度（-90 至 90） x1e7 degree

// # x1e-2
// int32 altitude     # Altitude 海拔高度，单位（cm）

#pragma once

namespace jojo {
namespace common_struct {
  
struct GeoPoint {
  double longitude;  // deg
  double latitude;  // deg
  double altitude;  // meters
};

}  // namespace common_struct
}  // namespace jojo