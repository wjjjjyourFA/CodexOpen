#pragma once

namespace jojo {
namespace common_struct {

struct Vector2f {
  Vector2f(float x_ = 0, float y_ = 0) 
      : x(x_), y(y_) {}

  float x;
  float y;
};

// # This represents a vector in free space.
// int32 x
// int32 y
// int32 z

// ### example  GaussPoint
// # Warning 默认高斯坐标 WGS84 坐标系
// # x1e-2
// # int32 x         # in cm
// # int32 y         # in cm

// # 用于地图对齐、SLAM等用途
// # 高斯本身并没有z坐标
// # x1e-2
// # int32 z         # in cm

struct Vector3f {
  Vector3f(float x_ = 0, float y_ = 0, float z_ = 0) 
      : x(x_), y(y_), z(z_) {}

  float x;
  float y;
  float z;
};

struct Vector4f {
  Vector4f(float x_ = 0, float y_ = 0, float z_ = 0, float w_ = 0) 
      : x(x_), y(y_), z(z_), w(w_) {}

  float x;
  float y;
  float z;
  float w;
};

}  // namespace common_struct
}  // namespace jojo