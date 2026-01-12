/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <limits>
#include <memory>
#include <vector>

namespace jojo /*apollo*/ {
namespace perception {
namespace base {

// 当结构体大小为16字节时，数据访问可能更优化。因此，使用16字节对齐。
// 即使部分点云处理并不需要intensity，也可以使用PointXYZI结构体，只是intensity值为0。
template <typename T>
struct alignas(16) Point {
  T x = 0;
  T y = 0;
  T z = 0;
  typedef T Type;

  Point(T x_ = 0, T y_ = 0, T z_ = 0)
      : x(x_), y(y_), z(z_) {}

  Point& operator+=(const Point& other) {
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
    return *this;
  }
};

template <typename T>
struct alignas(16) PointXYZI : public Point<T> {
  T intensity = 0;

  PointXYZI(T x_ = 0, T y_ = 0, T z_ = 0, T intensity_ = 0)
      : Point<T>(x_, y_, z_), intensity(intensity_) {}

  PointXYZI& operator+=(const PointXYZI& other) {
    Point<T>::operator+=(other);  // 调用基类 += 处理 x,y,z
    this->intensity = (this->intensity + other.intensity) / 2;  // 取均值
    return *this;
  }
};
/* fast trans but unsafe */
// std::vector<PointXYZI> points_b;
// std::vector<PointXYZ> points_a(points_b.size());
// std::memcpy(points_a.data(), points_b.data(), points_b.size() * sizeof(PointXYZ));

// 如果父类的 alignas(16) 已经声明，那么子类默认也会有 16 字节的对齐。
template <typename T>
struct PointXYZIT : public PointXYZI<T> {
  double timestamp = 0.0;
};

template <typename T>
struct PointXYZITH : public PointXYZIT<T> {
  float height = std::numeric_limits<float>::max();
};

template <typename T>
struct PointXYZITHB : public PointXYZITH<T> {
  int32_t beam_id = -1;
};

template <typename T>
struct PointXYZITHBL : public PointXYZITHB<T> {
  uint8_t label = 0;
};

using PointF = Point<float>;
using PointI = Point<int>;
using PointD = Point<double>;

using PointXYZIF = PointXYZI<float>;
using PointXYZID = PointXYZI<double>;
using PointXYZITF = PointXYZIT<float>;
using PointXYZITD = PointXYZIT<double>;
using PointXYZITHF = PointXYZITH<float>;
using PointXYZITHD = PointXYZITH<double>;
using PointXYZITHBF = PointXYZITHB<float>;
using PointXYZITHBD = PointXYZITHB<double>;
using PointXYZITHBLF = PointXYZITHBL<float>;
using PointXYZITHBLD = PointXYZITHBL<double>;

const std::size_t kDefaultReservePointNum = 50000;

struct PointIndices {
  PointIndices() { indices.reserve(kDefaultReservePointNum); }

  std::vector<int> indices;

  typedef std::shared_ptr<PointIndices> Ptr;
  typedef std::shared_ptr<const PointIndices> ConstPtr;
};

template <typename T>
struct alignas(16) PointRGB {
  T x = 0;
  T y = 0;
  T z = 0;
  typedef T Type;

  unsigned int r = 0;
  unsigned int g = 0;
  unsigned int b = 0;

  void init_random() {
    x = (rand() % 40);
    y = (rand() % 40);
    z = (rand() % 40);
  }
};

template <typename T>
struct alignas(16) Point2D {
  T x = 0;
  T y = 0;
  typedef T Type;

  Point2D(T x_ = 0, T y_ = 0) : x(x_), y(y_) {}

  // 重载 == 运算符
  bool operator==(const Point2D<T>& other) const {
    return (x == other.x) && (y == other.y);
  }

  // 不等于运算符
  bool operator!=(const Point2D<T>& other) const { return !(*this == other); }
};

using Point2DF = Point2D<float>;
using Point2DI = Point2D<int>;
using Point2DD = Point2D<double>;

/**** struct of the base point ****/
// 3D坐标点一般表示
template <typename T>
struct alignas(16) Point3D {
  T x = 0;
  T y = 0;
  T z = 0;
  typedef T Type;

  Point3D(T x_ = 0, T y_ = 0, T z_ = 0) : x(x_), y(y_), z(z_) {}

  // 编译器能够对这些简单的运算符进行内联优化
  // 在大数据点云处理的场景下，operator+= 和 operator*= 的开销不会成为瓶颈

  Point3D& operator+=(const Point3D& other) {
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
    return *this;
  }

  // *= 操作符重载
  // 对于float类型，我们使用float倍数，对于int类型，我们保持整数的乘法
  Point3D& operator*=(T scalar) {
    if constexpr (std::is_floating_point<T>::value) {  // 如果T是float或double
      this->x *= scalar;
      this->y *= scalar;
      this->z *= scalar;
    } else if constexpr (std::is_integral<T>::value) {  // 如果T是int类型
      this->x *= static_cast<int>(scalar);
      this->y *= static_cast<int>(scalar);
      this->z *= static_cast<int>(scalar);
    }
    return *this;
  }
};

using Point3DF = Point3D<float>;
using Point3DI = Point3D<int>;
using Point3DD = Point3D<double>;

template <typename T>
struct alignas(16) RadarPoint {
  T x = 0;
  T y = 0;
  T z = 0;
  T velocity = 0;
  T comp_vel = 0;
  T rcs = 0;
  typedef T Type;

  RadarPoint(T x_ = 0, T y_ = 0, T z_ = 0, T velocity_ = 0)
      : x(x_), y(y_), z(z_), velocity(velocity_) {}

  // 重载 += 运算符
  RadarPoint& operator+=(const RadarPoint& other) {
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
    this->velocity += other.velocity;  // maybe ?
    return *this;
  }

  // 重载 *= 运算符（用于标量乘法）
  RadarPoint& operator*=(T scalar) {
    if constexpr (std::is_floating_point<T>::value) {  // 如果T是float或double
      this->x *= scalar;
      this->y *= scalar;
      this->z *= scalar;
      this->velocity *= scalar;
    } else if constexpr (std::is_integral<T>::value) {  // 如果T是int类型
      this->x *= static_cast<int>(scalar);
      this->y *= static_cast<int>(scalar);
      this->z *= static_cast<int>(scalar);
      this->velocity *= static_cast<int>(scalar);
    }
    return *this;
  }
};

using RadarPointF = RadarPoint<float>;
using RadarPointD = RadarPoint<double>;
using RadarPointXYZVRF = RadarPoint<float>;
using RadarPointXYZVRD = RadarPoint<double>;

}  // namespace base
}  // namespace perception
}  // namespace apollo
