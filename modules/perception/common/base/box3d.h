#ifndef BOX3D_H_
#define BOX3D_H_
/* 这里是我对BBox2D的模仿拓展，用于替换8point格式，来源于 openpcdet
它是一个基础类型结构体，包含六个成员变量，分别是x、y、z、length、width、height
分别表示:
  左下角的x坐标、左下角的y坐标、左下角的z坐标 ==> 3号点
  长度\宽度\高度 ==> 0-->3 \ 0-->1 \ 0-->4 ==> k1 k0 k2 
  0-->6 k3
*/
//            7 -------- 4
//           /|         /|
//          6 -------- 5 .
//          | |     k3 | |
//          . 3 -------- 0
//        k2|/         |/k0
//          2 -------- 1
//               k1
#pragma once

#include <algorithm>
#include <sstream>
#include <string>

#include "modules/perception/common/base/comparison_traits.h"
#include "modules/perception/common/base/point.h"

namespace jojo {
namespace perception {
namespace base {

template <typename T>
struct BBox3D;

// x y z 是指立方体的左下角坐标
// 各点值，最小的点为0号点，最大的点为6号点 
template <typename T>
struct Cube {
  Cube() : x(0), y(0), z(0), length(0), width(0), height(0) {}

  Cube(const T &x_in, const T &y_in, const T &z_in, const T &length_in, const T &width_in, const T &height_in)
      : x(x_in), y(y_in), z(z_in), length(length_in), width(width_in), height(height_in) {}

  explicit Cube(const BBox3D<T> &bbox) {
    this->x      = bbox.xmin;
    this->y      = bbox.ymin;
    this->z      = bbox.zmin;
    this->length = bbox.xmax - bbox.xmin;
    this->width  = bbox.ymax - bbox.ymin;
    this->height = bbox.zmax - bbox.zmin;
  }

  Cube<T> &operator=(const BBox3D<T> &bbox) {
    this->x      = bbox.xmin;
    this->y      = bbox.ymin;
    this->z      = bbox.zmin;
    this->length = bbox.xmax - bbox.xmin;
    this->width  = bbox.ymax - bbox.ymin;
    this->height = bbox.zmax - bbox.zmin;
    return *this;
  }

  Point3D<T> Center() const {
    Point3D<T> p;
    p.x = this->x + this->length / 2;
    p.y = this->y + this->width / 2;
    p.z = this->z + this->height / 2;
    return p;
  }

  void SetCenter(Point3D<T> p) {
    this->x = p.x - this->length / 2;
    this->y = p.y - this->width / 2;
    this->z = p.z - this->height / 2;
  }

  T Volume() const { return this->length * this->width * this->height; }

  std::string ToStr() const {
    std::stringstream ss;
    ss << "[ " << length << " x " << width << " x " << height << " ] from ( " << x << " , " 
       << y << " , " << z << " )";
    return ss.str();
  }

  friend Cube<T> operator&(const Cube<T> &cube1, const Cube<T> &cube2) {
    T r1_xmin = cube1.x;
    T r1_xmax = cube1.x + cube1.length;
    T r1_ymin = cube1.y;
    T r1_ymax = cube1.y + cube1.width;
    T r1_zmin = cube1.z;
    T r1_zmax = cube1.z + cube1.height;
    T r2_xmin = cube2.x;
    T r2_xmax = cube2.x + cube2.length;
    T r2_ymin = cube2.y;
    T r2_ymax = cube2.y + cube2.width;
    T r2_zmin = cube2.z;
    T r2_zmax = cube2.z + cube2.height;
    if (r2_xmin <= r1_xmax && r2_xmax >= r1_xmin && r2_ymin <= r1_ymax &&
        r2_ymax >= r1_ymin && r2_zmin <= r1_zmax && r2_zmax >= r1_zmin) {
      T xmin = std::max(r1_xmin, r2_xmin);
      T ymin = std::max(r1_ymin, r2_ymin);
      T zmin = std::max(r1_zmin, r2_zmin);
      T xmax = std::min(r1_xmax, r2_xmax);
      T ymax = std::min(r1_ymax, r2_ymax);
      T zmax = std::min(r1_zmax, r2_zmax);
      return Cube<T>(xmin, ymin, zmin, xmax - xmin, ymax - ymin, zmax - zmin);
    } else {
      return Cube<T>(0, 0, 0, 0, 0, 0);
    }
  }

  friend Cube<T> operator|(const Cube<T> &cube1, const Cube<T> &cube2) {
    Cube<T> ret;
    ret.x = std::min(cube1.x, cube2.x);
    ret.y = std::min(cube1.y, cube2.y);
    ret.z = std::min(cube1.z, cube2.z);
    ret.length = 
        std::max(cube1.x + cube1.length, cube2.x + cube2.length) - ret.x;
    ret.width = std::max(cube1.y + cube1.width, cube2.y + cube2.width) - ret.y;
    ret.height =
        std::max(cube1.z + cube1.height, cube2.z + cube2.height) - ret.z;

    return ret;
  }

  friend inline bool operator==(const Cube &cube1, const Cube &cube2) {
    return (Equal(cube1.x, cube2.x) && Equal(cube1.y, cube2.y) && Equal(cube1.z, cube2.z) &&
            Equal(cube1.length, cube2.length) &&
            Equal(cube1.width, cube2.width) &&
            Equal(cube1.height, cube2.height));
  }

  friend inline bool operator!=(const Cube &cube1, const Cube &cube2) {
    return !(cube1 == cube2);
  }

  // 原BBox2D基于图像坐标，这里需要转换到世界坐标
  T x = 0;  // ==> 0号点
  T y = 0;
  T z = 0;
  T length = 0;
  T width = 0;
  T height = 0;
};

template <typename T>
struct BBox3D {
  BBox3D() : xmin(0), ymin(0), zmin(0), xmax(0), ymax(0), zmax(0) {}

  BBox3D(const T &xmin_in, const T &ymin_in, const T &zmin_in, const T &xmax_in, const T &ymax_in, const T &zmax_in)
      : xmin(xmin_in), ymin(ymin_in), zmin(zmin_in), xmax(xmax_in), ymax(ymax_in), zmax(zmax_in) {}

  explicit BBox3D(const Cube<T> &cube) {
    this->xmin = cube.x;
    this->ymin = cube.y;
    this->zmin = cube.z;
    this->xmax = cube.x + cube.length;
    this->ymax = cube.y + cube.width;
    this->zmax = cube.z + cube.height;
  }

  BBox3D<T> &operator=(const Cube<T> &cube) {
    this->xmin = cube.x;
    this->ymin = cube.y;
    this->zmin = cube.z;
    this->xmax = cube.x + cube.length;
    this->ymax = cube.y + cube.width;
    this->zmax = cube.z + cube.height;
    return *this;
  }

  Point3D<T> Center() const {
    Point3D<T> p;
    p.x = this->xmin + (this->xmax - this->xmin) / 2;
    p.y = this->ymin + (this->ymax - this->ymin) / 2;
    p.z = this->zmin + (this->zmax - this->zmin) / 2;
    return p;
  }

  T Volume() const { return (xmax - xmin) * (ymax - ymin) * (zmax - zmin); }
  // 原BBox2D基于图像坐标，这里需要转换到世界坐标
  T xmin = 0;  // ==> 0号点
  T ymin = 0;
  T zmin = 0;
  T xmax = 0;  // ==> 6号点
  T ymax = 0;
  T zmax = 0;
};

typedef Cube<int> CubeI;
typedef Cube<float> CubeF;
typedef Cube<double> CubeD;

typedef BBox3D<int> BBox3DI;
typedef BBox3D<float> BBox3DF;
typedef BBox3D<double> BBox3DD;

}  // namespace base
}  // namespace perception
}  // namespace jojo

#endif  // BOX3D_H_