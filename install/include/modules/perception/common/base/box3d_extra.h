#ifndef BOX3D_EXTRA_H
#define BOX3D_EXTRA_H
//            7 -------- 4
//           /|         /|
//          6 -------- 5 .
//          | |     k3 | |
//          . 3 -------- 0
//        k2|/         |/k0
//          2 -------- 1
//               k1

#pragma once

#include <vector>

#include <Eigen/Dense>

#include "modules/perception/common/base/box3d.h"
#include "modules/perception/common/base/comparison_traits.h"
#include "modules/perception/common/base/point.h"

namespace jojo {
namespace perception {
namespace base {

struct BBox3DExtra {
  BBox3DExtra() : box3d{0, 0, 0, 0, 0, 0}, length(0), width(0), height(0) {
    UpdateCorners();
  }

  // 8 points 3x8=24
  // float box[24] = {0};
  jojo::perception::base::BBox3DF box3d;

  // clang-format off
  BBox3DExtra(float x_min, float y_min, float z_min, 
              float x_max, float y_max, float z_max)
      : box3d{x_min, y_min, z_min, x_max, y_max, z_max},
        length(x_max - x_min),
        width(y_max - y_min),
        height(z_max - z_min) {
    UpdateCorners();
  }

  explicit BBox3DExtra(const jojo::perception::base::BBox3DF& box)
      : BBox3DExtra(box.xmin, box.ymin, box.zmin, box.xmax, box.ymax, box.zmax) {}

  /* 参数量一样，导致编译器无法区分
  explicit BBox3DExtra(float x_min, float y_min, float z_min, 
                       float length_in, float width_in, float height_in)
      : BBox3DExtra(x_min, y_min, z_min, 
                    x_min + length_in, y_min + width_in, z_min + height_in) {}
  */
  // clang-format on

  Point3DF corners[8];

  float length = 0;
  float width  = 0;
  float height = 0;

  float k[4] = {0};
  float b[4] = {0};

  void UpdateCorners() {
    const float& xmin = box3d.xmin;
    const float& ymin = box3d.ymin;
    const float& zmin = box3d.zmin;
    const float& xmax = box3d.xmax;
    const float& ymax = box3d.ymax;
    const float& zmax = box3d.zmax;

    // bottom face
    corners[2] = {xmin, ymin, zmin};
    corners[1] = {xmax, ymin, zmin};
    corners[3] = {xmin, ymax, zmin};
    corners[0] = {xmax, ymax, zmin};

    // top face
    corners[6] = {xmin, ymin, zmax};
    corners[5] = {xmax, ymin, zmax};
    corners[7] = {xmin, ymax, zmax};
    corners[4] = {xmax, ymax, zmax};
  }

  void Reset() {
    box3d.Reset();
    length = width = height = 0;
    std::fill(k, k + 4, 0.f);
    std::fill(b, b + 4, 0.f);
    UpdateCorners();
  }
};

// 3D Box 结构体  Center-Size
// template <typename T>
// 需要计算旋转，所以用float，取消模板
struct BBox3DRotated {
  Eigen::Vector3f center;  // 物体中心点 (x, y, z)
  Eigen::Vector3f dimensions;  // 尺寸 (长, 宽, 高) 不一定和轴对齐
  Eigen::Quaternionf orientation;  // 朝向：使用四元数表示旋转

  BBox3DRotated()
      : center(Eigen::Vector3f::Zero()),
        dimensions(Eigen::Vector3f::Zero()),
        orientation(Eigen::Quaternionf::Identity()) {}  // (w,x,y,z)=(1,0,0,0)

  BBox3DRotated(const Eigen::Vector3f& center,
                const Eigen::Vector3f& dimensions,
                const Eigen::Quaternionf& orientation)
      : center(center), dimensions(dimensions), orientation(orientation) {}

  std::vector<Point3D<float>> Vertices() const {
    std::vector<Point3D<float>> vertices;
    vertices.reserve(8);

    // 计算每个维度的一半长度
    float hlx = dimensions.x() / 2.0f;  // half_length_x
    float hly = dimensions.y() / 2.0f;  // half_length_y
    float hlz = dimensions.z() / 2.0f;  // half_length_z

    // 计算每个顶点的坐标
    // 八个顶点相对中心的偏移量
    // clang-format off
    const std::vector<Eigen::Vector3f> offsets = {
        { hlx,  hly, -hlz},  // 0
        { hlx, -hly, -hlz},  // 1
        {-hlx, -hly, -hlz},  // 2
        {-hlx,  hly, -hlz},  // 3

        { hlx,  hly,  hlz},  // 4
        { hlx, -hly,  hlz},  // 5
        {-hlx, -hly,  hlz},  // 6
        {-hlx,  hly,  hlz}   // 7
    };
    // clang-format on

    // 根据中心点计算绝对坐标
    // 将偏移量通过旋转矩阵变换并加到中心点
    Eigen::Matrix3f rotation_matrix = orientation.toRotationMatrix();
    for (const auto& offset : offsets) {
      Eigen::Vector3f transformed_point = rotation_matrix * offset + center;
      vertices.push_back(Point3D<float>{
          transformed_point.x(), transformed_point.y(), transformed_point.z()});
    }
    return vertices;
  }

  // 计算3D Box的体积
  float Volume() const {
    return dimensions.x() * dimensions.y() * dimensions.z();
  }

  // Sets the orientation using a quaternion.
  void SetOrientation(const Eigen::Quaternionf& new_orientation) {
    orientation = new_orientation.normalized();
  }

  // Sets the orientation using Euler angles (yaw, pitch, roll). ZYX
  void SetOrientationFromEuler(float yaw, float pitch, float roll) {
    orientation = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                  Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                  Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
  }

  // Returns the rotation matrix corresponding to the box's orientation.
  Eigen::Matrix3f GetRotationMatrix() const {
    return orientation.toRotationMatrix();
  }

  // Returns the yaw (rotation around the Z-axis) of the box.
  float GetYaw() const {
    return std::atan2(2.0f * (orientation.w() * orientation.z() +
                              orientation.x() * orientation.y()),
                      1.0f - 2.0f * (orientation.y() * orientation.y() +
                                     orientation.z() * orientation.z()));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Converts a rotation matrix to a quaternion.
inline Eigen::Quaternionf MatrixToQuaternion(
    const Eigen::Matrix3f& rotation_matrix) {
  return Eigen::Quaternionf(rotation_matrix);
}

// Creates a quaternion from Euler angles (yaw, pitch, roll).
inline Eigen::Quaternionf EulerToQuaternion(float yaw, float pitch,
                                            float roll) {
  return Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
}

// #### for BBox3DRotated #### //
// Rotates a Box3D by a specified angle around the Z-axis.
inline void RotateBox(BBox3DRotated& box, float angle) {
  Eigen::Quaternionf rotation(
      Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
  // Apply the rotation to the current orientation
  // way 1 绕世界 Z轴 旋转
  // box.orientation = rotation * box.orientation;
  // box.orientation = (rotation * box.orientation).normalized();
  // way 2 绕物体自身 Z轴 旋转
  box.orientation = (box.orientation * rotation).normalized();
}

// Calculates the Euclidean distance between the centers of two Box3D objects.
inline float Distance(const BBox3DRotated& box1, const BBox3DRotated& box2) {
  return (box1.center - box2.center).norm();
}

}  // namespace base
}  // namespace perception
}  // namespace jojo

#endif  // BOX3D_EXTRA_H
