#ifndef BOX3D_EXTRA_H_
#define BOX3D_EXTRA_H_
//            7 -------- 4
//           /|         /|
//          6 -------- 5 .
//          | |     k3 | |
//          . 3 -------- 0
//        k2|/         |/k0
//          2 -------- 1
//               k1

#pragma once

#include <Eigen/Dense>
#include <vector>

// #include "modules/perception/common/base/comparison_traits.h"
#include "modules/perception/common/base/point.h"

namespace jojo {
namespace perception {
namespace base {

struct BBox3DSimple {
  // 8 points 3x8=24
  float box[24] = {0};
  float k[4]    = {0};
  float b[4]    = {0};

  Point3DF p[8];

  float length;
  float width;
  float height;

  void set_cube(float x /*min_x*/, float y /*min_y*/, float z /*min_z*/,
                float l, float w, float h) {
    p[0] = {x, y, z};
    p[1] = {x, y + w, z};
    p[2] = {x + l, y + w, z};
    p[3] = {x + l, y, z};
    p[4] = {x, y, z + h};
    p[5] = {x, y + w, z + h};
    p[6] = {x + l, y + w, z + h};
    p[7] = {x + l, y, z + h};

    length = l;
    width  = w;
    height = h;
  }

  void get_vertex() {
    /* way 1
    for (int i = 0; i < 8; i++) {
      p[i].x = box[i * 3 + 0];
      p[i].y = box[i * 3 + 1];
      p[i].z = box[i * 3 + 2];
    }
    */
    float* box_ptr = box;  // 指向 box[0]
    for (auto& vertex : p) {
      vertex.x = *box_ptr++;
      vertex.y = *box_ptr++;
      vertex.z = *box_ptr++;
    }
  }

  void update_box_8points() {
    float* box_ptr = box;
    for (const auto& vertex : p) {
      *box_ptr++ = vertex.x;
      *box_ptr++ = vertex.y;
      *box_ptr++ = vertex.z;
    }
  }
};

// 3D Box 结构体  Center-Size
// template <typename T>
// 需要计算旋转，所以用float，取消模板
struct BBox3DRotated {
  Eigen::Vector3f center;  // 物体中心点 (x, y, z)
  Eigen::Matrix3f dimensions;  // 尺寸 (长, 宽, 高) 不一定和轴对齐
  Eigen::Quaternionf orientation;  // 朝向：使用四元数表示旋转

  BBox3DRotated()
      : center(Eigen::Vector3f::Zero()),
        dimensions(Eigen::Matrix3f::Zero()),
        orientation(Eigen::Quaternionf::Identity()) {}  // (w,x,y,z)=(1,0,0,0)

  BBox3DRotated(const Eigen::Vector3f& center,
                const Eigen::Matrix3f& dimensions,
                const Eigen::Quaternionf& orientation)
      : center(center), dimensions(dimensions), orientation(orientation) {}

  std::vector<Point3D<float>> Vertices() const {
    std::vector<Point3D<float>> vertices;
    vertices.reserve(8);

    // 计算每个维度的一半长度
    float hlx = dimensions(0) / 2.0f;  // half_length_x
    float hly = dimensions(1) / 2.0f;  // half_length_y
    float hlz = dimensions(2) / 2.0f;  // half_length_z

    // 计算每个顶点的坐标
    // 八个顶点相对中心的偏移量
    // clang-format off
    std::vector<Eigen::Vector3f> offsets = {
        {-hlx, -hly, -hlz},  // bottom-back-right
        {-hlx,  hly, -hlz},  // bottom-back-left
        { hlx,  hly, -hlz},  // bottom-front-left
        { hlx, -hly, -hlz},  // bottom-front-right
        {-hlx, -hly,  hlz},  // top-back-right
        {-hlx,  hly,  hlz},  // top-back-left
        { hlx,  hly,  hlz},  // top-front-left
        { hlx, -hly,  hlz}   // top-front-right
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
  float Volume() const { return dimensions(0) * dimensions(1) * dimensions(2); }

  // Sets the orientation using a quaternion.
  void SetOrientation(const Eigen::Quaternionf& new_orientation) {
    orientation = new_orientation;
  }

  // Sets the orientation using Euler angles (yaw, pitch, roll).
  void SetOrientationFromEuler(float yaw, float pitch, float roll) {
    orientation = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                  Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()) *
                  Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitY());
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
         Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()) *
         Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitY());
}

// #### for BBox3DRotated #### //
// Rotates a Box3D by a specified angle around the Z-axis.
inline void RotateBox(BBox3DRotated& box, float angle) {
  Eigen::Quaternionf rotation(
      Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
  box.orientation =
      rotation *
      box.orientation;  // Apply the rotation to the current orientation
}

// Calculates the Euclidean distance between the centers of two Box3D objects.
inline float Distance(const BBox3DRotated& box1, const BBox3DRotated& box2) {
  return (box1.center - box2.center).norm();
}

}  // namespace base
}  // namespace perception
}  // namespace jojo

#endif  // BOX3D_EXTRA_H_
