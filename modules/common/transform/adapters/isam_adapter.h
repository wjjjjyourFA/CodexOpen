#pragma once

#include <array>

#include "modules/common/transform/geometry/rotation_conversions.h"

namespace isam {
// ISAM 数学坐标系

using TransformArray = std::array<float, 6>;

void Ro2Ang(const Eigen::Matrix3d& R, Eigen::Vector3d& angle);

void Transform2Tr(float* transform, Eigen::Matrix4f& RT);

Eigen::Matrix4f Transform2Tr(const TransformArray& transform);

void Tr2Transform(const Eigen::Matrix4f& RT, float* transform);

TransformArray Tr2Transform(const Eigen::Matrix4f& RT);

}  // namespace isam
