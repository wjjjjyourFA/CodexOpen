#pragma once

#include <array>

#include "modules/common/transform/geometry/rotation_conversions.h"

namespace loam {
// LOAM 数学坐标系

using TransformArray = std::array<float, 6>;

void Ro2Ang(const Eigen::Matrix3d& R, Eigen::Vector3d& angle);

void Transform2NormalTr(float* transform_, Eigen::Matrix4f& RT);

Eigen::Matrix4f Transform2NormalTr(const TransformArray& transform);

void NormalTr2Transform(const Eigen::Matrix4f& RT, float* transform_);

TransformArray NormalTr2Transform(const Eigen::Matrix4f& RT);

}  // namespace loam
