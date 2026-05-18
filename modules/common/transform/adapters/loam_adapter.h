#pragma once

#include "modules/common/transform/geometry/rotation_conversions.h"

namespace loam {
// LOAM 数学坐标系

void Ro2Ang(const Eigen::Matrix3d& R, Eigen::Vector3d& angle);

void Transform2NormalTr(float* transform_, Eigen::Matrix4f& RT);

void NormalTr2Transform(const Eigen::Matrix4f& RT, float* transform_);

}  // namespace loam