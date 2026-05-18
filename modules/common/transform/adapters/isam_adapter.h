#pragma once

#include "modules/common/transform/geometry/rotation_conversions.h"

namespace isam {
// ISAM 数学坐标系

void Ro2Ang(const Eigen::Matrix3d& R, Eigen::Vector3d& angle);

void Transform2Tr(float* transform, Eigen::Matrix4f& RT);

void Tr2Transform(const Eigen::Matrix4f& RT, float* transform);

}  // namespace isam