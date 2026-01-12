#ifndef BASE_PARAMETER_UTILS_H
#define BASE_PARAMETER_UTILS_H

#pragma once

#include <iostream>

#include <Eigen/Dense>

namespace jojo {
namespace perception {
namespace config {

Eigen::Matrix4f ComputeExtrinsicMatrix(
    const Eigen::Matrix3f &intrinsic_matrix,
    const Eigen::Matrix4f &projection_matrix);

Eigen::VectorXf IntrinsicParamsToVector(
    const Eigen::Matrix3f &intrinsic_matrix,
    const Eigen::Matrix<float, 8, 1> &distortion_params);

Eigen::Matrix4f TransProjMatrixmm2m(const Eigen::Matrix3f &intrinsic_matrix,
                                    const Eigen::Matrix4f &p_matrix_mm);

Eigen::Matrix4f TransRtMatrixmm2m(const Eigen::Matrix4f &rt_matrix_mm);

}  // namespace config
}  // namespace perception
}  // namespace jojo

#endif  // BASE_PARAMETER_UTILS_H
