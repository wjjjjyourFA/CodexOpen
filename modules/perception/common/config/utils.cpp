#include "modules/perception/common/config/utils.h"

namespace jojo {
namespace perception {
namespace config {

Eigen::Matrix4f ComputeExtrinsicMatrix(
    const Eigen::Matrix3f &intrinsic_matrix,
    const Eigen::Matrix4f &projection_matrix) {
  // 假设你已经知道了相机内参矩阵 K、和投影矩阵 P 的值
  // 使用相机内参矩阵 K 进行分解
  // Eigen::Matrix3d Kd = K.cast<double>();
  Eigen::Matrix3f intrinsic_matrix_inv = intrinsic_matrix.inverse();

  Eigen::Matrix<float, 3, 4> proj_3x4 = projection_matrix.block<3, 4>(0, 0);

  // 计算外参 [R | t] = K⁻¹ * P
  Eigen::Matrix<float, 3, 4> extrinsic_3x4 = intrinsic_matrix_inv * proj_3x4;

  // 构造 4x4 齐次外参矩阵
  // 第四行这个数字是求不出来的，本来就是增广手动补上去的
  // 重新计算P阵时，第四行也没有被用来计算 extrinsic_matrix.row(3) << 0, 0, 0, 1;
  Eigen::Matrix4f extrinsic_matrix   = Eigen::Matrix4f::Identity();
  extrinsic_matrix.block<3, 4>(0, 0) = extrinsic_3x4;

  return extrinsic_matrix;
}

Eigen::VectorXf IntrinsicParamsToVector(
    const Eigen::Matrix3f &intrinsic_matrix,
    const Eigen::Matrix<float, 8, 1> &distortion_params) {
  // 创建一个长度为 9 + 8 = 17 的向量
  Eigen::VectorXf params(17);
  // clang-format off
  // 将 intrinsic_matrix 和 distortion_params 按照顺序放入 params 向量中
  params << intrinsic_matrix(0, 0), intrinsic_matrix(0, 1), intrinsic_matrix(0, 2), 
            intrinsic_matrix(1, 0), intrinsic_matrix(1, 1), intrinsic_matrix(1, 2), 
            intrinsic_matrix(2, 0), intrinsic_matrix(2, 1), intrinsic_matrix(2, 2), 
            distortion_params(0), distortion_params(1),
            distortion_params(2), distortion_params(3), 
            distortion_params(4),
            distortion_params(5), distortion_params(6), distortion_params(7);
  // clang-format on

  // std::cout << params << std::endl;

  return params;
}

Eigen::Matrix4f TransProjMatrixmm2m(const Eigen::Matrix3f &intrinsic_matrix,
                                    const Eigen::Matrix4f &p_matrix_mm) {
  // 假设你已经知道了相机内参矩阵 K、和投影矩阵 P 的值
  Eigen::Matrix4f extrinsic_matrix =
      ComputeExtrinsicMatrix(intrinsic_matrix, p_matrix_mm);

  // mm => mi
  extrinsic_matrix = TransRtMatrixmm2m(extrinsic_matrix);

  // 第四行这个数字是求不出来的，本来就是增广手动补上去的
  // 重新计算P阵时，第四行也没有被用来计算 projection_matrix.row(3) << 0, 0, 0, 1;
  Eigen::Matrix4f p_matrix_m = p_matrix_mm;
  p_matrix_m.block<3, 4>(0, 0) =
      intrinsic_matrix * extrinsic_matrix.block<3, 4>(0, 0);
  // 手动补上
  p_matrix_m.row(3) << 0, 0, 0, 1;

  return p_matrix_m;
}

Eigen::Matrix4f TransRtMatrixmm2m(const Eigen::Matrix4f &rt_matrix_mm) {
  // 假设你已经知道了相机内参矩阵 K、和旋转矩阵 RT 的值
  // 从旋转矩阵 RT 中提取旋转矩阵 R 和平移向量 t
  Eigen::Matrix4f rt_matrix_m = rt_matrix_mm;
  rt_matrix_m.block<3, 1>(0, 3) /= 1000.0f;

  return rt_matrix_m;
}

}  // namespace config
}  // namespace perception
}  // namespace jojo
