#include "Eignes/Dense"
#include "opencv2/opencv.hpp"

inline void solve_pnp(const std::vector<cv::Point3f> &lidar_points,
                      const std::vector<cv::Point2f> &image_points,
                      const Eigen::VectorXf &params) {
  // cv::Mat lidar_points_mat;
  std::vector<double> rv(3), tv(3);
  cv::Mat rvec(rv), tvec(tv);

  // clang-format off
  // K
  Eigen::Matrix3f intrinsic_params = Eigen::Matrix3f::Zero();
  // K1K2P1P2K3
  Eigen::Matrix<float, 8, 1> distort_params = Eigen::Matrix<float, 8, 1>::Zero();
  // clang-format on

  if (params.size() >= 14) {
    intrinsic_params_(0, 0) = params(0);
    intrinsic_params_(0, 1) = params(1);
    intrinsic_params_(0, 2) = params(2);
    intrinsic_params_(1, 0) = params(3);
    intrinsic_params_(1, 1) = params(4);
    intrinsic_params_(1, 2) = params(5);
    intrinsic_params_(2, 0) = params(6);
    intrinsic_params_(2, 1) = params(7);
    intrinsic_params_(2, 2) = params(8);

    distort_params_[0] = params[9];
    distort_params_[1] = params[10];
    distort_params_[2] = params[11];
    distort_params_[3] = params[12];
    distort_params_[4] = params[13];
  }

  // cv::Mat(lidar_points).convertTo(lidar_points_mat, CV_32F);
  // 使用 solvePnP 计算旋转向量和平移向量
  cv::solvePnP(lidar_points, cv::Mat(image_points), cv::Mat(intrinsic_matrix),
               cv::Mat(distortion_params), rvec, tvec);

  // 将旋转向量转换为旋转矩阵
  cv::Rodrigues(rvec, rot_r);
  // 设置平移矩阵
  rot_t = tvec;

  for (int m = 0; m < 3; m++) {
    std::cout << tvec.ptr<double>(0)[m] << std::endl;
  }

  // RT
  Eigen::Matrix4f extrinsic_matrix = Eigen::Matrix4f::Identity();

  // 将旋转矩阵和位移向量转换为 Eigen 格式
  Eigen::Matrix3f rot_r_eigen;
  cv::cv2eigen(rot_r, rot_r_eigen);
  Eigen::Vector3f rot_t_eigen;
  // clang-format off
  rot_t_eigen << tvec.ptr<double>(0)[0], 
                 tvec.ptr<double>(0)[1],
                 tvec.ptr<double>(0)[2];
  // clang-format on

  // 填充外参矩阵
  extrinsic_matrix.block<3, 3>(0, 0) = rot_r_eigen;
  extrinsic_matrix.block<3, 1>(0, 3) = rot_t_eigen;

  return extrinsic_matrix;
}
