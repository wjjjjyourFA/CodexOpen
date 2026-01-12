#include "modules/perception/common/camera/common/undistortion_handler_cv.h"

#include <vector>

#include "Eigen/Dense"
#include "cyber/common/log.h"
// #include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
// #include "modules/perception/common/camera/common/image_data_operations.h"

#ifdef _OPENMP
// CPU 并行计算
#include <omp.h>
// 防止忘记设置 线程数，可在环境变量中声明
// export OMP_NUM_THREADS=8
#include <atomic>
#endif

namespace jojo {
namespace perception {
namespace camera {

void UndistortionHandlerCv::InitUndistortRectifyMapSimple(
    const Eigen::Matrix3f& intrinsic_params,
    const Eigen::Matrix<float, 8, 1>& distort_params, int width, int height) {
  cameraMatrix = cv::Mat::eye(3, 3, CV_32F);
  // 1. 构建相机内参矩阵 K (float)
  cameraMatrix.at<float>(0, 0) = intrinsic_params(0, 0);  // fx
  cameraMatrix.at<float>(1, 1) = intrinsic_params(1, 1);  // fy
  cameraMatrix.at<float>(0, 2) = intrinsic_params(0, 2);  // cx
  cameraMatrix.at<float>(1, 2) = intrinsic_params(1, 2);  // cy

  distCoeffs = cv::Mat::zeros(1, 8, CV_32F);
  // cv::undistort 的标准畸变系数顺序 [k1, k2, p1, p2, k3, k4, k5, k6]
  for (int i = 0; i < 5; ++i) {
    distCoeffs.at<float>(0, i) = distort_params(i, 0);
  }

  newCameraMatrix = cameraMatrix.clone();

  cv::Size dim(width, height);

  cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                              newCameraMatrix, dim, CV_16SC2, map1, map2);
}

void UndistortionHandlerCv::InitFisheyeUndistortRectifyMapSimple(
    const Eigen::Matrix3f& intrinsic_params,
    const Eigen::Matrix<float, 8, 1>& distort_params, int width, int height) {
  // std::cout << "[UndistortionHandlerCv] InitFisheyeUndistortRectifyMapSimple() called" << std::endl;

  cameraMatrix = cv::Mat::eye(3, 3, CV_32F);
  // 1. 构建相机内参矩阵 K (float)
  cameraMatrix.at<float>(0, 0) = intrinsic_params(0, 0);  // fx
  cameraMatrix.at<float>(1, 1) = intrinsic_params(1, 1);  // fy
  cameraMatrix.at<float>(0, 2) = intrinsic_params(0, 2);  // cx
  cameraMatrix.at<float>(1, 2) = intrinsic_params(1, 2);  // cy

  distCoeffs = cv::Mat::zeros(1, 4, CV_32F);
  // cv::fish::undistort 的标准畸变系数顺序 [k1, k2, k3, k4]
  for (int i = 0; i < 4; ++i) {
    distCoeffs.at<float>(0, i) = distort_params(i, 0);
    std::cout << " " << distort_params(i, 0) << std::endl;
  }

  newCameraMatrix = cameraMatrix.clone();

  cv::Size dim(width, height);

  cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                       newCameraMatrix, dim, CV_16SC2, map1,
                                       map2);
}

bool UndistortionHandlerCv::CorrectImage(const cv::Mat& src_img,
                                         cv::Mat& dst_img) {
  if (src_img.empty() || map1.empty() || map2.empty()) {
    return false;
  }

  cv::remap(src_img, dst_img, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
            cv::Scalar());

  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace jojo
