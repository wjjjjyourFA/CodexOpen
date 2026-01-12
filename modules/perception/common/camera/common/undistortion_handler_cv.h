#pragma once

#include <string>

#include <opencv2/opencv.hpp>

// #include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/distortion_model.h"
// #include "modules/perception/common/base/image.h"
#include "modules/perception/common/camera/common/undistortion_handler_legacy.h"

namespace jojo {
namespace perception {
namespace camera {
// namespace base = apollo::perception::base;

class UndistortionHandlerCv : public UndistortionHandler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 默认使用 Brown 模型，也可以传参
  explicit UndistortionHandlerCv(
      CameraDistortionModel mode = CameraDistortionModel::Brown)
      : UndistortionHandler(mode) {
    std::cout << " use opencv undistortion " << std::endl;
  }

  ~UndistortionHandlerCv() { Release(); }

  void InitUndistortRectifyMapSimple(
      const Eigen::Matrix3f& intrinsic_params,
      const Eigen::Matrix<float, 8, 1>& distort_params, int width,
      int height) override;

  void InitFisheyeUndistortRectifyMapSimple(
      const Eigen::Matrix3f& intrinsic_params,
      const Eigen::Matrix<float, 8, 1>& distort_params, int width,
      int height) override;

 protected:
  // 部分已在父类中声明
  // cv::Mat cameraMatrix, distCoeffs;
  // cv::Mat newCameraMatrix;
  cv::Mat map1, map2;

 private:
  bool CorrectImage(const cv::Mat& src_img, cv::Mat& dst_img) override;
};

}  // namespace camera
}  // namespace perception
}  // namespace jojo
