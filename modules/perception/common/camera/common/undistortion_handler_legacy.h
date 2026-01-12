#pragma once

#include <string>

#include <opencv2/opencv.hpp>

// #include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/distortion_model.h"
// #include "modules/perception/common/base/image.h"
#include "modules/perception/common/base/fisheye_model.h"

#include "modules/perception/common/camera/params/camera_params.h"

namespace jojo {
namespace perception {
namespace camera {
namespace base = jojo::perception::base;

enum class CameraDistortionModel : uint {
  None    = 0,
  Brown   = 1,
  Kannala = 2,
};

class UndistortionHandler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  UndistortionHandler(
      CameraDistortionModel mode = CameraDistortionModel::Brown) {
    inited_ = false;

    // clang-format off
    switch (mode) {
      case CameraDistortionModel::Brown:
        distort_model = std::make_shared<apollo::perception::base::BrownCameraDistortionModel>();
        type = CameraDistortionModel::Brown;
        std::cout << "name : " << distort_model->name() << std::endl;
        break;
      case CameraDistortionModel::Kannala:
        distort_model = std::make_shared<base::KannalaCameraDistortionModel>();
        type = CameraDistortionModel::Kannala;
        std::cout << "name : " << distort_model->name() << std::endl;
        break;
      default:
        distort_model = nullptr;
        break;
    }
    // clang-format on
  }

  ~UndistortionHandler() { Release(); }

  // bool set_device(int device);

  // clang-format off
  bool InitParams(size_t width, size_t height, double k1, double k2, double k3, double p1, double p2, 
                  double fx, double fy, double cx, double cy);
  bool InitParams(size_t width, size_t height, double k1, double k2, double k3, double k4, 
                  double fx, double fy, double cx, double cy);
  bool InitParams(size_t width, size_t height, const Eigen::VectorXf& params);
  // clang-format on

  // bool Init(const std::string &sensor_name, int device);
  bool Init(const std::string& sensor_name);

  // 整数映射表，不同于cv::initUndistortRectifyMap的浮点数映射表
  // 主要是为了加速运算，但无法再双线性差值，并且可能像素锯齿
  // camera_model ==> intrinsic_params
  // distortion ==> distort_params
  virtual void InitUndistortRectifyMapSimple(
      const Eigen::Matrix3f& intrinsic_params,
      const Eigen::Matrix<float, 8, 1>& distort_params, int width, int height);

  virtual void InitFisheyeUndistortRectifyMapSimple(
      const Eigen::Matrix3f& intrinsic_params,
      const Eigen::Matrix<float, 8, 1>& distort_params, int width, int height);

  // bool Handle(const base::Image8U &src_img, base::Image8U *dst_img);
  bool Handle(const cv::Mat& src_img, cv::Mat* dst_img);
  bool Handle(const cv::Mat& src_img, cv::Mat& dst_img);
  // @brief: Release the resources
  bool Release(void);

  std::shared_ptr<apollo::perception::base::BaseCameraDistortionModel>
      distort_model;
  CameraDistortionModel type = CameraDistortionModel::None;

  // bool UndistortSimple(const cv::Mat &src_img, cv::Mat &dst_img, bool show);

  void GetRectifiedIntrinsic(std::shared_ptr<CameraMatrix> matrix);

 protected:
  cv::Mat cameraMatrix, distCoeffs;
  // 等效针孔模型的内参
  cv::Mat newCameraMatrix;

  void UpdateIntrinsicParams(const Eigen::Matrix3f& intrinsic_params,
                             const Eigen::Matrix<float, 8, 1>& distort_params);

  void UpdateFisheyeIntrinsicParams(
      const Eigen::Matrix3f& intrinsic_params,
      const Eigen::Matrix<float, 8, 1>& distort_params);

 private:
  std::pair<double, double> ApplyDistortion(double x, double y, double k1,
                                            double k2, double k3, double k4,
                                            double k5, double k6, double p1,
                                            double p2);

  std::pair<double, double> ApplyFisheyeDistortion(double x, double y,
                                                   double k1, double k2,
                                                   double k3, double k4);

  virtual bool CorrectImage(const cv::Mat& src_img, cv::Mat& dst_img);

  // clang-format off
  // std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> uv_map_;
  // 使用 aligned_allocator 保证 uv_map_ 内部的 Eigen 对象对齐
  std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>,
              Eigen::aligned_allocator<std::pair<Eigen::Vector2i, Eigen::Vector2i>>> uv_map_;
  // clang-format on

  int width_   = 0;  // image cols
  int height_  = 0;  // image rows
  bool inited_ = false;
};

}  // namespace camera
}  // namespace perception
}  // namespace jojo
