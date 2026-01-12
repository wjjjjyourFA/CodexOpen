#include "modules/perception/common/base/fisheye_model.h"

#include "cyber/common/log.h"

namespace jojo {
namespace perception {
namespace base {

Eigen::Vector2f KannalaCameraDistortionModel::Project(
    const Eigen::Vector3f& point3d) {
  if (std::isless(point3d[2], 0.f)) {
    AERROR << "The input point (" << point3d
           << ") should be in front of the camera";
  }
  // distortion coefficients
  const float k1 = distort_params_[0];
  const float k2 = distort_params_[1];
  const float k3 = distort_params_[4];
  const float k4 = distort_params_[5];

  Eigen::Vector2f pt2d_img;
  // normalized
  const Eigen::Vector2f pt_normalized(point3d[0] / point3d[2],
                                      point3d[1] / point3d[2]);
  const float x_n = pt_normalized[0];
  const float y_n = pt_normalized[1];
  const float x_mul_x = x_n * x_n;
  const float y_mul_y = y_n * y_n;
  const float r = std::sqrt(x_mul_x + y_mul_y);
  if (r < 1e-8f) {  // 光心附近
    return Eigen::Vector2f(intrinsic_params_(0, 2), intrinsic_params_(1, 2));
  }
  const float theta  = std::atan(r);
  const float theta2 = theta * theta;
  const float theta4 = theta2 * theta2;
  const float theta6 = theta4 * theta2;
  const float theta8 = theta4 * theta4;

  // fisheye 多项式畸变
  const float theta_d =
      theta * (1.f + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);

  const float scale = theta_d / r;
  pt2d_img          = pt_normalized * scale;

  // transform to image coordinates
  const float fx = intrinsic_params_(0, 0);
  const float fy = intrinsic_params_(1, 1);
  const float cx = intrinsic_params_(0, 2);
  const float cy = intrinsic_params_(1, 2);
  pt2d_img[0] = fx * pt2d_img[0] + cx;
  pt2d_img[1] = fy * pt2d_img[1] + cy;

  return pt2d_img;
}

// 用于从派生类指针安全地向基类指针转换
// PinholeCameraModel 是 BaseCameraModel 的派生类
std::shared_ptr<base::BaseCameraModel>
KannalaCameraDistortionModel::get_camera_model() {
  // std::shared_ptr<base::PinholeCameraModel> camera_model(new base::PinholeCameraModel());
  auto camera_model = std::make_shared<base::PinholeCameraModel>();
  camera_model->set_width(width_);
  camera_model->set_height(height_);
  camera_model->set_intrinsic_params(intrinsic_params_);

  return std::dynamic_pointer_cast<base::BaseCameraModel>(camera_model);
}

bool KannalaCameraDistortionModel::set_params(size_t width, size_t height,
                                              const Eigen::VectorXf& params) {
  // Brown distortion model
  if (params.size() >= 13) {
    width_ = width;
    height_ = height;
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
    return true;
  }
  return false;
}

}  // namespace base
}  // namespace perception
}  // namespace jojo
