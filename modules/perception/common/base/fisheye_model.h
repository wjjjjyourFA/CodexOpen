#ifndef FISHEYE_MODEL_H
#define FISHEYE_MODEL_H

#pragma once

#include <memory>
#include <string>

#include "modules/perception/common/base/camera.h"
#include "modules/perception/common/base/distortion_model.h"

namespace jojo {
namespace perception {
namespace base {
namespace base = apollo::perception::base;

class KannalaCameraDistortionModel : public base::BaseCameraDistortionModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  KannalaCameraDistortionModel()  = default;
  ~KannalaCameraDistortionModel() = default;

  Eigen::Vector2f Project(const Eigen::Vector3f& point3d) override;

  std::shared_ptr<base::BaseCameraModel> get_camera_model() override;

  std::string name() const override { return "KannalaCameraDistortionModel"; }

  bool set_params(size_t width, size_t height,
                  const Eigen::VectorXf& params) override;

 protected:
  // k1, k2, k3, k4, k5, k6
};

using KannalaCameraDistortionModelPtr =
    std::shared_ptr<KannalaCameraDistortionModel>;

using KannalaCameraDistortionModelConstPtr =
    std::shared_ptr<const KannalaCameraDistortionModel>;

}  // namespace base
}  // namespace perception
}  // namespace jojo

#endif  //  FISHEYE_MODEL_H
