/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
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
