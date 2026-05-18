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
#ifndef  DISTORTION_MODEL_H
#define  DISTORTION_MODEL_H

#pragma once

#include <memory>
#include <string>
#include <iostream>

#include "modules/perception/common/base/camera.h"

namespace apollo {
namespace perception {
namespace base {

// @brief: base class for camera distortion model
class BaseCameraDistortionModel {
 public:
  BaseCameraDistortionModel() = default;
  virtual ~BaseCameraDistortionModel() = default;

  // @brief: project a point from camera space to image plane
  // @params[IN] point3d: 3d point in camera space;
  // @return: 2d point in image plane
  // @note: the input point should be in front of the camera,
  //        i.e. point3d[2] > 0
  virtual Eigen::Vector2f Project(const Eigen::Vector3f& point3d) = 0;

  virtual std::shared_ptr<BaseCameraModel> get_camera_model() = 0;
  virtual std::string name() const = 0;
  virtual bool set_params(size_t width, size_t height,
                          const Eigen::VectorXf& params) = 0;

  size_t get_height() const { return height_; }
  size_t get_width() const { return width_; }

  inline Eigen::Matrix3f get_intrinsic_params() const {
    return intrinsic_params_;
  }

  inline Eigen::Matrix<float, 8, 1> get_distort_params() const {
    return distort_params_;
  }

 protected:
  size_t width_ = 0;
  size_t height_ = 0;

  /*
    fx  0   cx
    0   fy  cy
    0   0   1
  */
  // clang-format off
  Eigen::Matrix3f intrinsic_params_ = Eigen::Matrix3f::Zero();
  Eigen::Matrix<float, 8, 1> distort_params_ = Eigen::Matrix<float, 8, 1>::Zero();
  // clang-format on
};

/* TODO(all): to remove
typedef std::shared_ptr<BaseCameraDistortionModel> BaseCameraDistortionModelPtr;
typedef std::shared_ptr<const BaseCameraDistortionModel>
    BaseCameraDistortionModelConstPtr;
*/

class BrownCameraDistortionModel : public BaseCameraDistortionModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  BrownCameraDistortionModel() = default;
  ~BrownCameraDistortionModel() = default;

  Eigen::Vector2f Project(const Eigen::Vector3f& point3d) override;

  std::shared_ptr<BaseCameraModel> get_camera_model() override;

  std::string name() const override { return "BrownCameraDistortionModel"; }

  bool set_params(size_t width, size_t height,
                  const Eigen::VectorXf& params) override;

 protected:
  // k1, k2, p1, p2, k3
};

using BrownCameraDistortionModelPtr =
    std::shared_ptr<BrownCameraDistortionModel>;

using BrownCameraDistortionModelConstPtr =
    std::shared_ptr<const BrownCameraDistortionModel>;

}  // namespace base
}  // namespace perception
}  // namespace apollo

#endif  //  DISRTION_MODEL_H
