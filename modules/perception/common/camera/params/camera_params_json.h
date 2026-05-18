#ifndef READ_CAMERA_PARAMS_JSON_H
#define READ_CAMERA_PARAMS_JSON_H

#pragma once

#include "modules/common/config/config_file_json.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/config/utils.h"

namespace jojo {
namespace perception {
namespace camera {

class CameraParamsJson : public CameraParams {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraParamsJson() {};
  virtual ~CameraParamsJson() = default;

  // set ext = ".json" for using json file
  // void LoadFromName(const std::string& camera_name,
  //                   const std::string& ext = ".ini");

 protected:
  // clang-format off
  bool LoadFromFileBase(const std::string& filename, 
                        Eigen::Matrix3f& intrinsic_matrix,
                        Eigen::Matrix<float, 8, 1>& distortion_params,
                        Eigen::Matrix4f& extrinsic_matrix,
                        Eigen::Matrix4f& projection_matrix) override;
  // clang-format on
};

}  // namespace camera
}  // namespace perception
}  // namespace jojo

#endif