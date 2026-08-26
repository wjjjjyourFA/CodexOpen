#ifndef CAMERA_LOCATION_ESTIMATION_RUNTIME_CONFIG_H
#define CAMERA_LOCATION_ESTIMATION_RUNTIME_CONFIG_H

#include <string>

#include "modules/common/config/config_file_base.h"
#include "modules/perception/camera_location_estimation/common.h"

namespace jojo {
namespace perception {
namespace cle {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;
  void LoadConfig(const std::string& config_path) override;
  bool Validate(std::string* error = nullptr) const;

  std::string wts_file;
  std::string engine_file;
  std::string onnx_file;

  bool b_undistort = false;
  std::string calib_file_path;

  int use_det_or_track = 1;  // 兼容现有 ini

  int dist_threshold = 100;

  ImageLocationHyperparams location;
  InferenceMode inference_mode = InferenceMode::kDetection;

  bool valid = false;
  std::string validation_error;
};

}  // namespace cle
}  // namespace perception
}  // namespace jojo
#endif
