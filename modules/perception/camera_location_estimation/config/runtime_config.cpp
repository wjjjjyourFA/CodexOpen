#include "modules/perception/camera_location_estimation/config/runtime_config.h"

#include <iostream>

namespace jojo {
namespace perception {
namespace cle {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  valid = false;
  validation_error.clear();
  try {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);
    wts_file    = pt.get<std::string>("general.wts_file", "");
    engine_file = pt.get<std::string>("general.engine_file", "");
    onnx_file   = pt.get<std::string>("general.onnx_file", "");

    use_det_or_track = pt.get<int>("general.use_det_or_track", 1);

    b_undistort     = pt.get<bool>("general.b_undistort", false);
    calib_file_path = pt.get<std::string>("calibration.calib_file_path", "");
    dist_threshold  = pt.get<int>("general.dist_threshold", 100);

    // clang-format off
    location.scale = pt.get<float>("location.roi_scale", location.scale);
    location.eps = pt.get<float>("location.cluster_epsilon", location.eps);
    location.minPts = pt.get<int>("location.cluster_min_points", location.minPts);
    location.pixel_threshold = pt.get<int>("location.minimum_roi_points", location.pixel_threshold);
    location.projection_epsilon = pt.get<float>("location.projection_epsilon", location.projection_epsilon);
    // clang-format on

    if (!ParseInferenceMode(use_det_or_track, &inference_mode)) {
      validation_error = "general.use_det_or_track must be 1 or 2";
      std::cerr << validation_error << std::endl;
      return;
    }
    valid = Validate(&validation_error);
    if (!valid) std::cerr << validation_error << std::endl;
  } catch (const std::exception& e) {
    validation_error = std::string("Error reading ini file: ") + e.what();
    std::cerr << validation_error << std::endl;
  }
}

bool RuntimeConfig::Validate(std::string* error) const {
  if (engine_file.empty()) {
    if (error) *error = "general.engine_file must not be empty";
    return false;
  }
  if (calib_file_path.empty()) {
    if (error) *error = "calibration.calib_file_path must not be empty";
    return false;
  }
  if (dist_threshold <= 0) {
    if (error) *error = "general.dist_threshold must be positive";
    return false;
  }
  return location.Validate(error);
}

}  // namespace cle
}  // namespace perception
}  // namespace jojo
