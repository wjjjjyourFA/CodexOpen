#ifndef CAMERA_LOCATION_ESTIMATION_RUNTIME_CONFIG_H
#define CAMERA_LOCATION_ESTIMATION_RUNTIME_CONFIG_H

#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {
namespace cle {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  std::string wts_file;
  std::string engine_file;
  std::string onnx_file;

  bool b_undistort;
  std::string calib_file_path = "";

  int use_det_or_track;

  int dist_threshold = 100;
};

}  // namespace cle
}  // namespace perception
}  // namespace jojo

#endif
