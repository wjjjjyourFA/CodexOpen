#ifndef CAMERA_DETECTION_SINGLE_STAGE_RUNTIME_CONFIG_H
#define CAMERA_DETECTION_SINGLE_STAGE_RUNTIME_CONFIG_H

#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {
namespace cdss {

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
};

}  // namespace cdss
}  // namespace perception
}  // namespace jojo

#endif
