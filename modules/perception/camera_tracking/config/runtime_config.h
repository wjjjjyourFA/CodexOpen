#ifndef CAMERA_TRACKING_RUNTIME_CONFIG_H
#define CAMERA_TRACKING_RUNTIME_CONFIG_H

#pragma once

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {
namespace ct {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  std::string det_wts_file, sort_wts_file;
  std::string det_engine_file, sort_engine_file;
  std::string det_onnx_file, sort_onnx_file;

  bool b_undistort;
  std::string calib_file_path = "";
};

}  // namespace ct
}  // namespace perception
}  // namespace jojo

#endif
