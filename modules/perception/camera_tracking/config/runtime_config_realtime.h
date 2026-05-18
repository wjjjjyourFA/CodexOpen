#ifndef PARAMS_REALTIME_H
#define PARAMS_REALTIME_H

#pragma once

#include <iostream>

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

  bool b_compressed;
  std::string image_topic = "";

  int rate = 10;
};

}
}  // namespace perception
}  // namespace jojo

#endif
