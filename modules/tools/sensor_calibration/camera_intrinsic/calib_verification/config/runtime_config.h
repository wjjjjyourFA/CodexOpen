#ifndef PARAMS_H
#define PARAMS_H

#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace tools {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;
  
  void InitPrefixPath();

 public:
  std::string data_root_path =
      "./../../../data/SensorCalibration/CalibrationHarp";

  bool b_jpg_or_png = 0;

  std::string image_file = "";

 private:
  std::string image_name = "";
};

}  // namespace tools
}  // namespace jojo

#endif
