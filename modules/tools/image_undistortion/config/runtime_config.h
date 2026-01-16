#ifndef IMAGE_UNDISTORT_PARAMS_H
#define IMAGE_UNDISTORT_PARAMS_H

#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace tools {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;
  
  void LoadResolveFile(const std::string& file);

  void InitPrefixPath();

 public:
  std::string resolve_file    = "";
  std::string calib_file_path = "";
  std::string data_root_path  = "";
  bool b_matched              = false;
};

}  // namespace tools
}  // namespace jojo

#endif
