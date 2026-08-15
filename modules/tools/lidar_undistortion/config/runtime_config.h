#ifndef LIDAR_UNDISTORT_RUNTIME_CONFIG_H
#define LIDAR_UNDISTORT_RUNTIME_CONFIG_H

#pragma once

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace tools {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  std::string lidar_calib_file_path  = "";
  std::string camera_calib_file_path = "";
};

}  // namespace tools
}  // namespace jojo

#endif
