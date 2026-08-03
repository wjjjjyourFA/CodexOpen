#ifndef RADAR2CAMERA_RUNTIME_CONFIG_H
#define RADAR2CAMERA_RUNTIME_CONFIG_H

#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  std::string camera_calib_file_path = "";
  std::string radar_calib_file_path  = "";

  bool b_do_undistort = 0;

  // m
  int dist_threshold = 1000;
};

}  // namespace perception
}  // namespace jojo

#endif
