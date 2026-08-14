#ifndef RADAR2CAMERA_CONFIG_INTERFACE_H
#define RADAR2CAMERA_CONFIG_INTERFACE_H

#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {

class InterfaceConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  bool b_compressed  = true;
  bool b_pointcloud2 = true;

  std::string image_topic = "";
  std::string radar_topic = "";

  int rate = 20;
};

}  // namespace perception
}  // namespace jojo

#endif
