#ifndef CAMERA_TRACKING_INTERFACE_CONFIG_H
#define CAMERA_TRACKING_INTERFACE_CONFIG_H

#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {
namespace ct {

class InterfaceConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  bool b_compressed;
  std::string image_topic = "";

  int rate = 10;
};

}  // namespace ct
}  // namespace perception
}  // namespace jojo

#endif
