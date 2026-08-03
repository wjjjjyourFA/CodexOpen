#ifndef GROUND_REMOVE_INTERFACE_CONFIG_H
#define GROUND_REMOVE_INTERFACE_CONFIG_H

#pragma once

#include <Eigen/Core>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {

class InterfaceConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  std::string pose_topic  = "";
  std::string lidar_topic = "";

  std::string map_topic = "";

  std::string map_frame = "";

  int rate = 10;
};

}  // namespace perception
}  // namespace jojo

#endif
