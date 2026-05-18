#ifndef MAPPER_MAP_PARAMS_H
#define MAPPER_MAP_PARAMS_H

#pragma once

#include <fstream>

#include "modules/common/config/config_file_yaml.h"

namespace jojo {
namespace mapping {

class StaticConfig : public jojo::common::config::ConfigFileYaml {
 public:
  using jojo::common::config::ConfigFileYaml::ConfigFileYaml;

  void LoadConfig(const std::string& config_path) override;

  std::string map_save_path;

  bool b_use_pose_center = false;

 private:
  YAML::Node yaml;
};

}  // namespace mapping
}  // namespace jojo

#endif
