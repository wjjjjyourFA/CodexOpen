#ifndef CONFIG_FILE_YAML_H
#define CONFIG_FILE_YAML_H

#pragma once

#include <iostream>
#include <string>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace jojo {
namespace common {
namespace config {

class ConfigFileYaml {
 public:
  ConfigFileYaml()          = default;
  virtual ~ConfigFileYaml() = default;

  virtual void LoadConfig(const std::string& config_path) = 0;

  void set_name(const std::string& name) { this->name_ = name; }

  const std::string& get_name() const { return this->name_; }

 protected:
  std::string name_ = "";

  Eigen::Vector3d Vec3FromYaml(const YAML::Node& node);
};

}  // namespace config
}  // namespace common
}  // namespace jojo

#endif  // CONFIG_FILE_YAML_H
