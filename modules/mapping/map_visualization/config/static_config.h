#ifndef MAP_VISUALIZATION_STATIC_CONFIG_H
#define MAP_VISUALIZATION_STATIC_CONFIG_H

#pragma once

#include <fstream>

#include "modules/common/config/config_file_json.h"
#include "modules/common/config/config_file_yaml.h"

namespace jojo {
namespace mapping {

class StaticConfig : public jojo::common::config::ConfigFileYaml {
 public:
  using jojo::common::config::ConfigFileYaml::ConfigFileYaml;

  void LoadConfig(const std::string& config_path) override;

  // 这里是已经生成的 map 的固有属性
  std::string map_file_path;
  // 单位 m
  double map_min_x, map_min_y;
  float map_resolution = 0.2;
  // 单位 pixel
  int width, height;

  // std::string map_label_path;
  // std::string map_cube_path;

  bool b_use_pose_center = false;

 protected:
  void LoadJson(const std::string& config_path);

 private:
  YAML::Node yaml;

  std::string map_info_file = "";
};

void from_json(const nlohmann::json& j, StaticConfig& c);

}  // namespace mapping
}  // namespace jojo

#endif
