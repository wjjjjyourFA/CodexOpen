#ifndef MAP_CENTER_VIEW_MAP_STATIC_CONFIG_H
#define MAP_CENTER_VIEW_MAP_STATIC_CONFIG_H

#pragma once

#include <fstream>

#include "modules/common/config/config_file_yaml.h"
// #include "modules/common/config/config_file_json.h"

namespace jojo {
namespace dreamview {

class StaticConfig : public jojo::common::config::ConfigFileYaml {
 public:
  using jojo::common::config::ConfigFileYaml::ConfigFileYaml;

  void LoadConfig(const std::string& config_path) override;

  std::string map_file_path;
  // std::vector<double> map_center;
  // Eigen::Vector3d map_center;

  // double yaw_offset_euler;  // 手动微调 初始 yaw
  // double yaw_offset_rad;  // 由 yaw_offset_euler 转换而来

  // int init_method    = 0;

  bool b_use_pose_center = false;

  bool b_display_roi  = false;
  bool b_display_body = false;

 protected:
  void LoadJson(const std::string& config_path);

 private:
  YAML::Node yaml;

  // void SwitchMethod(int method);
  // std::string map_info_file = "";

 public:
  std::vector<double> pose_offset;
};

// ===== RuntimeConfig 映射 =====
// void from_json(const nlohmann::json& j, StaticConfig& c);

}  // namespace dreamview
}  // namespace jojo

#endif
