#ifndef DATA_PROCESSOR_INTERFACE_CONFIG_DATASET_H
#define DATA_PROCESSOR_INTERFACE_CONFIG_DATASET_H

#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

#include "modules/common/config/config_file_json.h"

namespace jojo {
namespace tools {

class InterfaceConfig : public jojo::common::config::ConfigFileJson {
 public:
  using jojo::common::config::ConfigFileJson::ConfigFileJson;

  void LoadConfig(const std::string& config_path) override;

 public:
  bool b_local_pose, b_global_pose, b_imu_data;
  bool b_lidar;
  int b_radar, b_radar4d;
  int b_camera, b_infra, b_star;

  std::string topic_local_pose_sub;
  std::string topic_global_pose_sub;
  std::string topic_imu_data_sub;
  std::string topic_pose_sub;

  bool b_difop;
  std::string topic_lidar_sub;
  std::string topic_lidar_ori_sub, topic_lidar_difop_sub;

  bool b_compressed;
  std::vector<std::string> topic_camera_sub;
  std::vector<std::string> topic_infra_sub;
  std::vector<std::string> topic_star_sub;

  std::string topic_radar_sub;
  std::vector<std::string> topic_radar4d_sub;
};

// ===== InterfaceConfig 映射 =====
void from_json(const nlohmann::json& j, InterfaceConfig& c);

}  // namespace tools
}  // namespace jojo

#endif  // DATA_PROCESSOR_INTERFACE_CONFIG_DATASET_H
