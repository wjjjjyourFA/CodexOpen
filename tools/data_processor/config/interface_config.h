#ifndef DATA_PROCESSOR_INTERFACE_CONFIG_H
#define DATA_PROCESSOR_INTERFACE_CONFIG_H

#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace tools {

class InterfaceConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  bool b_local_pose{false};
  bool b_global_pose{false};
  bool b_imu_data{false};
  bool b_lidar{false};
  int b_radar{0};
  int b_radar4d{0};
  int b_camera{0};
  int b_infra{0};
  int b_star{0};

  std::string topic_local_pose_sub;
  std::string topic_global_pose_sub;
  std::string topic_imu_data_sub;
  std::string topic_pose_sub;

  bool b_difop{false};
  std::string topic_lidar_sub;
  std::string topic_lidar_ori_sub, topic_lidar_difop_sub;

  bool b_compressed{false};
  std::vector<std::string> topic_camera_sub;
  std::vector<std::string> topic_infra_sub;
  std::vector<std::string> topic_star_sub;

  std::string topic_radar_sub;
  std::vector<std::string> topic_radar4d_sub;
};

}  // namespace tools
}  // namespace jojo

#endif  // DATA_PROCESSOR_INTERFACE_CONFIG_H
