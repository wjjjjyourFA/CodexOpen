#ifndef DATA_LOADER_INTERFACE_CONFIG_H
#define DATA_LOADER_INTERFACE_CONFIG_H

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

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

  std::string topic_local_pose_pub;
  std::string topic_global_pose_pub;
  std::string topic_imu_data_pub;
  std::string topic_pose_pub;

  bool b_difop{false};
  std::string topic_lidar_pub;
  std::string topic_lidar_ori_pub, topic_lidar_difop_pub;

  bool b_undistort{false};
  std::vector<std::string> topic_camera_pub;
  std::vector<std::string> topic_infra_pub;
  std::vector<std::string> topic_star_pub;

  std::string topic_radar_pub;
  std::vector<std::string> topic_radar4d_pub;
};

}  // namespace tools
}  // namespace jojo

#endif
