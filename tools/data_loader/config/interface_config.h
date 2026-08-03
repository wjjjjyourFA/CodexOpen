#ifndef DATA_LOADER_INTERFACE_CONFIG_H
#define DATA_LOADER_INTERFACE_CONFIG_H

#pragma once

#include <iostream>
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
  bool b_local_pose, b_global_pose, b_imu_data;
  bool b_lidar;
  int b_radar, b_radar4d;
  int b_camera, b_infra, b_star;

  std::string topic_local_pose_pub;
  std::string topic_global_pose_pub;
  std::string topic_imu_data_pub;
  std::string topic_pose_pub;

  bool b_difop;
  std::string topic_lidar_pub;
  std::string topic_lidar_ori_pub, topic_lidar_difop_pub;

  bool b_undistort;
  std::vector<std::string> topic_camera_pub;
  std::vector<std::string> topic_infra_pub;
  std::vector<std::string> topic_star_pub;

  std::string topic_radar_pub;
  std::vector<std::string> topic_radar4d_pub;
};

}  // namespace tools
}  // namespace jojo

#endif
