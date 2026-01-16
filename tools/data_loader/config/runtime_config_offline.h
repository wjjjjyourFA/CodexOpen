#ifndef DATA_LOADER_CONFIG_OFFLINE_H
#define DATA_LOADER_CONFIG_OFFLINE_H

#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace tools {

class RuntimeConfigOffline : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  std::string root_path;
  std::string file_name;
  int64_t start_time, end_time, current_time;
  float pause_time;

  bool b_local_pose, b_global_pose, b_imu_data;
  bool b_lidar;
  int b_radar, b_radar_4d;
  int b_camera, b_infra, b_star;

  bool use_bin_or_pcd, use_txt_or_pcd;
  int use_jpg_or_png;

  bool b_compensation_cloud;
  int b_lt_none_rt;

  bool b_undistort;
  std::string calib_file_path;
  std::vector<int> compress_params;

  std::string lidar_type;
  std::vector<std::string> camera_name;
  std::vector<std::string> infra_name;
  std::vector<std::string> star_name;
  std::string radar_type;
  std::string radar_4d_type;
};

}  // namespace tools
}  // namespace jojo

#endif  // DATA_LOADER_CONFIG_OFFLINE_H
