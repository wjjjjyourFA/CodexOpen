#ifndef DATA_LOADER_RUNTIME_CONFIG_H
#define DATA_LOADER_RUNTIME_CONFIG_H

#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace tools {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  std::string root_path;
  std::string file_name;
  int64_t start_time, end_time, current_time;
  float pause_time;

  bool use_bin_or_pcd, use_txt_or_pcd;
  int use_jpg_or_png;

  bool b_compensation_cloud;
  int b_lt_none_rt;

  bool b_do_undistort;
  std::string calib_file_dir;

  std::vector<int> compress_params;

  bool b_imu_vec;
  bool b_pose_vec;

  std::string lidar_type;
  std::vector<std::string> camera_name;
  std::vector<std::string> infra_name;
  std::vector<std::string> star_name;
  std::string radar_type;
  std::string radar4d_type;
};

}  // namespace tools
}  // namespace jojo

#endif  // DATA_LOADER_RUNTIME_CONFIG_H
