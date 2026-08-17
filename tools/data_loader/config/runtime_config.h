#ifndef DATA_LOADER_RUNTIME_CONFIG_H
#define DATA_LOADER_RUNTIME_CONFIG_H

#pragma once

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

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
  int64_t start_time{0};
  int64_t end_time{0};
  int64_t current_time{0};
  float pause_time{1.0F};

  bool use_bin_or_pcd{false};
  bool use_txt_or_pcd{false};
  int use_jpg_or_png{0};

  bool b_compensation_cloud{false};
  int b_lt_none_rt{1};

  bool b_do_undistort{false};
  std::string calib_file_dir;

  std::vector<int> compress_params;

  bool b_imu_vec{false};
  bool b_pose_vec{false};

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
