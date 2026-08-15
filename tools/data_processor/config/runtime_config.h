#ifndef DATA_PROCESSOR_RUNTIME_CONFIG_H
#define DATA_PROCESSOR_RUNTIME_CONFIG_H

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
  std::string rosbag_path;
  std::string rosbag_name;
  bool b_save_data{false};
  std::string save_path;
  int prepare_data_num   = -1;
  int sample_interval    = 1;
  int useless_time       = 0;  // s ==> ms
  float distance_epsilon = 1e-3;  // 小于这个就当作 0, 并移除
  float intensity_epsilon = 1e-3F;  // 小于这个就当作 0, 并移除

  bool use_bin_or_pcd{false};
  bool use_txt_or_pcd{false};
  int use_jpg_or_png{0};

  bool b_compensation_cloud{false};
  int b_lt_none_rt{1};

  bool b_do_undistort{false};
  std::string calib_file_dir;
  std::vector<int> compress_params;

  std::string imu_type;
  std::string lidar_type;
  std::vector<std::string> camera_name;
  std::vector<std::string> infra_name;
  std::vector<std::string> star_name;
  std::string radar_type;
  std::string radar4d_type;
};

}  // namespace tools
}  // namespace jojo

#endif  // DATA_PROCESSOR_RUNTIME_CONFIG_H
