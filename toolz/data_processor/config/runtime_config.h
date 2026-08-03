#ifndef DATA_PROCESSOR_RUNTIME_CONFIG_DATASET_H
#define DATA_PROCESSOR_RUNTIME_CONFIG_DATASET_H

#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

#include "modules/common/config/config_file_json.h"

namespace jojo {
namespace tools {

class RuntimeConfig : public jojo::common::config::ConfigFileJson {
 public:
  using jojo::common::config::ConfigFileJson::ConfigFileJson;

  void LoadConfig(const std::string& config_path) override;

 public:
  std::string rosbag_path;
  std::string rosbag_name;
  bool b_save_data;
  std::string save_path;
  int prepare_data_num   = -1;
  int sample_interval    = 1;
  int useless_time       = 0;  // s ==> ms
  float distance_epsilon = 1e-3;  // 小于这个就当作 0, 并移除
  int intensity_epsilon  = 1e-3;  // 小于这个就当作 0, 并移除

  bool use_bin_or_pcd, use_txt_or_pcd;
  int use_jpg_or_png;

  bool b_compensation_cloud;
  int b_lt_none_rt;

  bool b_do_undistort;
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

// ===== RuntimeConfig 映射 =====
void from_json(const nlohmann::json& j, RuntimeConfig& c);

}  // namespace tools
}  // namespace jojo

#endif  // DATA_PROCESSOR_RUNTIME_CONFIG_DATASET_H
