#ifndef DATA_PROCESSOR_CONFIG_H
#define DATA_PROCESSOR_CONFIG_H

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
  bool b_save_data;
  std::string save_path;
  int prepare_data_num   = -1;
  int sample_interval    = 1;
  int useless_time       = 0;  // s ==> ms
  float distance_epsilon = 1e-3;  // 小于这个就当作 0, 并移除
  int intensity_epsilon  = 1e-3;  // 小于这个就当作 0, 并移除

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
  std::vector<std::string> topic_radar_4d_sub;
};

}  // namespace tools
}  // namespace jojo

#endif  // DATA_PROCESSOR_CONFIG_H
