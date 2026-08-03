#ifndef FAST_LIO_STATIC_CONFIG_H
#define FAST_LIO_STATIC_CONFIG_H

#pragma once

#include <fstream>

#include "modules/common/config/config_file_json.h"
#include "modules/common/config/config_file_yaml.h"
#include "modules/localization/fast_lio/include/preprocess.h"

namespace jojo {
namespace localization {

class StaticConfig : public jojo::common::config::ConfigFileYaml {
 public:
  using jojo::common::config::ConfigFileYaml::ConfigFileYaml;

  void LoadConfig(const std::string& config_path) override;

  std::string map_file_path;
  // std::vector<double> map_center;
  Eigen::Vector3d map_center;

  double yaw_offset_euler;  // 手动微调 初始 yaw
  double yaw_offset_rad;  // 由 yaw_offset_euler 转换而来

  int init_method    = 0;
  bool b_chaos_start = false;  // 随机初始位置
  // 20260427：未启用
  bool b_hand_start = false;
  std::vector<double> hand_init_pose;

  bool b_init_twice = true;

  bool b_display_init = true;

  bool feature_enabled = false;

  int lidar_type       = 1;
  int point_filter_num = 5;

  double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;

  // lasermap_fov_segment
  float DET_RANGE     = 100.0f;
  float MOV_THRESHOLD = 1.5f;
  double fov_deg      = 0;
  double cube_len     = 200;

  // map_incremental
  double filter_size_surf_min, filter_size_map_min;

  // update_iterated_dyn_share_modified
  bool extrinsic_est_en = false;

  int NUM_MAX_ITERATIONS = 5;

  // preprocess
  double blind;
  int N_SCANS;
  int time_unit;
  int SCAN_RATE;

 protected:
  void LoadJson(const std::string& config_path);

 private:
  YAML::Node yaml;

  void SwitchMethod(int method);
  std::string map_info_file = "";

 public:
  std::vector<double> pose_offset;
};

// ===== RuntimeConfig 映射 =====
void from_json(const nlohmann::json& j, StaticConfig& c);

}  // namespace localization
}  // namespace jojo

#endif
