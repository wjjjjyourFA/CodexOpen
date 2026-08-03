#ifndef GROUND_REMOVE_RUNTIME_CONFIG_H
#define GROUND_REMOVE_RUNTIME_CONFIG_H

#pragma once

#include <Eigen/Core>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  // 地面参考高度
  float ref_height = 0;
  float hanging_z  = 2.0;
  // 车轮半高 <== 点云是车辆中心的坐标系，小于该值 认为是地面一下的点
  // 主要用于设置偏移值
  float mean_z_thresh  = -0.55;
  float delta_z_thresh = 0.1;

  // m
  int dist_threshold = 100;

  float map_resolution = 0.2;

  int32_t map_rows = 512;
  int32_t map_cols = 512;

  Eigen::Vector3f min_bound;
  Eigen::Vector3f max_bound;

  float far_resolution    = 0.5;
  float middle_resolution = 0.4;
  float near_resolution   = 0.2;

  std::string camera_calib_file_path = "";
  std::string gravity_lidar_calib_file_path;

  bool b_use_legacy   = false;
  bool b_use_gaussian = false;
  bool b_gravity      = false;

  bool b_show_color_point = false;

  // 车体感知的边界距离，单位 m
  // perception bound ==> front left right back
  Eigen::Vector4f perception_bound = Eigen::Vector4f(50, 40, 40, 30);
};

Eigen::Vector3f ReadVec3(const boost::property_tree::ptree& pt,
                         const std::string& key,
                         const Eigen::Vector3f& default_val);

}  // namespace perception
}  // namespace jojo

#endif
