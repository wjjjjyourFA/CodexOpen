#ifndef PARAMS_H
#define PARAMS_H

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
  float map_resolution = 0.2;

  int32_t map_rows = 512;
  int32_t map_cols = 512;

  Eigen::Vector3f min_bound;
  Eigen::Vector3f max_bound;

  std::string camera_calib_file_path = "";

  bool b_show_color_point = false;
};

Eigen::Vector3f ReadVec3(const boost::property_tree::ptree& pt,
                         const std::string& key,
                         const Eigen::Vector3f& default_val);

}  // namespace perception
}  // namespace jojo

#endif
