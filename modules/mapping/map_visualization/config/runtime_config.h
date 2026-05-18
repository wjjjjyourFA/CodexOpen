#ifndef MAP_VISUALIZATION_PARAMS_H
#define MAP_VISUALIZATION_PARAMS_H

#pragma once

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace mapping {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  std::string root_path;
  std::string file_name;

  bool b_generate_label_gray = false;
  bool b_generate_label_color  = false;

  double half_length     = 1.0;
  double max_search_dist = 20.0;

  float line_width; 

  std::string lidar_calib_file_path  = "";
  std::string camera_calib_file_path = "";
};

}  // namespace mapping
}  // namespace jojo

#endif
