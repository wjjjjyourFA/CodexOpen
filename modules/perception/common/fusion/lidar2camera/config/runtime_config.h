#ifndef LIDAR2CAMERA_CONFIG_H
#define LIDAR2CAMERA_CONFIG_H

#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;
  
  void InitPrefixPath();

 public:
  std::string data_root_path  = "./../data/PerceptionFuse";
  std::string calib_file_path = "";

  bool b_bin_or_pcd = 0;
  bool b_jpg_or_png = 0;
  bool b_undistort  = 0;
  int b_lt_none_rt  = 1;
  bool b_matched    = 0;

  std::string lidar_file = "";
  std::string image_file = "";

  // m
  int dist_threshold = 100; 
   
 private:
  std::string lidar_name = "";
  std::string image_name = "";
};

}  // namespace tools
}  // namespace jojo

#endif
